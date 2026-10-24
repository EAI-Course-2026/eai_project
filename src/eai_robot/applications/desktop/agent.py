"""Desktop chat coordinator. UI, cloud media and the existing harness stay separate."""
from copy import deepcopy
from threading import Event, RLock, Thread
import uuid

from eai_robot.applications.agent.session import is_stop
from eai_robot.applications.agent.speech import validate_wav
from eai_robot.policy.model_io.contracts import ProviderError


class DeskAgent:
    def __init__(self, session=None, speech=None, on_stop=None):
        self.session, self.speech, self.on_stop = session, speech, on_stop
        self.runtime_id = uuid.uuid4().hex
        self._lock, self._cancel = RLock(), Event()
        self._thread, self._closed, self._busy = None, False, False
        self._generation, self._audio = 0, None
        self._messages, self._tools = [], []
        self._status, self._error = 'idle', None
        self._notice = ''
        self._answer = None

    def snapshot(self):
        with self._lock:
            stages = getattr(self.speech, 'profiles', {'asr': None, 'tts': None}) if self.speech else {}
            return {'enabled': self.session is not None, 'speech_enabled': self.speech is not None,
                    'runtime_id': self.runtime_id,
                    'asr_enabled': 'asr' in stages, 'tts_enabled': 'tts' in stages,
                    'mode': 'dry_run', 'busy': self._busy, 'status': self._status,
                    'error_code': self._error, 'generation': self._generation,
                    'notice': self._notice,
                    'messages': deepcopy(self._messages[-20:]), 'tools': deepcopy(self._tools[-12:])}

    def _message(self, role, text):
        row = {'id': len(self._messages) + self._generation * 100, 'role': role, 'text': text[:32768]}
        self._messages.append(row)
        self._messages = self._messages[-20:]
        return row

    def model_event(self, event):
        with self._lock:
            if self._cancel.is_set() or not self._busy:
                return
            kind, data = event['type'], event['data']
            if kind == 'model_delta' and self._answer is not None:
                self._answer['text'] = (self._answer['text'] + data['text'])[:32768]
            elif kind in {'tool_start', 'tool_end'}:
                self._status = 'tool' if kind == 'tool_start' else 'thinking'
                self._tools.append({'tool': data['tool'], 'status': data.get('status', 'running')})
                self._tools = self._tools[-12:]

    def submit(self, *, text=None, audio=None, speak=True, expected_generation=None):
        if isinstance(text, str) and is_stop(text):
            self.stop()
            return
        if (type(speak) is not bool or (text is None) == (audio is None)
                or text is not None and (not isinstance(text, str) or not text.strip() or len(text) > 8192)):
            raise ValueError('invalid_input')
        if audio is not None:
            validate_wav(audio)
        with self._lock:
            if expected_generation is not None and (type(expected_generation) is not int or expected_generation != self._generation):
                raise ValueError('stale_input')
            if self.session is None or self._closed:
                raise ValueError('agent_not_configured')
            if audio is not None and (self.speech is None or 'asr' not in getattr(self.speech, 'profiles', {'asr': None})):
                raise ValueError('speech_not_configured')
            if self._busy:
                raise ValueError('busy')
            self._generation += 1
            generation = self._generation
            self._cancel = Event()
            self._busy, self._error, self._audio = True, None, None
            self._notice = ''
            self._status = 'recognizing' if audio is not None else 'thinking'
            self._thread = Thread(target=self._run, args=(generation, self._cancel, text, audio, speak), daemon=True)
            self._thread.start()

    def _current(self, generation, cancel):
        return generation == self._generation and not cancel.is_set() and not self._closed

    def _run(self, generation, cancel, text, audio, speak):
        try:
            if audio is not None:
                text = self.speech.transcribe(audio, cancel=cancel)['text']
            with self._lock:
                if not self._current(generation, cancel):
                    return
                self._message('user', text)
            if is_stop(text):
                self.stop()
                return
            with self._lock:
                self._status = 'thinking'
                self._answer = self._message('assistant', '')
                answer = self._answer
            result = self.session.run_turn(text, cancellation=cancel)
            with self._lock:
                if not self._current(generation, cancel):
                    return
                answer['text'] = result.text[:32768]
                if result.status != 'completed':
                    self._error = result.error_code or result.status
                    answer['text'] = '本轮未完成：' + self._error
                    return
                self._status = 'synthesizing' if speak and self.speech else 'idle'
            if speak and self.speech and 'tts' in getattr(self.speech, 'profiles', {'tts': None}):
                # Speech failure must not repeat a successful tool/model turn.
                generated = self.speech.synthesize(result.text, cancel=cancel)
                with self._lock:
                    if self._current(generation, cancel):
                        self._audio = (generation, generated['audio'])
                        answer['audio_url'] = f'/api/agent/audio/{self.runtime_id}/{generation}'
        except ProviderError as exc:
            with self._lock:
                if self._current(generation, cancel):
                    self._error = exc.code
        except Exception:
            with self._lock:
                if self._current(generation, cancel):
                    self._error = 'agent_service_error'
        finally:
            with self._lock:
                self._busy, self._answer = False, None
                self._status = 'idle'

    def audio(self, generation):
        with self._lock:
            if self._audio is None or self._audio[0] != generation or generation != self._generation:
                return None
            return self._audio[1]

    def stop(self, *, stop_robot=True):
        with self._lock:
            self._cancel.set()
            self._generation += 1
            self._audio = None
            self._error = None
            self._notice = '本轮与播报已停止；已连接设备时同步请求停止动作。'
            self._status = 'stopping' if self._busy else 'idle'
        # run_turn shares this cancellation token, including before turn start.
        if self.session is not None:
            self.session.cancel()
        if self.on_stop and stop_robot:
            self.on_stop()

    def close(self):
        self._closed = True
        self.stop(stop_robot=self._busy)
        if self.speech:
            self.speech.close()
        if self.session:
            self.session.close()
        if self._thread:
            self._thread.join(2)
