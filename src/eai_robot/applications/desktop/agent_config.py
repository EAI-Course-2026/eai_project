"""Browser-session API settings, held in RAM; never serialize secrets to disk."""
from copy import deepcopy
from dataclasses import asdict
from importlib.resources import files
import json
import secrets
from threading import RLock
import time

from eai_robot.applications.agent.cli import reference_poses
from eai_robot.applications.agent.session import create_robot_session
from eai_robot.applications.agent.skills import DryRunSkills
from eai_robot.applications.agent.speech import CloudSpeech, SpeechEndpoint
from eai_robot.control.planning.poses import POSES_PATH
from eai_robot.policy.model_io.providers import ChatCompletionsProvider, ProviderConfig
from eai_robot.policy.model_io.responses import ResponsesProvider
from .agent import DeskAgent


def catalog():
    data = json.loads(files('eai_robot.applications.agent').joinpath('provider_presets.json').read_text(encoding='utf-8'))
    return {'presets': data['presets'], 'aliases': data['aliases']}


def seed_from_files(chat_path, chat_env=None, speech_path=None, speech_env=None):
    """Optional operator CLI compatibility; web users never need these files."""
    from eai_robot.applications.agent.cli import load_provider
    from eai_robot.applications.agent.speech import load_speech
    from eai_robot.policy.model_io.auth import EnvironmentCredential
    initial = {}
    if chat_path:
        provider = load_provider(chat_path, chat_env)
        try:
            spec = asdict(provider.config)
            for key in ('endpoint', 'credential_env'):
                spec.pop(key, None)
            spec.update(base_url=provider.config.api_base_url.rstrip('/'), provider='custom',
                        adapter=provider.describe()['adapter'], api_key=provider._credential.resolve())
            initial['chat'] = spec
        finally:
            provider.close()
    if speech_path:
        speech = load_speech(speech_path, speech_env)
        try:
            for stage, profile in speech.profiles.items():
                spec = asdict(profile)
                spec.pop('credential_env')
                spec['api_key'] = EnvironmentCredential(profile.credential_env, speech._values).resolve()
                initial[stage] = spec
        finally:
            speech.close()
    return initial or None


class AgentRecord:
    def __init__(self, on_stop=None, initial=None):
        self._lock, self._on_stop = RLock(), on_stop
        self._settings, self._keys = {}, {}
        self.agent = DeskAgent(on_stop=on_stop)
        self.touched = time.monotonic()
        if initial:
            self.configure(initial)

    def public_settings(self):
        with self._lock:
            return {stage: {**deepcopy(spec), 'key_configured': bool(self._keys.get(stage))}
                    for stage, spec in self._settings.items()}

    def configure(self, body):
        if not isinstance(body, dict) or set(body) - {'chat', 'asr', 'tts'}:
            raise ValueError('invalid_api_settings')
        with self._lock:
            if self.agent.snapshot()['busy'] or self.agent.session and not self.agent.session.wait_for_idle(0):
                raise ValueError('busy')
            settings, keys = {}, {}
            # Validate everything before replacing the active runtime.
            for stage, original in body.items():
                if not isinstance(original, dict):
                    raise ValueError('invalid_api_settings')
                spec = deepcopy(original)
                key = spec.pop('api_key', '')
                if not isinstance(key, str) or len(key) > 4096 or '\n' in key or '\r' in key:
                    raise ValueError('invalid_credential')
                field = 'base_url' if stage == 'chat' else 'websocket_url'
                if field in spec and isinstance(spec[field], str):
                    spec[field] = spec[field].strip().rstrip('/')
                previous = self._settings.get(stage, {})
                if not key:
                    # Never carry credentials to a changed provider/endpoint.
                    if spec.get(field) != previous.get(field) or spec.get('provider') != previous.get('provider'):
                        raise ValueError('credential_required_for_new_endpoint')
                    key = self._keys.get(stage, '')
                if not key:
                    raise ValueError('missing_credential')
                settings[stage], keys[stage] = spec, key
            runtime = self._build(settings, keys)
            old = self.agent
            self.agent, self._settings, self._keys = runtime, settings, keys
            old.close()
            return self.public_settings()

    def submit(self, runtime_id, **request):
        with self._lock:
            if runtime_id != self.agent.runtime_id:
                raise ValueError('stale_input')
            self.agent.submit(**request)

    def _build(self, settings, keys):
        provider, speech = None, None
        profiles, values = {}, {}
        try:
            for stage in ('asr', 'tts'):
                if stage in settings:
                    spec = dict(settings[stage])
                    spec['credential_env'] = 'EAI_DESK_' + secrets.token_hex(12).upper()
                    profiles[stage] = SpeechEndpoint(**spec)
                    if stage == 'tts' and not profiles[stage].voice:
                        raise ValueError('tts_voice_required')
                    values[spec['credential_env']] = keys[stage]
            speech = CloudSpeech(profiles, values) if profiles else None
            runtime = DeskAgent(speech=speech, on_stop=self._on_stop)
            if 'chat' in settings:
                spec = dict(settings['chat'])
                spec.pop('provider', None)  # Display label; routing comes from the explicit URL.
                adapter = spec.pop('adapter', 'chat_completions')
                provider_class = {'chat_completions': ChatCompletionsProvider, 'responses': ResponsesProvider}.get(adapter)
                if provider_class is None:
                    raise ValueError('unsupported_chat_protocol')
                name = 'EAI_DESK_' + secrets.token_hex(12).upper()
                spec['credential_env'] = name
                provider = provider_class(ProviderConfig(**spec), credential_values={name: keys['chat']})
                runtime.session = create_robot_session(provider, DryRunSkills(reference_poses(POSES_PATH)), emit=runtime.model_event)
            return runtime
        except Exception:
            if provider:
                provider.close()
            if speech:
                speech.close()
            # Constructor errors can include user input: keep HTTP output stable.
            raise ValueError('invalid_api_settings') from None

    def clear(self):
        with self._lock:
            self.agent.close()
            self.agent = DeskAgent(on_stop=self._on_stop)
            self._settings, self._keys = {}, {}

    def close(self):
        self.clear()


class AgentHub:
    """Bounded, HttpOnly-cookie sessions. No credentials in browser storage."""
    def __init__(self, on_stop=None, initial=None):
        self._on_stop, self._initial = on_stop, initial
        self._lock, self._records = RLock(), {}

    def get(self, token=None):
        now = time.monotonic()
        with self._lock:
            for key, record in list(self._records.items()):
                if now - record.touched > 3600 and not record.agent.snapshot()['busy']:
                    record.close()
                    del self._records[key]
            if token not in self._records:
                if len(self._records) >= 16:
                    raise ValueError('session_limit')
                token = secrets.token_hex(24)
                self._records[token] = AgentRecord(self._on_stop, self._initial)
            record = self._records[token]
            record.touched = now
            return token, record

    def stop(self):
        with self._lock:
            for record in self._records.values():
                record.agent.stop(stop_robot=False)

    def close(self):
        with self._lock:
            for record in self._records.values():
                record.close()
            self._records.clear()
            self._initial = None
