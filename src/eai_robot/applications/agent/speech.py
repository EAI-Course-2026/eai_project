"""Cloud speech profiles and bounded SDK calls. No device capture or model loading."""
from dataclasses import dataclass
import io
import math
import multiprocessing
from pathlib import Path
from threading import Event, Lock
import time
import tomllib
from urllib.parse import urlsplit
import wave

from dotenv import dotenv_values
from eai_robot.policy.model_io.auth import EnvironmentCredential
from eai_robot.policy.model_io.contracts import ProviderError

MAX_AUDIO = 2 * 1024 * 1024


@dataclass(frozen=True)
class SpeechEndpoint:
    provider: str
    websocket_url: str
    model: str
    credential_env: str
    voice: str = ""
    timeout_s: float = 30.

    def __post_init__(self):
        if not isinstance(self.websocket_url, str) or not isinstance(self.credential_env, str) or not self.credential_env:
            raise ValueError('Explicit endpoint and credential variable required')
        url = urlsplit(self.websocket_url)
        if (self.provider != "dashscope" or url.scheme != "wss" or not url.hostname
                or url.username or url.password or url.query or url.fragment
                or "YOUR_WORKSPACE_ID" in self.websocket_url
                or not isinstance(self.model, str) or not 1 <= len(self.model) <= 128
                or not isinstance(self.voice, str) or len(self.voice) > 128
                or type(self.timeout_s) not in (int, float) or not math.isfinite(self.timeout_s)
                or not 1 <= self.timeout_s <= 120):
            raise ValueError("Invalid cloud speech profile")
        EnvironmentCredential(self.credential_env)


def load_speech(path, env_file=None):
    with Path(path).open('rb') as stream:
        data = tomllib.load(stream)
    if set(data) != {"asr", "tts"}:
        raise ValueError("Expected [asr] and [tts]")
    try:
        profiles = {stage: SpeechEndpoint(**settings) for stage, settings in data.items()}
    except TypeError:
        raise ValueError("Unknown speech profile fields") from None
    if not profiles['tts'].voice:
        raise ValueError("TTS requires voice")
    values = dotenv_values(env_file, interpolate=False) if env_file is not None else {}
    if env_file is not None and not Path(env_file).is_file():
        raise ValueError('Explicit speech environment file does not exist')
    return CloudSpeech(profiles, values)


def validate_wav(audio):
    if not isinstance(audio, bytes) or not 44 <= len(audio) <= MAX_AUDIO:
        raise ValueError("Audio must be a bounded WAV")
    try:
        with wave.open(io.BytesIO(audio), 'rb') as wav:
            if (wav.getnchannels(), wav.getsampwidth(), wav.getframerate(), wav.getcomptype()) != (1, 2, 16000, 'NONE'):
                raise ValueError("Expected mono 16-bit 16kHz PCM WAV")
            frames = wav.getnframes()
            if not 0 < frames <= 16000 * 30 or len(wav.readframes(frames)) != frames * 2:
                raise ValueError("Audio must contain 0–30 seconds of complete PCM")
            return frames / 16000
    except (wave.Error, EOFError):
        raise ValueError("Invalid WAV") from None


class CloudSpeech:
    """One call at a time; SDK globals and blocking I/O live in an owned process.

    Cancellation terminates local I/O; it does not promise remote billing stops.
    No SDK retry, config discovery or credential mutation in the application.
    """
    def __init__(self, profiles, values=None, *, _target=None):
        self.profiles, self._values, self._target = profiles, values or {}, _target
        self._slot, self._closed = Lock(), Event()

    def describe(self):
        return {stage: {"provider": p.provider, "model": p.model} for stage, p in self.profiles.items()}

    def check_credentials(self):
        for profile in self.profiles.values():
            EnvironmentCredential(profile.credential_env, self._values).resolve()

    def transcribe(self, audio, *, cancel=None):
        validate_wav(audio)
        return self._call('asr', audio, cancel)

    def synthesize(self, text, *, cancel=None):
        if not isinstance(text, str) or not text.strip() or len(text) > 1000:
            raise ValueError("TTS text must contain 1–1000 characters")
        return self._call('tts', text, cancel)

    def _call(self, stage, payload, cancel):
        from ._dashscope_speech import worker
        cancel = cancel or Event()
        if self._closed.is_set() or cancel.is_set():
            raise ProviderError('cancelled')
        if not self._slot.acquire(blocking=False):
            raise ProviderError('speech_busy')
        profile = self.profiles[stage]
        process = receiving = sending = None
        started = time.monotonic()
        try:
            key = EnvironmentCredential(profile.credential_env, self._values).resolve()
            ctx = multiprocessing.get_context('spawn')
            receiving, sending = ctx.Pipe(duplex=False)
            process = ctx.Process(target=self._target or worker, args=(sending, stage, profile, key, payload), daemon=True)
            process.start()
            sending.close()
            while True:
                if cancel.is_set() or self._closed.is_set():
                    raise ProviderError('cancelled')
                if time.monotonic() - started >= profile.timeout_s:
                    raise ProviderError('speech_timeout')
                if receiving.poll(.02):
                    try:
                        result = receiving.recv()
                    except EOFError:
                        raise ProviderError('speech_worker_failed') from None
                    if cancel.is_set() or self._closed.is_set():
                        raise ProviderError('cancelled')
                    if not result['ok']:
                        raise ProviderError(result['error_code'])
                    result['duration_ms'] = round((time.monotonic() - started) * 1000, 2)
                    return result
                if not process.is_alive():
                    raise ProviderError('speech_worker_failed')
        finally:
            if process is not None and process.pid is not None:
                process.join(.1)
                if process.is_alive():
                    process.terminate()
                    process.join(1)
                if process.is_alive():
                    process.kill()
                    process.join(1)
                process.close()
            for connection in (receiving, sending):
                if connection is not None:
                    connection.close()
            self._slot.release()

    def close(self):
        self._closed.set()
