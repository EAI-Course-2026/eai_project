"""Media bounds, explicit credentials and owned SDK worker cancellation, no API."""
from dataclasses import replace
import io
import os
from pathlib import Path
import tempfile
from threading import Event, Timer
import time
import unittest
from unittest.mock import patch
import wave

from eai_robot.applications.agent.speech import CloudSpeech, SpeechEndpoint, load_speech, validate_wav
from eai_robot.policy.model_io.contracts import ProviderError


def wav_bytes(frames=1600):
    stream = io.BytesIO()
    with wave.open(stream, 'wb') as wav:
        wav.setnchannels(1); wav.setsampwidth(2); wav.setframerate(16000)
        wav.writeframes(b'\0\0' * frames)
    return stream.getvalue()


def hanging_sdk(connection, stage, profile, key, payload):
    time.sleep(10)


def successful_sdk(connection, stage, profile, key, payload):
    connection.send({'ok': True, 'text': '模拟转写', 'usage': {}, 'first_package_ms': 1})
    connection.close()


class SpeechTests(unittest.TestCase):
    def client(self, target, timeout=2):
        profile = SpeechEndpoint('dashscope', 'wss://example.invalid/inference', 'fixture', 'SPEECH_TEST_KEY', 'fixture', timeout)
        client = CloudSpeech({'asr': profile, 'tts': profile}, {'SPEECH_TEST_KEY': 'PRIVATE_ONLY_IN_MEMORY'}, _target=target)
        self.addCleanup(client.close)
        return client

    def test_wav_bounds_format_duration_and_truncation_before_network(self):
        self.assertEqual(validate_wav(wav_bytes()), .1)
        for audio in (b'', b'not wav' * 10, wav_bytes()[:-1], wav_bytes(16000*31)):
            with self.subTest(length=len(audio)), self.assertRaises(ValueError):
                validate_wav(audio)
        with self.assertRaises(ValueError):
            self.client(successful_sdk).synthesize(' ')

    def test_unconfigured_stage_does_not_lock_the_configured_stage(self):
        client = self.client(successful_sdk)
        del client.profiles['tts']
        with self.assertRaisesRegex(ProviderError, '^speech_not_configured$'):
            client.synthesize('测试')
        self.assertEqual(client.transcribe(wav_bytes())['text'], '模拟转写')

    def test_cancel_terminates_owned_blocking_sdk_and_prevents_late_result(self):
        client = self.client(hanging_sdk)
        cancelled = Event(); timer = Timer(.2, cancelled.set)
        timer.start(); started = time.monotonic()
        try:
            with self.assertRaisesRegex(ProviderError, '^cancelled$'):
                client.transcribe(wav_bytes(), cancel=cancelled)
        finally:
            timer.cancel()
        self.assertLess(time.monotonic() - started, 3)
        # Slot released only after worker termination; a fresh explicit request can run.
        client._target = successful_sdk
        self.assertEqual(client.transcribe(wav_bytes())['text'], '模拟转写')

    def test_timeout_and_missing_credential_do_not_expose_key(self):
        client = self.client(hanging_sdk, timeout=1)
        with self.assertRaisesRegex(ProviderError, '^speech_timeout$'):
            client.synthesize('测试')
        with patch.dict(os.environ, {}, clear=True):
            client._values = {}
            with self.assertRaisesRegex(ProviderError, '^missing_credential$'):
                client.synthesize('测试')

    def test_profiles_require_explicit_wss_and_never_discover_dotenv(self):
        profile = self.client(successful_sdk).profiles['asr']
        for endpoint in ('ws://example.invalid/x', 'wss://user:SECRET@example.invalid/x', 'wss://example.invalid/x?key=SECRET'):
            with self.assertRaises(ValueError):
                replace(profile, websocket_url=endpoint)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'speech.toml'
            path.write_text('''[asr]
provider="dashscope"
websocket_url="wss://example.invalid/x"
model="fixture"
credential_env="MISSING_TEST_CREDENTIAL"
[tts]
provider="dashscope"
websocket_url="wss://example.invalid/x"
model="fixture"
voice="fixture"
credential_env="MISSING_TEST_CREDENTIAL"
''')
            (Path(folder)/'.env').write_text('MISSING_TEST_CREDENTIAL=DO_NOT_DISCOVER\n')
            client = load_speech(path)
            self.addCleanup(client.close)
            with patch.dict(os.environ, {}, clear=True), self.assertRaisesRegex(ProviderError, 'missing_credential'):
                client.transcribe(wav_bytes())


if __name__ == '__main__':
    unittest.main()
