"""Official DashScope v1 calls, isolated from application SDK globals and devices."""
import io
import logging
import os
from pathlib import Path
import tempfile
import wave
from eai_robot.policy.model_io.contracts import ProviderError


def worker(connection, stage, profile, key, payload):
    # Credentials travel in process memory, never command arguments or logs.
    logging.disable(logging.CRITICAL)
    with open(os.devnull, 'w') as quiet:
        os.dup2(quiet.fileno(), 1)
        os.dup2(quiet.fileno(), 2)
    try:
        import dashscope
        dashscope.api_key = key
        dashscope.base_websocket_api_url = profile.websocket_url
        result = perform(stage, profile, payload)
        connection.send({'ok': True, **result})
    except ImportError:
        connection.send({'ok': False, 'error_code': 'speech_extra_required'})
    except ProviderError as exc:
        connection.send({'ok': False, 'error_code': exc.code})
    except TimeoutError:
        connection.send({'ok': False, 'error_code': 'speech_timeout'})
    except Exception:
        # Remote SDK exceptions may include credentials/URLs/text: never forward.
        connection.send({'ok': False, 'error_code': 'speech_service_error'})
    finally:
        connection.close()


def perform(stage, profile, payload):
    if stage == 'tts':
        from dashscope.audio.tts_v2 import SpeechSynthesizer, AudioFormat
        sdk = SpeechSynthesizer(model=profile.model, voice=profile.voice,
                                format=AudioFormat.PCM_16000HZ_MONO_16BIT,
                                url=profile.websocket_url)
        pcm = sdk.call(payload, timeout_millis=int(profile.timeout_s * 1000))
        if not isinstance(pcm, bytes) or not 0 < len(pcm) <= 960000 or len(pcm) % 2:
            raise ValueError('Invalid synthesized audio')
        stream = io.BytesIO()
        with wave.open(stream, 'wb') as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(16000)
            wav.writeframes(pcm)
        response = sdk.get_response() or {}
        usage = response.get('payload', {}).get('usage', {})
        return {'audio': stream.getvalue(), 'first_package_ms': sdk.get_first_package_delay(),
                'usage': usage, 'text': ''}
    if stage != 'asr':
        raise ValueError('Unknown stage')
    from dashscope.audio.asr import Recognition, RecognitionResult
    sdk = Recognition(model=profile.model, format='wav', sample_rate=16000,
                      callback=None, disfluency_removal_enabled=False)
    with tempfile.TemporaryDirectory(prefix='eai-speech-') as folder:
        path = Path(folder) / 'input.wav'
        path.write_bytes(payload)
        response = sdk.call(str(path))
    if response.status_code != 200:
        status = response.status_code
        raise ProviderError('speech_authentication' if status in (401, 403) else
                            'speech_rate_limited' if status == 429 else
                            'speech_invalid_request' if status in (400, 404, 422) else 'speech_service_error')
    sentences = response.get_sentence() or []
    if isinstance(sentences, dict):
        sentences = [sentences]
    finals = [s for s in sentences if RecognitionResult.is_sentence_end(s)]
    text = ''.join(s.get('text', '') for s in finals)
    if not text.strip() or len(text) > 8192:
        raise ValueError('No final transcript')
    return {'text': text, 'usage': [response.get_usage(s) for s in finals],
            'first_package_ms': sdk.get_first_package_delay()}
