"""Explicit cloud speech file calls and synthetic round-trip acceptance."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import unicodedata

from eai_robot.policy.model_io.contracts import ProviderError
from .speech import load_speech

PHRASES = ('请查看机械臂的当前状态。', '不要移动，先告诉我有哪些姿态。', '停止当前任务。')


def normalized(text):
    return ''.join(c for c in unicodedata.normalize('NFKC', text) if c.isalnum())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--env-file', type=Path)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('check', help='Offline profiles and credentials check; no network')
    tts = sub.add_parser('tts', help='Send text to cloud TTS; save WAV, no playback')
    tts.add_argument('--text', required=True)
    tts.add_argument('--output', type=Path, required=True)
    asr = sub.add_parser('asr', help='Upload an explicitly selected mono 16kHz WAV')
    asr.add_argument('--audio', type=Path, required=True)
    probe = sub.add_parser('probe', help='Real TTS -> ASR calls using three fixed synthetic phrases')
    probe.add_argument('--rounds', type=int, choices=range(1, 4), default=3)
    probe.add_argument('--output-dir', type=Path, required=True, help='New directory; refuses to overwrite evidence')
    args = parser.parse_args(argv)
    client = None
    try:
        client = load_speech(args.config, args.env_file)
        if args.command == 'check':
            client.check_credentials()
            result = {'status': 'valid_local_config', 'provider': client.describe(), 'network_called': False}
        elif args.command == 'tts':
            # Reserve destination before any paid call.
            with args.output.open('xb') as stream:
                result = client.synthesize(args.text)
                stream.write(result.pop('audio'))
        elif args.command == 'asr':
            if args.audio.stat().st_size > 2 * 1024 * 1024:
                raise ValueError('Audio too large')
            result = client.transcribe(args.audio.read_bytes())
        else:
            args.output_dir.mkdir(parents=True, exist_ok=False)
            rows = []
            for index, text in enumerate(PHRASES[:args.rounds], 1):
                row = {'round': index, 'input': text}
                try:
                    generated = client.synthesize(text)
                    audio = generated.pop('audio')
                    (args.output_dir / f'{index}.wav').write_bytes(audio)
                    recognized = client.transcribe(audio)
                    row.update(tts=generated, asr=recognized,
                               status='passed' if normalized(text) == normalized(recognized['text']) else 'failed')
                except ProviderError as exc:
                    row.update(status='failed', error_code=exc.code)
                rows.append(row)
            result = {'at': datetime.now(timezone.utc).isoformat(), 'suite': 'synthetic_speech_roundtrip',
                      'status': 'passed' if all(r['status'] == 'passed' for r in rows) else 'failed',
                      'provider': client.describe(), 'microphone_used': False, 'hardware_connected': False, 'results': rows}
            (args.output_dir / 'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
        return 1 if result.get('status') == 'failed' else 0
    except ProviderError as exc:
        print(json.dumps({'status': 'failed', 'error_code': exc.code}))
        return 1
    except (OSError, ValueError):
        print(json.dumps({'status': 'failed', 'error_code': 'speech_configuration_or_input_error'}))
        return 1
    finally:
        if client is not None:
            client.close()


if __name__ == '__main__':
    raise SystemExit(main())
