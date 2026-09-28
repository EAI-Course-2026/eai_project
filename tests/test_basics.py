import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import contextlib
import io

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from eai_robot.control.mapping import ratio_to_position
from eai_robot.hardware.scs import command, parse_reply, ProtocolError
from eai_robot.config import ROOT as CONFIG_ROOT, load_config


class Basics(unittest.TestCase):
    def test_read_packet(self):
        self.assertEqual(command(1, 2, [56, 2]).hex(), 'ffff0104023802be')

    def test_reply_validation(self):
        self.assertEqual(parse_reply(bytes.fromhex('ff ff 01 04 00 02 01 f7'), 1, 2), b'\x02\x01')
        for raw in (bytes.fromhex('80 80 fe 18'), bytes.fromhex('ff ff 01 04 00 02 01 f6')):
            with self.assertRaises(ProtocolError):
                parse_reply(raw, 1, 2)
        with self.assertRaises(TimeoutError):
            parse_reply(b'', 1, 2)

    def test_mapping(self):
        self.assertEqual(ratio_to_position(.5, 100, 500), 300)
        self.assertEqual(ratio_to_position(0, 100, 500, -1), 500)
        self.assertEqual(ratio_to_position(1, 100, 500, -1), 100)
        for x in (-.1, 1.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                ratio_to_position(x, 0, 512)

    def test_project_root(self):
        self.assertEqual(CONFIG_ROOT, ROOT)
        self.assertIn('serial', load_config())

    def test_entry_imports_do_not_open_serial(self):
        files = list((ROOT / 'experiments/servos').glob('*.py')) + [
            ROOT / 'scripts' / name for name in ('servo_setup.py', 'arm_serial.py', 'arm_lerobot.py', 'arm_demo.py', 'arm_desk.py')
        ]
        with patch('serial.Serial', side_effect=AssertionError('import opened serial')):
            for n, file in enumerate(files):
                name = f'entry_{n}'
                spec = importlib.util.spec_from_file_location(name, file)
                module = importlib.util.module_from_spec(spec)
                sys.modules[name] = module
                spec.loader.exec_module(module)

    def test_default_scan_contains_exactly_six_ids(self):
        spec = importlib.util.spec_from_file_location('setup_six_test', ROOT / 'scripts/servo_setup.py')
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        with patch.object(sys, 'argv', ['servo_setup.py', '--scan']), \
                patch('serial.Serial'), patch.object(mod, 'scan', return_value=0) as scan:
            self.assertEqual(mod.main(), 0)
        self.assertEqual(list(scan.call_args.args[1]), [1, 2, 3, 4, 5, 6])

    def test_scan_does_not_write(self):
        spec = importlib.util.spec_from_file_location('setup_test', ROOT / 'scripts/servo_setup.py')
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        class Fake:
            def identity(self, sid):
                if sid != 6:
                    raise TimeoutError()
                return sid
        with contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(mod.scan(Fake(), range(1,7)), 0)
        self.assertIn('[6]', output.getvalue())


if __name__ == '__main__':
    unittest.main()
