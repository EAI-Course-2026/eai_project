"""Shared arm data stays portable; local devices are selected explicitly."""
import json
from pathlib import Path
import sys
import tempfile
import tomllib
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from eai_robot import config
from eai_robot.course.vision import visual_closed_loop as vision


class SharedCalibrationTests(unittest.TestCase):
    def test_checked_in_device_defaults_are_unset(self):
        with (ROOT/'configs/hardware.example.toml').open('rb') as stream:
            example = tomllib.load(stream)
        self.assertEqual(example['serial']['port'], '')
        self.assertEqual(example['camera']['index_or_path'], '')

    def test_only_active_pair_at_calibration_root(self):
        names = {p.name for p in (ROOT/'calibration').glob('*.json')}
        self.assertEqual(names, {'scs215_so101.json', 'scs215_so101.meta.json'})

    def test_shared_evidence_has_no_host_connection_fields(self):
        def check(value):
            if isinstance(value, dict):
                for key, item in value.items():
                    self.assertNotIn(key, {'port', 'port_at_capture', 'serial_port', 'camera_index'})
                    check(item)
            elif isinstance(value, list):
                for item in value:
                    check(item)
            elif isinstance(value, str):
                self.assertNotIn('/dev/', value)
                self.assertNotIn('/Users/', value)
                self.assertNotIn('C:\\Users\\', value)
        for path in (ROOT/'calibration').rglob('*.json'):
            with self.subTest(path=path):
                check(json.loads(path.read_text(encoding='utf-8')))

    def test_shared_metadata_preserves_unverified_motion(self):
        meta = json.loads((ROOT/'calibration/scs215_so101.meta.json').read_text())
        self.assertEqual(meta['hardware_id'], 'scs215_so101')
        self.assertTrue(meta['manual_endpoints_verified'])
        for key in ('full_range_motion_verified', 'powered_motion_verified', 'fk_alignment_verified'):
            self.assertFalse(meta[key])
        self.assertTrue(meta['eeprom_readback_verified'])

    def test_local_windows_and_mac_ports_do_not_change_shared_ranges(self):
        before = (ROOT/'calibration/scs215_so101.json').read_bytes()
        for port in ('COM12', '/dev/cu.usbmodemEXAMPLE'):
            with self.subTest(port=port), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                (root/'configs').mkdir()
                (root/'configs/hardware.example.toml').write_text('[serial]\nport = ""\nbaudrate = 1000000\n[camera]\nindex_or_path = ""\n')
                (root/'configs/hardware.local.toml').write_text(f'[serial]\nport = "{port}"\n[camera]\nindex_or_path = 2\n')
                with patch.object(config, 'ROOT', root):
                    settings = config.load_config()
                    self.assertEqual(settings['serial']['port'], port)
                    self.assertEqual(settings['serial']['baudrate'], 1000000)
                    self.assertEqual(config.default_camera_index(), 2)
        self.assertEqual((ROOT/'calibration/scs215_so101.json').read_bytes(), before)

    def test_camera_requires_explicit_selection_without_local_config(self):
        with patch.object(vision, 'default_camera_index', return_value=None), patch.object(sys, 'argv', ['vision']), patch.object(vision, 'make_robot') as make_robot:
            with self.assertRaises(SystemExit) as error:
                vision.parse_args()
            self.assertEqual(error.exception.code, 2)
            make_robot.assert_not_called()

    def test_cli_camera_overrides_local_index_including_zero(self):
        with patch.object(vision, 'default_camera_index', return_value=2), patch.object(sys, 'argv', ['vision', '--camera-index', '0']):
            self.assertEqual(vision.parse_args().camera_index, 0)

    def test_vision_rejects_unset_index_before_robot_construction(self):
        with patch.object(vision, 'make_robot') as make_robot:
            with self.assertRaises(ValueError):
                vision.VisionArmRuntime(SimpleNamespace(camera_index=None), vision.StopState())
            make_robot.assert_not_called()

    def test_archived_candidates_keep_matching_home_bindings(self):
        for date in ('20260928', '20261002'):
            history = ROOT/'calibration/history'
            cal = json.loads((history/f'scs215_safe_candidate_{date}.json').read_text())
            home = json.loads((history/f'demo_home_scs215_safe_candidate_{date}.json').read_text())
            self.assertEqual(home['calibration'], cal)
            meta = json.loads((history/f'scs215_safe_candidate_{date}.meta.json').read_text())
            self.assertEqual(meta['status'], 'archived_candidate_not_activated')


if __name__ == '__main__':
    unittest.main()
