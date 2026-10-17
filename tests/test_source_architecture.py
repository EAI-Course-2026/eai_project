"""Regression checks for module ownership, compatibility and calibration isolation."""
import ast
import importlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from eai_robot.calibration.results import JOINTS, load
from eai_robot.calibration.servo import save_manual_candidate

ROOT = Path(__file__).resolve().parents[1]


class SourceArchitectureTests(unittest.TestCase):
    def test_legacy_modules_share_canonical_state(self):
        for old, new in (
            ('arm.controller', 'control.arm_controller'),
            ('arm.backends', 'hardware.backends'),
            ('arm.desk_service', 'applications.desk_service'),
            ('arm.desk_server', 'applications.desk_server'),
            ('poses.control_presets', 'applications.pose_control'),
            ('kinematics.keyboard_control', 'applications.keyboard_control'),
            ('vision.aruco_ibvs', 'control.aruco_ibvs'),
        ):
            with self.subTest(old=old):
                self.assertIs(importlib.import_module('eai_robot.' + old),
                              importlib.import_module('eai_robot.' + new))

    def test_runtime_results_and_pure_motion_do_not_load_workflows_or_devices(self):
        code = '''
import sys
from eai_robot.calibration.results import load
from eai_robot.motion.poses import POSES_PATH, load_poses
from eai_robot.motion.demo import build_plan
from eai_robot.motion.cartesian import DeskCartesian
from eai_robot.control.aruco_ibvs import corner_error_px
for name in sys.modules:
    assert not name.startswith(('cv2', 'lerobot', 'serial', 'pynput',
        'eai_robot.applications', 'eai_robot.calibration.servo')), name
assert POSES_PATH.is_file()
'''
        result = subprocess.run([sys.executable, '-c', code], cwd=ROOT,
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_plugin_calibration_has_no_application_dependency(self):
        code = '''
import sys
import lerobot_robot_scs215.calibration
assert not any(n == 'eai_robot' or n.startswith('eai_robot.') for n in sys.modules)
'''
        result = subprocess.run([sys.executable, '-c', code], cwd=ROOT,
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_capability_modules_do_not_import_application_or_legacy_packages(self):
        for folder in ('calibration', 'control', 'hardware', 'kinematics', 'motion', 'robots', 'vision'):
            for file in (ROOT / 'src/eai_robot' / folder).rglob('*.py'):
                tree = ast.parse(file.read_text())
                if (ast.get_docstring(tree) or '').startswith('Compatibility'):
                    continue
                for node in ast.walk(tree):
                    names = ([node.module or ''] if isinstance(node, ast.ImportFrom)
                             else [item.name for item in node.names] if isinstance(node, ast.Import) else [])
                    for name in names:
                        self.assertFalse(name.startswith(('eai_robot.applications', 'eai_robot.arm', 'eai_robot.poses')),
                                         f'{file.relative_to(ROOT)} imports {name}')

    def test_gui_candidate_is_local_and_preserves_shared_result(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shared = root / 'calibration/scs215_so101.json'
            shared.parent.mkdir()
            shared.write_text('reviewed result must remain unchanged\n')
            captures = {name: (600, 200) for name in JOINTS}
            path = save_manual_candidate(captures, dict.fromkeys(range(1, 7), (100, 800)), root)
            self.assertTrue(path.is_relative_to(root / 'local_artifacts/experiments'))
            result = load(path)
            self.assertTrue(all(c.drive_mode == 1 and c.range_min == 200 and c.range_max == 600
                                for c in result.values()))
            self.assertEqual(shared.read_text(), 'reviewed result must remain unchanged\n')
            self.assertEqual(json.loads(path.with_suffix('.meta.json').read_text())['method'],
                             'manual_endpoints_gui')

    def test_invalid_gui_samples_create_no_candidate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for pair in ((None, 500), (300, 310), (50, 500)):
                with self.subTest(pair=pair):
                    with self.assertRaises(ValueError):
                        save_manual_candidate(dict.fromkeys(JOINTS, pair),
                                              dict.fromkeys(range(1, 7), (100, 800)), root)
                    self.assertEqual(list(root.iterdir()), [])
