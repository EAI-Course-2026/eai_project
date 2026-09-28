import contextlib
from dataclasses import asdict
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from eai_robot.arm.calibration import JOINTS, JointCalibration, load, save
from eai_robot.arm.demo import PROFILES, build_plan, check_start, load_home, main, run_stage
from eai_robot.arm.controller import ArmController
from test_arm import FakeArm


class DemoTests(unittest.TestCase):
    def calibration(self):
        return {n: JointCalibration(sid, sid % 2, 0, 100, 800) for sid, n in enumerate(JOINTS, 1)}

    def config(self, directory):
        cal = self.calibration()
        path, home_path = Path(directory) / 'calibration.json', Path(directory) / 'home.json'
        save(path, cal)
        home_path.write_text(json.dumps({'schema_version': 1, 'home_raw': [450] * 6,
                                        'start_radius_counts': [200] * 6,
                                        'calibration': {n: asdict(cal[n]) for n in JOINTS}}))
        return cal, path, home_path

    def test_all_profiles_have_identical_home_and_fixed_demo_targets_for_different_starts(self):
        cal = self.calibration()
        home, radius = dict.fromkeys(range(1, 7), 450), dict.fromkeys(range(1, 7), 200)
        for profile in PROFILES:
            a = build_plan(cal, dict.fromkeys(range(1, 7), 300), home, radius, profile)
            b = build_plan(cal, dict.fromkeys(range(1, 7), 600), home, radius, profile)
            self.assertEqual(a[1:], b[1:])
            self.assertEqual(a[1]['targets'], home)
            self.assertEqual(a[0]['targets'][1], 300)
            self.assertEqual(a[0]['targets'][2], 450)
            for item in a:
                for name, ratio in zip(JOINTS, item['ratios'], strict=True):
                    self.assertEqual(cal[name].position(ratio), item['targets'][cal[name].id])
                    self.assertTrue(0 <= ratio <= 1)

    def test_gripper_excursion_is_310_counts(self):
        cal = self.calibration()
        home = dict.fromkeys(range(1, 7), 450)
        plan = build_plan(cal, home, home, dict.fromkeys(range(1, 7), 200), 'gripper')
        self.assertEqual(max(p['targets'][6] for p in plan) - min(p['targets'][6] for p in plan), 310)

    def test_calibration_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            cal, _, home = self.config(d)
            cal['gripper'] = JointCalibration(6, 0, 0, 100, 801)
            with self.assertRaisesRegex(ValueError, '不匹配'):
                load_home(home, cal)

    def test_outside_envelope_and_unsafe_target_rejected_before_movement(self):
        cal = self.calibration()
        home = dict.fromkeys(range(1, 7), 450)
        with self.assertRaisesRegex(ValueError, '入口'):
            build_plan(cal, dict.fromkeys(range(1, 7), 701), home, dict.fromkeys(range(1, 7), 200))
        home[6] = 700
        with self.assertRaisesRegex(ValueError, '不自动裁剪'):
            build_plan(cal, home, home, dict.fromkeys(range(1, 7), 200), 'gripper')

    def test_gravity_drift_one_count_below_entry_is_accepted_but_larger_drift_is_not(self):
        cal = load(ROOT/'calibration/scs215_so101.json')
        home, radius = load_home(ROOT/'configs/demo_home.json', cal)
        start = home.copy()
        start[2] = 63
        self.assertEqual(check_start(cal, start, home, radius)[2], [61, 287])
        plan = build_plan(cal, start, home, radius, 'home')
        self.assertTrue(all(item['targets'][2] >= cal['shoulder_lift'].range_min for item in plan))
        start[2] = 60
        with self.assertRaisesRegex(ValueError, 'shoulder_lift.*入口'):
            check_start(cal, start, home, radius)

    def test_offline_preview_never_opens_serial(self):
        with tempfile.TemporaryDirectory() as d:
            _, path, home = self.config(d)
            with patch('eai_robot.arm.backends.SerialBackend', side_effect=AssertionError('opened port')), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(['--calibration', str(path), '--home-file', str(home), '--anchor-raw', *(['300'] * 6)]), 0)

    def test_complete_sequence_homes_before_demo_and_releases(self):
        bus = FakeArm()
        for sid in range(1, 7):
            bus.registers[sid]['Present_Position'] = 300
        with tempfile.TemporaryDirectory() as d:
            _, path, home = self.config(d)
            log = Path(d) / 'demo.json'
            with patch('eai_robot.arm.backends.SerialBackend', return_value=bus), patch('eai_robot.arm.demo.time.sleep'), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(['--calibration', str(path), '--home-file', str(home), '--log', str(log), '--enable']), 0)
            record = json.loads(log.read_text())
            self.assertTrue(record['completed'] and record['homed'])
            self.assertEqual([p['phase'] for p in record['stages'][:2]], ['home', 'home'])
            self.assertEqual(record['stages'][1]['feedback'], {str(s): 450 for s in range(1, 7)})
            self.assertEqual(set(record['torque_after_exit'].values()), {0})
            for previous, current in zip(bus.packets, bus.packets[1:]):
                self.assertLessEqual(max(abs(previous[s] - current[s]) for s in current), 20)
            self.assertTrue(bus.closed)

    def test_failed_homing_never_starts_demo_and_logs_stuck_joint(self):
        bus = FakeArm()
        bus.follow = False
        for sid in range(1, 7):
            bus.registers[sid]['Present_Position'] = 300
        with tempfile.TemporaryDirectory() as d:
            _, path, home = self.config(d)
            log = Path(d) / 'failed.json'
            with patch('eai_robot.arm.backends.SerialBackend', return_value=bus), patch('eai_robot.arm.controller.time.sleep'), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(main(['--calibration', str(path), '--home-file', str(home), '--log', str(log), '--timeout', '.1', '--enable']), 1)
            record = json.loads(log.read_text())
            self.assertFalse(record['completed'] or record['homed'])
            self.assertEqual(len(record['stages']), 1)
            self.assertFalse(record['stages'][0]['substeps'][-1]['reached'])
            self.assertIn('2', record['stages'][0]['substeps'][-1]['errors'])
            self.assertEqual(set(record['torque_after_exit'].values()), {0})

    def test_fault_at_start_has_no_torque_writes(self):
        bus = FakeArm()
        bus.registers[3]['Status'] = 1
        with tempfile.TemporaryDirectory() as d:
            _, path, home = self.config(d)
            with patch('eai_robot.arm.backends.SerialBackend', return_value=bus), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(main(['--calibration', str(path), '--home-file', str(home), '--log', str(Path(d)/'log.json'), '--enable']), 1)
        self.assertEqual(bus.writes, [])
        self.assertTrue(bus.closed)

    def test_teach_is_read_only_and_refuses_overwrite(self):
        bus = FakeArm()
        with tempfile.TemporaryDirectory() as d:
            _, path, _ = self.config(d)
            home = Path(d)/'taught.json'
            args = ['--calibration', str(path), '--teach-home', str(home)]
            with patch('eai_robot.arm.backends.SerialBackend', return_value=bus), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(main(args), 0)
                self.assertEqual(main(args), 1)
            self.assertEqual(json.loads(home.read_text())['home_raw'], [450]*6)
        self.assertEqual(bus.writes, [])
        self.assertEqual(bus.packets, [])

    def test_intermediate_static_error_does_not_block_but_final_precision_is_enforced(self):
        class LaggingArm(FakeArm):
            def sync_positions(self, targets):
                super().sync_positions(targets)
                if targets[3] != 350:
                    self.registers[3]['Present_Position'] += 16
        bus = LaggingArm()
        arm = ArmController(bus, self.calibration(), tolerance=15, timeout=.1)
        arm.enabled = True
        item = {'targets': dict.fromkeys(range(1, 7), 350)}
        with patch('eai_robot.arm.demo.time.sleep'):
            feedback = run_stage(arm, item)
        self.assertEqual(feedback[3], 350)
        self.assertEqual([s['tolerance_counts'] for s in item['substeps']], [25, 25, 25, 25, 15])
        self.assertEqual(arm.tolerance, 15)

    def test_final_static_error_is_not_reported_as_success(self):
        class StuckArm(FakeArm):
            def sync_positions(self, targets):
                super().sync_positions(targets)
                self.registers[3]['Present_Position'] += 16
        bus = StuckArm()
        arm = ArmController(bus, self.calibration(), tolerance=15, timeout=.1)
        arm.enabled = True
        item = {'targets': dict.fromkeys(range(1, 7), 350)}
        with patch('eai_robot.arm.demo.time.sleep'), self.assertRaises(TimeoutError):
            run_stage(arm, item)
        self.assertFalse(item['substeps'][-1]['reached'])
        self.assertEqual(item['substeps'][-1]['errors'][3], 16)
        self.assertEqual(item['substeps'][-1]['feedback_source'], 'at_failure_before_release')
        self.assertEqual(arm.tolerance, 15)
        self.assertEqual({bus.read(s, 'Torque_Enable') for s in range(1, 7)}, {0})


if __name__ == '__main__':
    unittest.main()
