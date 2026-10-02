"""Cartesian desk integration, exclusively using fake serial hardware."""
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
import test_desk as _desk
from eai_robot.arm.desk_cartesian import DeskCartesian, vector, speed, LEASE_SECONDS
from eai_robot.arm.calibration import JOINTS, JointCalibration


class CartesianDeskTests(unittest.TestCase):
    make_desk = _desk.DeskTests.make_desk
    settle = _desk.DeskTests.settle
    connect = _desk.DeskTests.connect

    def ready(self, enable=True):
        service, bus = self.make_desk()
        for sid, raw in enumerate((531, 222, 514, 170, 442, 249), 1):
            bus.registers[sid]['Present_Position'] = raw
        self.connect(service)
        if enable:
            service.submit('enable_control')
            self.settle(service)
        return service, bus

    def preview(self, service, delta=(5, 0, 0)):
        xyz = np.array(service.snapshot()['cartesian']['actual']['xyz_mm']) + delta
        service.submit('cartesian_preview', {'xyz': xyz.tolist(), 'speed': 15})
        # Unreachable paths run IK and recovery searches. Windows hosted
        # runners can take longer than the ordinary hardware-job wait; this
        # only bounds offline planning, not stream leases or motion timeouts.
        return self.settle(service, timeout=30)['cartesian']['preview']

    def test_fk_is_read_only_and_mapping_matches_course_for_reverse_axes(self):
        service, bus = self.ready(False)
        self.assertEqual(bus.packets, [])
        self.assertEqual(bus.writes, [])
        pose = service.snapshot()['cartesian']['actual']
        self.assertAlmostEqual(pose['xyz_mm'][0], 58.292482, places=5)
        reversed_cal = {n: JointCalibration(c.id, 1, 0, c.range_min, c.range_max)
                        for n, c in service.calibration.items()}
        model = DeskCartesian(reversed_cal)
        raw = {c.id: c.position(.5) for c in reversed_cal.values()}
        q = model.angles(raw)
        self.assertEqual(model.raw_targets(q, raw), raw)

    def test_preview_never_writes_and_execute_preserves_endpoint_gripper(self):
        service, bus = self.ready()
        packets = len(bus.packets)
        writes = len(bus.writes)
        preview = self.preview(service)
        self.assertTrue(preview['reachable'])
        self.assertEqual(preview['speed'], 15)
        self.assertGreater(len(preview['path_mm']), 2)
        self.assertGreater(preview['orientation_change_deg'], 0)
        self.assertEqual((len(bus.packets), len(bus.writes)), (packets, writes))
        service.submit('cartesian_execute', {'preview_id': preview['id']})
        state = self.settle(service)
        self.assertIsNone(state['error'], state['error'])
        self.assertEqual(state['motion_state'], 'reached')
        self.assertTrue(state['control_enabled'])
        self.assertTrue(all(p[6] == 249 for p in bus.packets))
        self.assertLess(state['cartesian']['tracking_error_mm'], 3)
        for packet in bus.packets:
            for n, c in service.calibration.items():
                self.assertTrue(c.range_min <= packet[c.id] <= c.range_max, n)

    def test_partial_plan_is_never_executed_and_manual_control_remains(self):
        service, bus = self.ready()
        preview = self.preview(service, (0, 0, 100))
        self.assertFalse(preview['reachable'])
        packets = len(bus.packets)
        service.submit('cartesian_execute', {'preview_id': preview['id']})
        state = self.settle(service)
        # Only a current-position hold is allowed, never any partial waypoint.
        self.assertEqual(bus.packets[-1], bus.packets[packets-1])
        self.assertTrue(state['control_enabled'])
        self.assertEqual(state['motion_state'], 'paused')
        service.submit('jog', {'joint': 'shoulder_pan', 'delta': 5})
        self.assertEqual(self.settle(service)['motion_state'], 'reached')

    def test_preview_expires_or_pose_changes_without_executing_old_path(self):
        for cause in ('pose', 'expiry'):
            with self.subTest(cause=cause):
                service, bus = self.ready()
                preview = self.preview(service)
                if cause == 'pose':
                    bus.registers[1]['Present_Position'] += 4
                else:
                    service.preview['expires'] = time.monotonic() - 1
                actual = {s: bus.read(s, 'Present_Position') for s in range(1, 7)}
                service.submit('cartesian_execute', {'preview_id': preview['id']})
                state = self.settle(service)
                self.assertEqual(state['motion_state'], 'paused')
                self.assertTrue(state['control_enabled'])
                self.assertEqual(bus.packets[-1], actual)

    def test_lease_timeout_holds_and_late_renewal_cannot_restart(self):
        service, bus = self.ready()
        before = dict(bus.packets[-1])
        service.submit('cartesian_start', {'owner': 'one', 'sequence': 1, 'direction': [1, 0, 0], 'speed': 15})
        state = self.settle(service, timeout=2)
        self.assertEqual(state['motion_state'], 'paused')
        self.assertTrue(state['torque_on'])
        self.assertNotEqual(bus.packets[-1], before)
        count = len(bus.packets)
        with self.assertRaisesRegex(RuntimeError, '连续控制已停止'):
            service.submit('cartesian_intent', {'owner': 'one', 'sequence': 2, 'direction': [1, 0, 0], 'speed': 15})
        time.sleep(.1)
        self.assertEqual(len(bus.packets), count)

    def test_owner_ordering_and_pause_are_independent_of_snapshot_heartbeat(self):
        service, bus = self.ready()
        service.submit('cartesian_start', {'owner': 'one', 'sequence': 10, 'direction': [1, 0, 0], 'speed': 15})
        with self.assertRaisesRegex(RuntimeError, '过期'):
            service.submit('cartesian_intent', {'owner': 'one', 'sequence': 9, 'direction': [0, 1, 0], 'speed': 15})
        with self.assertRaisesRegex(RuntimeError, '已停止'):
            service.submit('cartesian_intent', {'owner': 'two', 'sequence': 11, 'direction': [1, 0, 0], 'speed': 15})
        for seq in range(11, 16):
            service.submit('cartesian_intent', {'owner': 'one', 'sequence': seq, 'direction': [1, 0, 0], 'speed': 15})
            service.snapshot()
            time.sleep(.06)
        service.submit('pause')
        state = self.settle(service)
        self.assertTrue(state['control_enabled'])
        self.assertTrue(all(bus.read(s, 'Torque_Enable') == 1 for s in range(1, 7)))
        self.assertEqual(state['motion_state'], 'paused')
        service.submit('cartesian_step', {'direction': [-1, 0, 0], 'speed': 15, 'distance': 5})
        self.assertEqual(self.settle(service)['motion_state'], 'reached')

    def test_cancelled_owner_cannot_start_from_a_delayed_http_request(self):
        service, bus = self.ready()
        service.submit('pause', {'owner': 'delayed'})
        self.settle(service)
        packets = len(bus.packets)
        with self.assertRaisesRegex(RuntimeError, '已取消'):
            service.submit('cartesian_start', {'owner': 'delayed', 'sequence': 1, 'direction': [1, 0, 0], 'speed': 15})
        self.assertEqual(len(bus.packets), packets)
        self.assertFalse(service.snapshot()['busy'])

    def test_stop_invalidates_active_stream_and_requires_takeover(self):
        service, bus = self.ready()
        service.submit('cartesian_start', {'owner': 'one', 'sequence': 1, 'direction': [1, 0, 0], 'speed': 15})
        time.sleep(.06)
        service.submit('stop')
        state = self.settle(service)
        self.assertFalse(state['control_enabled'])
        self.assertTrue(all(bus.read(s, 'Torque_Enable') == 0 for s in range(1, 7)))
        with self.assertRaises(RuntimeError):
            service.submit('cartesian_step', {'direction': [1, 0, 0], 'speed': 15, 'distance': 5})

    def test_torque_drop_and_goal_mismatch_release_all_axes(self):
        for cause in ('torque', 'goal'):
            with self.subTest(cause=cause):
                service, bus = self.ready()
                if cause == 'torque':
                    bus.registers[1]['Torque_Enable'] = 0
                else:
                    original = bus.sync_positions
                    def bad(targets):
                        original(targets)
                        bus.registers[1]['Goal_Position'] += 1
                    bus.sync_positions = bad
                service.submit('cartesian_step', {'direction': [1, 0, 0], 'speed': 15, 'distance': 5})
                state = self.settle(service)
                self.assertFalse(state['control_enabled'])
                self.assertIsNotNone(state['error'])
                self.assertTrue(all(bus.read(s, 'Torque_Enable') == 0 for s in range(1, 7)))

    def test_pausing_legacy_joint_move_keeps_torque_and_holds_feedback(self):
        service, bus = self.ready()
        bus.follow = False
        service.submit('move', {'values': [.8, .238, .552, .168, .426, .025], 'joints': ['shoulder_pan']})
        time.sleep(.06)
        service.submit('pause')
        state = self.settle(service)
        self.assertEqual(state['motion_state'], 'paused')
        self.assertTrue(state['control_enabled'])
        self.assertTrue(state['torque_on'])
        self.assertEqual(bus.packets[-1][1], bus.read(1, 'Present_Position'))

    def test_slow_solver_and_tracking_error_stop_target_accumulation(self):
        service, bus = self.ready()
        original = service.cartesian.planner
        def slow(*args):
            planner = original(*args)
            actual = planner.plan
            def plan(*a, **kw):
                time.sleep(LEASE_SECONDS + .05)
                return actual(*a, **kw)
            planner.plan = plan
            return planner
        packets = len(bus.packets)
        with patch.object(service.cartesian, 'planner', side_effect=slow):
            service.submit('cartesian_start', {'owner': 'one', 'sequence': 1, 'direction': [1, 0, 0], 'speed': 15})
            state = self.settle(service)
        self.assertEqual(state['motion_state'], 'paused')
        self.assertEqual(bus.packets[packets:], [bus.packets[packets-1]])
        q = service.cartesian.angles(service.arm.positions())
        with self.assertRaisesRegex(Exception, '跟随误差'):
            service._cart_feedback(q + 13)

    def test_untrusted_input_is_rejected_before_any_write(self):
        service, bus = self.ready()
        count = len(bus.packets)
        for payload in ({'xyz': [float('nan'), 0, 0]}, {'xyz': [0, 0]}, {'xyz': [True, 0, 0]},
                        {'xyz': [0, 0, 0], 'speed': 100}):
            with self.assertRaises(ValueError):
                service.submit('cartesian_preview', payload)
        for direction in ([0, 0, 0], [2, 0, 0], [None, 0, 0]):
            with self.assertRaises(ValueError):
                service.submit('cartesian_step', {'direction': direction, 'distance': 5})
        self.assertEqual(len(bus.packets), count)
        preview = self.preview(service, (400, 0, 0))
        self.assertIsNone(preview)
        self.assertTrue(service.snapshot()['control_enabled'])
        self.assertEqual(len(bus.packets), count)


if __name__ == '__main__':
    unittest.main()
