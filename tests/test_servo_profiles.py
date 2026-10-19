import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from eai_robot.config import ROOT
from eai_robot.applications.visual_servo.profiles import load_profile


class ServoProfileTest(unittest.TestCase):
    def test_validated_mvp_profile_never_enables_hardware_or_handover(self):
        from eai_robot.applications.visual_servo.ball import parse_args
        profile=ROOT/'configs/visual_servo/aruco_center_mvp.toml'
        with patch.object(sys,'argv',['vision','--profile',str(profile),'--camera-index','0','--max-runtime-s','12']):
            args=parse_args()
        self.assertTrue(args.joint_center)
        self.assertEqual(args.max_travel_mm,100)
        self.assertEqual(args.wrist_step_counts,12)
        self.assertEqual(args.max_runtime_s,12)
        self.assertFalse(args.execute)
        self.assertFalse(args.adopt_hold)

    def test_profiles_cannot_inject_authority_device_or_nonfinite_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'profile.toml'
            for field,value in [('execute','true'),('adopt_hold','true'),('camera_index','0'),
                                ('port','"/dev/example"'),('target','"invalid"'),('max_travel_mm','nan'),('marker_id','true')]:
                with self.subTest(field=field):
                    path.write_text(f'schema_version=1\n[servo]\n{field}={value}\n')
                    with self.assertRaises(ValueError): load_profile(path)

    def test_the_object_section_is_optional(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'profile.toml'
            path.write_text('schema_version=1\n[servo]\nmarker_id=0\n')
            self.assertEqual(load_profile(path), {'marker_id': 0})

    def test_the_object_section_flattens_into_the_same_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'profile.toml'
            # A profile always states the rule it runs, so [servo] stays required
            # even when every rule number is left at its default.
            path.write_text(
                'schema_version=1\n'
                '[servo]\n'
                'rule_step_mm=10.0\n'
                '[perception]\n'
                'object_rule=true\n'
                'tracker="fused"\n'
                'prompt="the red cup"\n'
                'reference_depth_m=0.32\n'
                'tracker_max_coast_s=0.15\n')
            values=load_profile(path)
            self.assertTrue(values['object_rule'])
            self.assertEqual(values['prompt'],'the red cup')
            self.assertEqual(values['reference_depth_m'],0.32)
            self.assertEqual(values['rule_step_mm'],10.0)

    def test_a_profile_cannot_carry_weights_or_calibration_paths(self):
        # Weights are machine-local bytes: naming them in a portable profile
        # would make "the same" run mean a different model elsewhere.
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'profile.toml'
            for field in ('detector_weights','intrinsics','hand_eye','weights'):
                with self.subTest(field=field):
                    path.write_text(f'schema_version=1\n[servo]\n'
                                    f'[perception]\n{field}="/tmp/w.bin"\n')
                    with self.assertRaises(ValueError): load_profile(path)

    def test_the_object_section_is_validated_not_trusted(self):
        cases = {
            'unknown tracker': 'object_rule=true\ntracker="mystery"\nreference_depth_m=0.3',
            'texture tracker without a region': 'object_rule=true\ntracker="support"',
            'no distance scale': 'object_rule=true\ntracker="fused"',
            'two distance scales': ('object_rule=true\ntracker="fused"\n'
                                    'reference_depth_m=0.3\ntarget_extent_m=0.05'),
            'negative distance': 'object_rule=true\ntracker="fused"\nreference_depth_m=-0.3',
            'unknown device': 'model_device="tpu"',
            'a number quoted as text': 'tracker="fused"\nreference_depth_m="0.3"',
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'profile.toml'
            for label, body in cases.items():
                with self.subTest(case=label):
                    path.write_text(f'schema_version=1\n[servo]\n'
                                    f'[perception]\n{body}\n')
                    with self.assertRaises(ValueError):
                        load_profile(path)

    def test_a_profile_cannot_enable_two_servo_modes(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'profile.toml'
            path.write_text(
                'schema_version=1\n[servo]\nfixed_rule=true\n'
                '[perception]\nobject_rule=true\ntracker="fused"\n'
                'reference_depth_m=0.3\n')
            with self.assertRaises(ValueError): load_profile(path)

    def test_the_shipped_object_profile_parses(self):
        profile=ROOT/'configs/visual_servo/object_rule_example.toml'
        values=load_profile(profile)
        self.assertTrue(values['object_rule'])
        self.assertEqual(values['tracker'],'fused')
        self.assertNotIn('detector_weights',values)
