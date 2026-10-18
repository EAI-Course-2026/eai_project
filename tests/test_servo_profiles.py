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
