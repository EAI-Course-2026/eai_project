"""Hardware-free Cartesian desk model, sharing the course's URDF and solvers."""
from pathlib import Path
import math
import numpy as np

from eai_robot.course.kinematics.joint_mapping import ARM_JOINT_NAMES, SO101JointMapper
from eai_robot.course.kinematics.urdf_fk import URDFFK
from eai_robot.course.kinematics.position_ik import DampedLeastSquaresIK
from eai_robot.course.kinematics.cartesian_planner import CartesianLinePlanner

URDF = Path(__file__).parents[1] / 'course/kinematics/so101_new_calib.urdf'
PERIOD = .05
LEASE_SECONDS = .30


def vector(value, label, bound):
    if (not isinstance(value, list) or len(value) != 3 or
            any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > bound for v in value)):
        raise ValueError(f'{label}必须是三个有限数值，绝对值不超过 {bound}')
    return np.array(value, dtype=float)


def speed(value):
    if type(value) not in (int, float) or not math.isfinite(value) or not 2 <= value <= 30:
        raise ValueError('末端速度必须在 2–30 mm/s 内')
    return float(value)


class DeskCartesian:
    def __init__(self, calibration, margin=20):
        self.calibration = calibration
        self.mapper = SO101JointMapper(calibration)
        issues = [i for i in self.mapper.validate() if i.level == 'ERROR']
        if issues:
            raise ValueError('; '.join(i.message for i in issues))
        self.margin = margin
        self.fk = URDFFK(URDF, 'gripper_frame_link', ARM_JOINT_NAMES)

    def angles(self, raw):
        for name in ARM_JOINT_NAMES:
            c = self.calibration[name]
            if not c.range_min <= raw[c.id] <= c.range_max:
                raise ValueError(f'{name} 反馈 {raw[c.id]} 超出标定范围')
        return np.array([self.mapper.raw_to_urdf_degrees(n, raw[self.calibration[n].id], clip=False)
                         for n in ARM_JOINT_NAMES])

    def pose(self, raw):
        transform = self.fk.forward_kinematics(self.angles(raw))
        return {'xyz_mm': (transform[:3, 3] * 1000).tolist(),
                'rotation': transform[:3, :3].tolist()}

    def raw_targets(self, angles, previous):
        result = dict(previous)  # In particular, preserve the independent gripper.
        for name, angle in zip(ARM_JOINT_NAMES, angles, strict=True):
            c = self.calibration[name]
            normalized = self.mapper.urdf_degrees_to_normalized(name, float(angle), clip=False)
            raw = c.position((normalized + 100) / 200)
            low, high = c.bounds(self.margin)
            old = previous[c.id]
            inward = c.range_min <= old < low and old <= raw <= high or high < old <= c.range_max and low <= raw <= old
            if not c.range_min <= raw <= c.range_max or not (low <= raw <= high or inward):
                raise ValueError(f'{name} 接近关节端点；请反向移动或用关节控制调整姿态')
            result[c.id] = raw
        return result

    def planner(self, start, velocity):
        ik = DampedLeastSquaresIK(self.fk, tolerance_m=.00015, max_joint_update_deg=3)
        # Takeover is allowed throughout calibration. At an endpoint margin,
        # retain the current pose and permit inward recovery without a seed jump.
        for i, name in enumerate(ARM_JOINT_NAMES):
            c = self.calibration[name]
            angles = [self.mapper.raw_to_urdf_degrees(name, r, clip=False) for r in c.bounds(self.margin)]
            here = self.mapper.raw_to_urdf_degrees(name, start[c.id], clip=False)
            ik.lower_deg[i] = min(*angles, here)
            ik.upper_deg[i] = max(*angles, here)
        return CartesianLinePlanner(self.fk, ik, max_cartesian_step_m=velocity * PERIOD / 1000,
                                    max_joint_step_deg=3)

    def plan(self, start, xyz_mm, velocity, check=None):
        q = self.angles(start)
        target = np.asarray(xyz_mm, dtype=float)
        if np.linalg.norm(target - np.array(self.pose(start)['xyz_mm'])) > 300:
            raise ValueError('单次直线距离不能超过 300 mm')
        plan = self.planner(start, velocity).plan(q, target / 1000, on_step=check)
        previous = dict(start)
        if plan.reached_target:
            for waypoint in plan.waypoints:
                previous = self.raw_targets(waypoint.joint_degrees, previous)
        return plan

    def summary(self, plan, start, velocity):
        final_q = plan.waypoints[-1].joint_degrees if plan.waypoints else self.angles(start)
        start_pose = self.fk.forward_kinematics(self.angles(start))
        end_pose = self.fk.forward_kinematics(final_q)
        rotation = start_pose[:3, :3].T @ end_pose[:3, :3]
        angle = math.degrees(math.acos(float(np.clip((np.trace(rotation) - 1) / 2, -1, 1))))
        points = [plan.start_position * 1000] + [w.achieved_position * 1000 for w in plan.waypoints]
        stride = max(1, math.ceil(len(points) / 100))
        shown = points[::stride]
        if not np.array_equal(shown[-1], points[-1]):
            shown.append(points[-1])
        distance = float(np.linalg.norm(plan.requested_target - plan.start_position) * 1000)
        return {'reachable': plan.reached_target, 'target_mm': (plan.requested_target * 1000).tolist(),
                'path_mm': [p.tolist() for p in shown], 'distance_mm': distance, 'speed': velocity,
                'duration_s': distance / velocity + .5,
                'orientation_change_deg': angle,
                'joint_change_deg': (final_q - self.angles(start)).tolist(),
                'message': '整条直线路径可解；未执行碰撞检查' if plan.reached_target else
                    '直线路径不可完整到达；未启动运动。请缩短距离、反向移动或调整关节姿态。'}
