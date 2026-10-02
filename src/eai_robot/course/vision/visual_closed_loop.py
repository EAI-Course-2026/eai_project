"""Black-ball image-based closed-loop control for the course SCS215 arm.

The script uses this repository's SCS215-adapted ``SOFollower`` and reuses the
Week 4 Task 2 calibration mapping, FK, IK and Cartesian line planner.  Without
``--execute`` it opens only the wrist camera and never opens the motor bus.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np
from eai_robot.course.input import keyboard

from lerobot.cameras import ColorMode
from lerobot.cameras.opencv import OpenCVCameraConfig
from eai_robot.config import default_camera_index
from eai_robot.course.robot import CourseFollower as SO101Follower, make_robot, DEFAULT_CALIBRATION, default_port

from eai_robot.course.kinematics.cartesian_planner import CartesianLinePlanner, CartesianPlan  # noqa: E402
from eai_robot.course.kinematics.joint_mapping import ARM_JOINT_NAMES, SO101JointMapper  # noqa: E402
from eai_robot.course.kinematics.kinematics_backend import create_fk_solver  # noqa: E402
from eai_robot.course.kinematics.position_ik import create_position_ik_solver  # noqa: E402
from eai_robot.course.kinematics.run_steps3_to5 import (  # noqa: E402
    arm_degrees_from_raw,
    ensure_execution_start_is_safe,
    raw_outside_calibration,
    read_raw_motors,
)

from eai_robot.course.vision.ball_detector import BallDetection, BlackBallDetector, annotate_frame
from eai_robot.course.vision.visual_math import AXIS_INDEX, axis_delta, damped_image_step, validate_image_jacobian


DEFAULT_URDF = Path(__file__).resolve().parents[1] / "kinematics/so101_new_calib.urdf"
WINDOW_NAME = "EAI visual servo - Q/Esc stop, Space emergency stop"


@dataclass(frozen=True)
class DepthCalibration:
    axis_name: str
    radius_derivative_px_per_m: float

    @property
    def approach_sign(self) -> float:
        return 1.0 if self.radius_derivative_px_per_m > 0 else -1.0


class StopState:
    def __init__(self) -> None:
        self.armed = threading.Event()
        self.stop_requested = threading.Event()
        self.emergency_requested = threading.Event()

    def on_press(self, key: keyboard.Key | keyboard.KeyCode) -> bool | None:
        # Ignore keys typed into the mandatory startup confirmation prompt.
        if not self.armed.is_set():
            return None
        if key == keyboard.Key.space:
            self.emergency_requested.set()
            self.stop_requested.set()
            return False
        if key == keyboard.Key.esc:
            self.stop_requested.set()
            return False
        char = getattr(key, "char", None)
        if char is not None and char.lower() == "q":
            self.stop_requested.set()
            return False
        return None


class StopRequestedError(RuntimeError):
    """Raised between motor waypoints so the outer safety cleanup runs."""


class PreviewWindow:
    """Small Tk/Pillow preview that also works with opencv-python-headless."""

    def __init__(self, stop_state: StopState) -> None:
        try:
            import tkinter as tk
            from PIL import Image, ImageTk
        except ImportError as exc:
            raise RuntimeError(
                "Preview requires tkinter and Pillow. Re-run without --preview if unavailable."
            ) from exc

        self._tk = tk
        self._image_module = Image
        self._image_tk_module = ImageTk
        try:
            self._root = tk.Tk()
        except tk.TclError as exc:
            raise RuntimeError(
                "Could not create a Windows preview. Re-run without --preview."
            ) from exc
        self._root.title(WINDOW_NAME)
        self._root.protocol("WM_DELETE_WINDOW", self._request_close)
        self._label = tk.Label(self._root)
        self._label.pack()
        self._photo = None
        self._stop_state = stop_state

    def _request_close(self) -> None:
        self._stop_state.stop_requested.set()
        self._root.withdraw()

    def show(
        self,
        frame_rgb: np.ndarray,
        detection: BallDetection | None,
        status: str,
    ) -> None:
        annotated_bgr = annotate_frame(frame_rgb, detection, status)
        annotated_rgb = cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB)
        image = self._image_module.fromarray(annotated_rgb)
        self._photo = self._image_tk_module.PhotoImage(image=image)
        self._label.configure(image=self._photo)
        try:
            self._root.update_idletasks()
            self._root.update()
        except self._tk.TclError:
            self._stop_state.stop_requested.set()

    def close(self) -> None:
        try:
            self._root.destroy()
        except self._tk.TclError:
            pass


class VisionArmRuntime:
    """Single owner of the camera, SCS215 bus, IK state and torque policy."""

    def __init__(self, args: argparse.Namespace, stop_state: StopState) -> None:
        if type(args.camera_index) is not int or args.camera_index < 0:
            raise ValueError("Select this machine's camera using --camera-index or hardware.local.toml")
        camera_config = OpenCVCameraConfig(
            index_or_path=args.camera_index,
            fps=args.camera_fps,
            width=args.camera_width,
            height=args.camera_height,
            color_mode=ColorMode.RGB,
            warmup_s=1,
        )
        self.robot = make_robot(
            args.port, args.robot_id, calibration_path=args.calibration,
            cameras={args.camera_key: camera_config},
        )
        self.camera_key = args.camera_key
        self.camera = self.robot.cameras[self.camera_key]
        self.args = args
        self.stop_state = stop_state

        self.mapper: SO101JointMapper | None = None
        self.fk = None
        self.planner: CartesianLinePlanner | None = None
        self.fk_backend = "not initialized"
        self.ik_backend = "not initialized"
        self.command_q: np.ndarray | None = None
        self.command_position: np.ndarray | None = None
        self.initial_position: np.ndarray | None = None
        self.torque_enabled = False

    def connect(self, *, camera_only: bool) -> None:
        if camera_only:
            self.camera.connect()
            return

        if not self.robot.calibration:
            raise FileNotFoundError(
                f"No calibration found at {self.robot.calibration_fpath}. "
                "Run lerobot-calibrate for this exact arm first."
            )
        self.mapper = SO101JointMapper(self.robot.calibration)
        issues = self.mapper.validate()
        if issues:
            details = "; ".join(
                f"{issue.level} {issue.joint}: {issue.message}" for issue in issues
            )
            raise RuntimeError(f"Calibration check failed: {details}")

        self.fk, self.fk_backend = create_fk_solver(self.args.urdf)
        ik, self.ik_backend = create_position_ik_solver(
            self.fk,
            self.fk_backend,
            tolerance_m=min(0.0004, self.args.max_step_mm / 1000.0 * 0.20),
            joint_margin_deg=self.args.joint_margin_deg,
        )
        self.planner = CartesianLinePlanner(
            self.fk,
            ik,
            max_cartesian_step_m=self.args.max_step_mm / 1000.0,
            max_joint_step_deg=self.args.max_joint_step_deg,
        )

        self.robot.bus.connect(handshake=False)
        self.robot.bus.port_handler.clearPort()
        time.sleep(0.10)
        raw = read_raw_motors(self.robot, ARM_JOINT_NAMES)
        outside = raw_outside_calibration(self.mapper, raw)
        if outside:
            details = ", ".join(
                f"{name}={raw[name]} "
                f"(range {self.mapper.calibration[name].range_min}.."
                f"{self.mapper.calibration[name].range_max})"
                for name in outside
            )
            raise RuntimeError(f"Raw positions outside calibration: {details}")

        self.command_q = arm_degrees_from_raw(self.mapper, raw)
        ensure_execution_start_is_safe(
            self.command_q, margin_deg=self.args.joint_margin_deg
        )
        self.command_position = self.fk.forward_kinematics(self.command_q)[:3, 3]
        self.initial_position = self.command_position.copy()

        # Initialize and validate the comparatively timing-sensitive protocol-1
        # motor bus before starting the continuous USB camera read thread.
        self.camera.connect()

    def enable_motion(self) -> None:
        mapper = self._require_mapper()
        if self.command_q is None:
            raise RuntimeError("arm state has not been initialized")
        raw = read_raw_motors(self.robot, ARM_JOINT_NAMES)
        current_goals = {
            name: mapper.raw_to_normalized(name, raw[name]) for name in ARM_JOINT_NAMES
        }
        # Seed the measured pose before torque-on to prevent a stale goal jump.
        self.robot.bus.sync_write("Goal_Position", current_goals)
        self.torque_enabled = True
        self.robot.enable_motion(list(ARM_JOINT_NAMES))

    def read_frame(self) -> np.ndarray:
        last_error: TimeoutError | None = None
        for attempt in range(1, self.args.camera_read_retries + 1):
            try:
                return self.camera.async_read(timeout_ms=self.args.camera_timeout_ms)
            except TimeoutError as exc:
                last_error = exc
                if attempt < self.args.camera_read_retries:
                    print(
                        f"\nCamera frame timeout ({attempt}/"
                        f"{self.args.camera_read_retries}); retrying...",
                        file=sys.stderr,
                    )
        raise TimeoutError(
            f"机械臂摄像头 {self.args.camera_index} 连续 "
            f"{self.args.camera_read_retries} 次取帧超时。请关闭占用摄像头的软件，"
            "重新插拔摄像头后运行 lerobot-find-cameras opencv。"
        ) from last_error

    def move_to(self, target_position: Sequence[float]) -> CartesianPlan:
        mapper = self._require_mapper()
        if self.planner is None or self.command_q is None or self.command_position is None:
            raise RuntimeError("Cartesian planner is not initialized")
        if not self.torque_enabled:
            raise RuntimeError("refusing motion while torque is disabled")

        target = np.asarray(target_position, dtype=float).reshape(3)
        plan = self.planner.plan(self.command_q, target)
        next_send = time.perf_counter()
        for waypoint in plan.waypoints:
            if self.stop_state.stop_requested.is_set():
                raise StopRequestedError("stop requested during Cartesian motion")
            action = {
                f"{name}.pos": mapper.urdf_degrees_to_normalized(name, value)
                for name, value in zip(
                    ARM_JOINT_NAMES, waypoint.joint_degrees, strict=True
                )
            }
            self.robot.send_action(action)
            self.command_q = waypoint.joint_degrees.copy()
            self.command_position = waypoint.achieved_position.copy()
            next_send += self.args.motion_period
            sleep_s = next_send - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
        return plan

    def move_delta(self, delta_xyz_m: Sequence[float]) -> CartesianPlan:
        if self.command_position is None:
            raise RuntimeError("arm state has not been initialized")
        return self.move_to(self.command_position + np.asarray(delta_xyz_m, dtype=float))

    def tracking_error_deg(self) -> float:
        if self.command_q is None:
            raise RuntimeError("arm state has not been initialized")
        measured_q = self.measured_joint_degrees()
        return float(np.max(np.abs(measured_q - self.command_q)))

    def measured_joint_degrees(self) -> np.ndarray:
        mapper = self._require_mapper()
        measured_raw = read_raw_motors(self.robot, ARM_JOINT_NAMES)
        return arm_degrees_from_raw(mapper, measured_raw)

    def measured_position(self) -> np.ndarray:
        if self.fk is None:
            raise RuntimeError("FK solver is not initialized")
        measured_q = self.measured_joint_degrees()
        return self.fk.forward_kinematics(measured_q)[:3, 3]

    def travel_from_start_m(self, position: np.ndarray | None = None) -> float:
        if self.initial_position is None or self.command_position is None:
            return 0.0
        value = self.command_position if position is None else position
        return float(np.linalg.norm(value - self.initial_position))

    def shutdown(self) -> None:
        if self.torque_enabled and self.robot.bus.is_connected:
            try:
                self.robot.bus.disable_torque(list(ARM_JOINT_NAMES), num_retry=5)
                print("Torque disabled on SCS215 arm joints ID 1-5.")
            except Exception as exc:
                print(f"WARNING: could not disable torque: {exc}", file=sys.stderr)
                print("Disconnect servo power immediately.", file=sys.stderr)
            self.torque_enabled = False
        if self.robot.bus.is_connected:
            self.robot.bus.disconnect(disable_torque=False)
        if self.camera.is_connected:
            self.camera.disconnect()

    def _require_mapper(self) -> SO101JointMapper:
        if self.mapper is None:
            raise RuntimeError("SCS215 calibration mapper is not initialized")
        return self.mapper


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Black-ball image-based closed-loop control for the SCS215 course arm."
    )
    parser.add_argument("--port", default=default_port())
    parser.add_argument("--robot-id", default="scs215_so101")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--camera-index", type=int, default=default_camera_index(),
                        help="Local camera index; set in hardware.local.toml or pass explicitly")
    parser.add_argument("--camera-key", default="wrist")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=15)
    parser.add_argument("--camera-timeout-ms", type=int, default=5000)
    parser.add_argument("--camera-read-retries", type=int, default=3)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--execute", action="store_true", help="Allow automatic arm motion")
    parser.add_argument("--preview", action="store_true", help="Show a Tk/Pillow camera preview")
    parser.add_argument("--black-threshold", type=int, default=75)
    parser.add_argument("--control-axes", nargs=2, choices=("x", "y", "z"), default=("x", "z"))
    parser.add_argument("--approach", action="store_true", help="Also approach the ball using its radius")
    parser.add_argument("--depth-axis", choices=("x", "y", "z"), default="y")
    parser.add_argument("--probe-mm", type=float, default=3.0)
    parser.add_argument(
        "--depth-probe-mm",
        type=float,
        default=12.0,
        help="Larger reversible probe used only to learn the approach direction",
    )
    parser.add_argument("--probe-settle-s", type=float, default=1.0)
    parser.add_argument("--max-step-mm", type=float, default=1.5)
    parser.add_argument("--max-travel-mm", type=float, default=40.0)
    parser.add_argument("--gain", type=float, default=0.35)
    parser.add_argument("--damping-px-per-m", type=float, default=250.0)
    parser.add_argument("--deadband-px", type=float, default=24.0)
    parser.add_argument("--stable-frames", type=int, default=8)
    parser.add_argument("--control-hz", type=float, default=8.0)
    parser.add_argument("--feedback-hz", type=float, default=2.0)
    parser.add_argument("--max-tracking-error-deg", type=float, default=12.0)
    parser.add_argument("--max-runtime-s", type=float, default=60.0)
    parser.add_argument("--target-radius-px", type=float, default=75.0)
    parser.add_argument("--approach-step-mm", type=float, default=0.4)
    parser.add_argument("--max-approach-mm", type=float, default=15.0)
    parser.add_argument("--approach-gate-px", type=float, default=180.0)
    parser.add_argument("--min-radius-change-px", type=float, default=0.25)
    parser.add_argument("--joint-margin-deg", type=float, default=2.0)
    parser.add_argument("--max-joint-step-deg", type=float, default=5.0)
    parser.add_argument("--motion-period", type=float, default=0.10)
    parser.add_argument("--feature-samples", type=int, default=5)
    parser.add_argument("--torque-settle-s", type=float, default=2.0)
    parser.add_argument("--detection-timeout-s", type=float, default=8.0)
    args = parser.parse_args()
    if args.camera_index is None or args.camera_index < 0:
        parser.error("Set --camera-index or [camera].index_or_path in hardware.local.toml")

    if args.control_axes[0] == args.control_axes[1]:
        parser.error("--control-axes must contain two different axes")
    if args.approach and args.depth_axis in args.control_axes:
        parser.error("--depth-axis must be different from both --control-axes")
    if not 1.0 <= args.probe_mm <= 8.0:
        parser.error("--probe-mm must be within 1..8")
    if not 4.0 <= args.depth_probe_mm <= 20.0:
        parser.error("--depth-probe-mm must be within 4..20")
    if not 0.2 <= args.probe_settle_s <= 3.0:
        parser.error("--probe-settle-s must be within 0.2..3")
    if not 0.5 <= args.max_step_mm <= 3.0:
        parser.error("--max-step-mm must be within 0.5..3")
    if not 10.0 <= args.max_travel_mm <= 220.0:
        parser.error("--max-travel-mm must be within 10..220")
    if not 2.0 <= args.control_hz <= 15.0:
        parser.error("--control-hz must be within 2..15")
    if not 1 <= args.feature_samples <= 15:
        parser.error("--feature-samples must be within 1..15")
    if not 0.5 <= args.torque_settle_s <= 5.0:
        parser.error("--torque-settle-s must be within 0.5..5")
    if not 3.0 <= args.detection_timeout_s <= 20.0:
        parser.error("--detection-timeout-s must be within 3..20")
    if not 500 <= args.camera_timeout_ms <= 10000:
        parser.error("--camera-timeout-ms must be within 500..10000")
    if not 1 <= args.camera_read_retries <= 5:
        parser.error("--camera-read-retries must be within 1..5")
    if args.max_runtime_s < 0:
        parser.error("--max-runtime-s must be non-negative")
    if args.target_radius_px <= 0:
        parser.error("--target-radius-px must be positive")
    if not 0.1 <= args.approach_step_mm <= args.max_step_mm:
        parser.error("--approach-step-mm must be within 0.1..max-step-mm")
    if not 1.0 <= args.max_approach_mm <= min(200.0, args.max_travel_mm):
        parser.error(
            "--max-approach-mm must be within 1..200 and no greater than "
            "--max-travel-mm"
        )
    if args.approach_gate_px <= args.deadband_px:
        parser.error("--approach-gate-px must be greater than --deadband-px")
    if args.min_radius_change_px <= 0:
        parser.error("--min-radius-change-px must be positive")
    return args


def save_png(path: Path, image: np.ndarray) -> None:
    """Save a PNG through a Unicode-safe path on Windows."""
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError(f"OpenCV 无法编码诊断图：{path}")
    try:
        path.write_bytes(encoded.tobytes())
    except OSError as exc:
        raise RuntimeError(f"无法保存诊断图：{path}: {exc}") from exc


def median_detection(
    runtime: VisionArmRuntime,
    detector: BlackBallDetector,
    sample_count: int,
    timeout_s: float | None = None,
) -> BallDetection:
    detections: list[BallDetection] = []
    last_frame: np.ndarray | None = None
    last_mask: np.ndarray | None = None
    if timeout_s is None:
        timeout_s = runtime.args.detection_timeout_s
    deadline = time.monotonic() + timeout_s
    while len(detections) < sample_count and time.monotonic() < deadline:
        last_frame = runtime.read_frame()
        detection, last_mask = detector.detect(last_frame)
        if detection is not None:
            detections.append(detection)
    if len(detections) < sample_count:
        debug_dir = VISION_DIR / "outputs" / "vision_debug"
        debug_dir.mkdir(parents=True, exist_ok=True)
        frame_path = debug_dir / "last_detection_failure.png"
        mask_path = debug_dir / "last_detection_mask.png"
        saved_paths: list[Path] = []
        if last_frame is not None:
            save_png(frame_path, cv2.cvtColor(last_frame, cv2.COLOR_RGB2BGR))
            saved_paths.append(frame_path)
        if last_mask is not None:
            save_png(mask_path, last_mask)
            saved_paths.append(mask_path)
        saved_text = " 和 ".join(str(path) for path in saved_paths)
        raise RuntimeError(
            f"无法稳定检测黑球（{len(detections)}/{sample_count} 帧）；"
            f"诊断图已保存到 {saved_text}"
        )
    values = np.median(
        np.asarray([[item.center_x, item.center_y, item.radius] for item in detections]),
        axis=0,
    )
    return BallDetection(
        center_x=float(values[0]),
        center_y=float(values[1]),
        radius=float(values[2]),
        area=float(np.median([item.area for item in detections])),
        circularity=float(np.median([item.circularity for item in detections])),
        confidence=float(np.median([item.confidence for item in detections])),
    )


def execute_probe(
    runtime: VisionArmRuntime,
    detector: BlackBallDetector,
    axis_name: str,
    probe_m: float,
    sample_count: int,
    baseline: BallDetection,
) -> np.ndarray:
    if runtime.command_position is None:
        raise RuntimeError("arm position is unavailable")
    origin = runtime.command_position.copy()
    measured_origin = runtime.measured_position()
    axis_index = AXIS_INDEX[axis_name]
    errors: list[str] = []

    for sign in (1.0, -1.0):
        requested = np.zeros(3, dtype=float)
        requested[axis_index] = sign * probe_m
        plan = runtime.move_delta(requested)
        try:
            if not plan.waypoints:
                errors.append(f"{sign:+.0f}{axis_name}: {plan.message}")
                continue
            time.sleep(runtime.args.probe_settle_s)
            measured_moved = runtime.measured_position()
            actual_delta = float(measured_moved[axis_index] - measured_origin[axis_index])
            minimum_actual_m = max(0.001, probe_m * 0.20)
            if abs(actual_delta) < minimum_actual_m:
                errors.append(
                    f"{sign:+.0f}{axis_name}: actual motion was only "
                    f"{actual_delta * 1000.0:.2f} mm"
                )
                continue
            moved = median_detection(runtime, detector, sample_count)
            return (moved.center - baseline.center) / actual_delta
        finally:
            if not runtime.stop_state.stop_requested.is_set():
                restore = runtime.move_to(origin)
                if not restore.reached_target:
                    raise RuntimeError(
                        f"探测 {axis_name} 后无法回到起点：{restore.message}；请立即停止实验"
                    )
                time.sleep(runtime.args.probe_settle_s)

    raise RuntimeError(
        f"无法沿 {axis_name.upper()} 轴完成 {probe_m * 1000:.1f} mm 探测："
        + "; ".join(errors)
    )


def learn_image_jacobian(
    runtime: VisionArmRuntime,
    detector: BlackBallDetector,
    axis_names: tuple[str, str],
    probe_m: float,
    sample_count: int,
    baseline: BallDetection | None = None,
) -> np.ndarray:
    if baseline is None:
        baseline = median_detection(runtime, detector, sample_count)
    columns = []
    print("\nLearning local image Jacobian with small Cartesian probes...")
    for axis_name in axis_names:
        print(f"  probing {axis_name.upper()} axis")
        column = execute_probe(
            runtime, detector, axis_name, probe_m, sample_count, baseline
        )
        columns.append(column)
        baseline = median_detection(runtime, detector, sample_count)

    jacobian = np.column_stack(columns)
    print("Measured image Jacobian (pixel / meter):")
    print(np.array2string(jacobian, precision=1, suppress_small=True))
    norms = np.linalg.norm(jacobian, axis=0)
    print(
        "Column motion: "
        + ", ".join(
            f"{axis.upper()}={norm:.1f} px/m"
            for axis, norm in zip(axis_names, norms, strict=True)
        )
    )
    validate_image_jacobian(jacobian)
    return jacobian


def learn_depth_response(
    runtime: VisionArmRuntime,
    detector: BlackBallDetector,
    axis_name: str,
    probe_m: float,
    sample_count: int,
    min_radius_change_px: float,
) -> DepthCalibration:
    """Determine which sign of one Cartesian axis makes the ball appear larger."""
    if runtime.command_position is None:
        raise RuntimeError("arm position is unavailable")
    origin = runtime.command_position.copy()
    axis_index = AXIS_INDEX[axis_name]
    errors: list[str] = []

    print(
        f"  probing {axis_name.upper()} axis for forward/backward response "
        f"({probe_m * 1000.0:.1f} mm)"
    )
    for sign in (1.0, -1.0):
        baseline = median_detection(runtime, detector, sample_count)
        measured_origin = runtime.measured_position()
        requested = np.zeros(3, dtype=float)
        requested[axis_index] = sign * probe_m
        plan = runtime.move_delta(requested)
        try:
            if not plan.waypoints:
                errors.append(f"{sign:+.0f}{axis_name}: {plan.message}")
                continue
            time.sleep(runtime.args.probe_settle_s)
            measured_moved = runtime.measured_position()
            actual_delta = float(measured_moved[axis_index] - measured_origin[axis_index])
            minimum_actual_m = max(0.001, probe_m * 0.20)
            if abs(actual_delta) < minimum_actual_m:
                errors.append(
                    f"{sign:+.0f}{axis_name}: actual motion was only "
                    f"{actual_delta * 1000.0:.2f} mm"
                )
                continue
            moved = median_detection(runtime, detector, sample_count)
            radius_change = moved.radius - baseline.radius
            if abs(radius_change) < min_radius_change_px:
                errors.append(
                    f"{sign:+.0f}{axis_name}: actual motion "
                    f"{actual_delta * 1000.0:+.2f} mm, radius "
                    f"{baseline.radius:.2f}->{moved.radius:.2f} px "
                    f"(change {radius_change:+.2f} px)"
                )
                continue
            derivative = float(radius_change / actual_delta)
            calibration = DepthCalibration(axis_name, derivative)
            direction = "+" if calibration.approach_sign > 0 else "-"
            print(
                f"Depth response: dr/d{axis_name}={derivative:+.1f} px/m; "
                f"approach direction is {direction}{axis_name.upper()}"
            )
            return calibration
        finally:
            if not runtime.stop_state.stop_requested.is_set():
                restore = runtime.move_to(origin)
                if not restore.reached_target:
                    raise RuntimeError(
                        f"深度探测 {axis_name} 后无法回到起点：{restore.message}；请立即停止实验"
                    )
                time.sleep(runtime.args.probe_settle_s)

    raise RuntimeError(
        f"无法通过黑球半径判断 {axis_name.upper()} 轴的前进方向：" + "; ".join(errors)
    )


def run_detection_only(
    runtime: VisionArmRuntime,
    detector: BlackBallDetector,
    preview: PreviewWindow | None,
) -> int:
    print("VISION-ONLY mode: motor serial port is not opened and no movement is possible.")
    print("Press Q, Esc, or Ctrl+C to stop.")
    while not runtime.stop_state.stop_requested.is_set():
        frame = runtime.read_frame()
        detection, _ = detector.detect(frame)
        if detection is None:
            status = "TARGET LOST"
        else:
            height, width = frame.shape[:2]
            status = (
                f"ex={detection.center_x - width / 2:+.0f}px "
                f"ey={detection.center_y - height / 2:+.0f}px "
                f"r={detection.radius:.1f}px conf={detection.confidence:.2f}"
            )
        print(f"\r{status:72s}", end="", flush=True)
        if preview is not None:
            preview.show(frame, detection, "VISION ONLY")
    print()
    return 0


def run_closed_loop(
    runtime: VisionArmRuntime,
    detector: BlackBallDetector,
    args: argparse.Namespace,
    stop_state: StopState,
    preview: PreviewWindow | None,
) -> int:
    assert runtime.command_position is not None
    print(f"FK backend: {runtime.fk_backend}")
    print(f"IK backend: {runtime.ik_backend}")
    print(f"Start TCP xyz (mm): {np.round(runtime.command_position * 1000.0, 2)}")
    print(f"Control axes: {args.control_axes[0].upper()}, {args.control_axes[1].upper()}")
    print("Q/Esc: normal stop; Space: emergency torque-off")

    initial = median_detection(runtime, detector, args.feature_samples)
    print(
        f"Initial ball: ({initial.center_x:.1f}, {initial.center_y:.1f}), "
        f"radius={initial.radius:.1f}, confidence={initial.confidence:.2f}"
    )
    long_approach = args.approach and args.max_approach_mm > 80.0
    if long_approach:
        print(
            "WARNING: LONG APPROACH ENABLED. The program has no collision or "
            "distance sensor. Verify measured free space exceeds "
            f"{args.max_approach_mm + 50.0:.0f} mm and keep a hand on Space."
        )
        phrase = f"START VISUAL SERVO {args.max_approach_mm:.0f}MM"
    else:
        phrase = "START VISUAL SERVO"
    if input(f'Type exactly "{phrase}" to enable torque: ').strip() != phrase:
        print("Confirmation did not match. No movement command was sent.")
        return 0

    runtime.enable_motion()
    stop_state.armed.set()
    print(f"Torque enabled; waiting {args.torque_settle_s:.1f}s for the arm and camera to settle...")
    time.sleep(args.torque_settle_s)
    post_torque_detection = median_detection(runtime, detector, args.feature_samples)
    print(
        f"Ball reacquired after torque-on: ({post_torque_detection.center_x:.1f}, "
        f"{post_torque_detection.center_y:.1f}), radius={post_torque_detection.radius:.1f}"
    )
    jacobian = learn_image_jacobian(
        runtime,
        detector,
        tuple(args.control_axes),
        args.probe_mm / 1000.0,
        args.feature_samples,
        baseline=post_torque_detection,
    )
    depth_calibration: DepthCalibration | None = None
    depth_origin_m = 0.0
    if args.approach:
        print("\nLearning forward direction from apparent ball size...")
        depth_calibration = learn_depth_response(
            runtime,
            detector,
            args.depth_axis,
            args.depth_probe_mm / 1000.0,
            args.feature_samples,
            args.min_radius_change_px,
        )
        assert runtime.command_position is not None
        depth_origin_m = float(runtime.command_position[AXIS_INDEX[args.depth_axis]])
        print(
            f"Approach enabled: target radius={args.target_radius_px:.1f}px, "
            f"maximum forward travel={args.max_approach_mm:.1f}mm"
        )

    stable_count = 0
    tracking_paused = False
    last_seen = time.monotonic()
    next_control = time.perf_counter()
    next_feedback = next_control
    started = next_control
    control_period = 1.0 / args.control_hz
    feedback_period = 1.0 / args.feedback_hz

    while not stop_state.stop_requested.is_set():
        now = time.perf_counter()
        if args.max_runtime_s > 0 and now - started >= args.max_runtime_s:
            print("\nMaximum runtime reached; stopping normally.")
            break

        frame = runtime.read_frame()
        detection, _ = detector.detect(frame)
        status = "TRACKING"

        if now >= next_feedback:
            tracking_error = runtime.tracking_error_deg()
            tracking_paused = tracking_error > args.max_tracking_error_deg
            if tracking_paused:
                status = f"TRACKING PAUSED ({tracking_error:.1f} deg joint error)"
            next_feedback = now + feedback_period

        if detection is None:
            stable_count = 0
            if now - last_seen > 0.8:
                status = "TARGET LOST - HOLDING"
        else:
            last_seen = now
            height, width = frame.shape[:2]
            desired_center = np.asarray([width / 2.0, height / 2.0])
            pixel_error = desired_center - detection.center
            pixel_error_norm = float(np.linalg.norm(pixel_error))
            aligned = bool(np.all(np.abs(pixel_error) <= args.deadband_px))
            approach_progress_m = 0.0
            approach_requested = False
            approach_done = True
            if depth_calibration is not None:
                depth_value = float(
                    runtime.command_position[AXIS_INDEX[depth_calibration.axis_name]]
                )
                approach_progress_m = max(
                    0.0,
                    depth_calibration.approach_sign * (depth_value - depth_origin_m),
                )
                approach_done = (
                    detection.radius >= args.target_radius_px
                    or approach_progress_m >= args.max_approach_mm / 1000.0
                )
                approach_requested = (
                    not approach_done and pixel_error_norm <= args.approach_gate_px
                )

            stable_count = stable_count + 1 if aligned and approach_done else 0
            if stable_count >= args.stable_frames:
                status = "LOCKED"
            elif tracking_paused:
                pass
            elif approach_requested:
                status = (
                    f"APPROACH ex={pixel_error[0]:+.0f} ey={pixel_error[1]:+.0f} "
                    f"r={detection.radius:.1f}px d={approach_progress_m * 1000.0:.1f}mm"
                )
            else:
                status = (
                    f"TRACKING ex={pixel_error[0]:+.0f} ey={pixel_error[1]:+.0f} "
                    f"r={detection.radius:.1f}px"
                )

            if (
                (not aligned or approach_requested)
                and not tracking_paused
                and now >= next_control
            ):
                delta_xyz = np.zeros(3, dtype=float)
                if not aligned:
                    axis_step = damped_image_step(
                        jacobian,
                        pixel_error,
                        gain=args.gain,
                        damping_px_per_m=args.damping_px_per_m,
                        max_step_m=args.max_step_mm / 1000.0,
                    )
                    delta_xyz += axis_delta(tuple(args.control_axes), axis_step)
                if approach_requested and depth_calibration is not None:
                    remaining_m = max(
                        0.0,
                        args.max_approach_mm / 1000.0 - approach_progress_m,
                    )
                    depth_step_m = min(args.approach_step_mm / 1000.0, remaining_m)
                    delta_xyz[AXIS_INDEX[depth_calibration.axis_name]] += (
                        depth_calibration.approach_sign * depth_step_m
                    )

                delta_norm = float(np.linalg.norm(delta_xyz))
                max_step_m = args.max_step_mm / 1000.0
                if delta_norm > max_step_m:
                    delta_xyz *= max_step_m / delta_norm
                proposed = runtime.command_position + delta_xyz
                if runtime.travel_from_start_m(proposed) > args.max_travel_mm / 1000.0:
                    status = "TRAVEL LIMIT - HOLDING"
                else:
                    plan = runtime.move_to(proposed)
                    if not plan.waypoints:
                        status = "IK NO SOLUTION - HOLDING"
                    elif plan.ik_failed:
                        status = "IK PARTIAL RECOVERY"
                next_control = time.perf_counter() + control_period

        print(f"\r{status:45s}", end="", flush=True)
        if preview is not None:
            preview.show(frame, detection, status)

    print()
    return 0


def main() -> int:
    args = parse_args()
    if not args.urdf.is_file():
        raise FileNotFoundError(f"URDF not found: {args.urdf}")

    detector = BlackBallDetector(black_threshold=args.black_threshold)
    stop_state = StopState()
    runtime = VisionArmRuntime(args, stop_state)
    listener: keyboard.Listener | None = None
    preview: PreviewWindow | None = None
    failed = False
    try:
        runtime.connect(camera_only=not args.execute)
        listener = keyboard.Listener(on_press=stop_state.on_press)
        listener.start()
        if args.preview:
            preview = PreviewWindow(stop_state)
        if not args.execute:
            stop_state.armed.set()
            return run_detection_only(runtime, detector, preview)

        return run_closed_loop(runtime, detector, args, stop_state, preview)
    except StopRequestedError as exc:
        print(f"\n{exc}; stopping.")
        return 130 if stop_state.emergency_requested.is_set() else 0
    except KeyboardInterrupt:
        failed = True
        stop_state.emergency_requested.set()
        print("\nCtrl+C received; disabling torque immediately.")
        return 130
    except Exception:
        failed = True
        stop_state.emergency_requested.set()
        raise
    finally:
        if listener is not None:
            listener.stop()
            listener.join(timeout=1.0)
        if runtime.torque_enabled and not stop_state.emergency_requested.is_set() and not failed:
            input("Support the arm, then press ENTER to disable torque.")
        runtime.shutdown()
        if preview is not None:
            preview.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        raise SystemExit(1) from None
