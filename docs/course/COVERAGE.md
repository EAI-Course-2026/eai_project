# Course migration coverage

All **35 files** under `examples/eai_course/` at the pinned source commit
[`6a077907c7989635218969ee78f5436f8faec92b`](https://github.com/EAI-Course-2026/eai-course-lerobot/tree/6a077907c7989635218969ee78f5436f8faec92b/examples/eai_course)
are accounted for below. The source main was checked on 2026-10-02 and still
pointed at this commit. Files are organized as a project package here rather
than keeping the original examples tree.

Source coverage means the implementation, assets, tests and documentation have
a destination. It does not mean every feature has been accepted on the actual
Mac/Windows arm. In particular, voice was documentation only; real camera,
keyboard/visual motion, data recording and CUDA training still need their own
acceptance. The root fork dependency retains the framework adaptations.

Historical Markdown bodies, the URDF/HTML assets, reference poses and reference
calibration were checked against the original during this inventory. The five
source test files run in the application suite. Source documents retain their
original content beneath a historical banner; follow [the current guide](README.md)
and [environment setup](../environments.md), not their old Conda/COM commands.

| Original path, relative to `examples/eai_course/` | Current destination | Disposition |
| --- | --- | --- |
| `HANDOFF.md` | [`docs/course/source/HANDOFF.md`](../../docs/course/source/HANDOFF.md) | Historical document; current commands are in the shared course/environment guides |
| `README.md` | [`docs/course/source/README.md`](../../docs/course/source/README.md) | Historical document; current commands are in the shared course/environment guides |
| `calibration/README.md` | [`docs/course/source/calibration/README.md`](../../docs/course/source/calibration/README.md) | Historical document; current commands are in the shared course/environment guides |
| `calibration/scs215_com5.reference.json` | [`src/eai_robot/control/planning/poses.reference.json`](../../src/eai_robot/control/planning/poses.reference.json) | Original reference retained in the calibration binding; active hardware snapshot is separate |
| `setup_windows.cmd` | [`scripts/setup_windows.cmd`](../../scripts/setup_windows.cmd) | Replaced by locked uv control setup; CUDA setup has its own installer |
| `vision/README.md` | [`docs/course/source/vision/README.md`](../../docs/course/source/vision/README.md) | Historical document; current commands are in the shared course/environment guides |
| `vision/REPORT.md` | [`docs/course/source/vision/REPORT.md`](../../docs/course/source/vision/REPORT.md) | Historical document; current commands are in the shared course/environment guides |
| `vision/detection/ball.py` | [`src/eai_robot/vision/detection/ball.py`](../../src/eai_robot/vision/detection/ball.py) | Migrated detector/math or adapted robot/keyboard imports; camera/motion acceptance pending |
| `vision/targets/black_ball.html` | [`src/eai_robot/vision/targets/black_ball.html`](../../src/eai_robot/vision/targets/black_ball.html) | Original target page retained |
| `vision/test_vision.py` | [`tests/test_course_vision.py`](../../tests/test_course_vision.py) | Migrated offline tests with package imports |
| `applications/visual_servo/ball.py` | [`src/eai_robot/applications/visual_servo/ball.py`](../../src/eai_robot/applications/visual_servo/ball.py) | Migrated detector/math or adapted robot/keyboard imports; camera/motion acceptance pending |
| `control/visual_servo/math.py` | [`src/eai_robot/control/visual_servo/math.py`](../../src/eai_robot/control/visual_servo/math.py) | Migrated detector/math or adapted robot/keyboard imports; camera/motion acceptance pending |
| `voice/README.md` | [`docs/course/source/voice/README.md`](../../docs/course/source/voice/README.md) | Historical document; current commands are in the shared course/environment guides |
| `week4/README.md` | [`docs/course/source/week4/README.md`](../../docs/course/source/week4/README.md) | Historical document; current commands are in the shared course/environment guides |
| `week4/task1/README.md` | [`docs/course/source/week4/task1/README.md`](../../docs/course/source/week4/task1/README.md) | Historical document; current commands are in the shared course/environment guides |
| `week4/task1/arm_config.py` | [`src/eai_robot/hardware/robots/joint_ranges.py`](../../src/eai_robot/hardware/robots/joint_ranges.py) | Adapted to shared control, read-only capture and portable terminal input |
| `week4/task1/control_presets.py` | [`src/eai_robot/applications/teleoperation/poses.py`](../../src/eai_robot/applications/teleoperation/poses.py) | Adapted to shared control, read-only capture and portable terminal input |
| `week4/task1/poses.json` | [`src/eai_robot/control/planning/poses.reference.json`](../../src/eai_robot/control/planning/poses.reference.json) | Original poses retained and bound to reference calibration; playback preserves raw targets |
| `week4/task1/record_pose.py` | [`src/eai_robot/applications/teleoperation/record_pose.py`](../../src/eai_robot/applications/teleoperation/record_pose.py) | Adapted to shared control, read-only capture and portable terminal input |
| `week4/task2/README.md` | [`docs/course/source/week4/task2/README.md`](../../docs/course/source/week4/task2/README.md) | Historical document; current commands are in the shared course/environment guides |
| `week4/task2/cartesian_planner.py` | [`src/eai_robot/control/kinematics/cartesian_planner.py`](../../src/eai_robot/control/kinematics/cartesian_planner.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/check_step1.py` | [`src/eai_robot/applications/tools/check_mapping.py`](../../src/eai_robot/applications/tools/check_mapping.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/check_step2_fk.py` | [`src/eai_robot/applications/tools/check_fk.py`](../../src/eai_robot/applications/tools/check_fk.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/joint_mapping.py` | [`src/eai_robot/control/kinematics/joint_mapping.py`](../../src/eai_robot/control/kinematics/joint_mapping.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/keyboard_control.py` | [`src/eai_robot/applications/teleoperation/keyboard.py`](../../src/eai_robot/applications/teleoperation/keyboard.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/kinematics_backend.py` | [`src/eai_robot/control/kinematics/kinematics_backend.py`](../../src/eai_robot/control/kinematics/kinematics_backend.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/position_ik.py` | [`src/eai_robot/control/kinematics/position_ik.py`](../../src/eai_robot/control/kinematics/position_ik.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/reindex_id5.py` | [`src/eai_robot/applications/tools/reindex.py`](../../src/eai_robot/applications/tools/reindex.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/run_steps3_to5.py` | [`src/eai_robot/applications/tools/cartesian_plan.py`](../../src/eai_robot/applications/tools/cartesian_plan.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/so101_new_calib.urdf` | [`src/eai_robot/control/kinematics/so101_new_calib.urdf`](../../src/eai_robot/control/kinematics/so101_new_calib.urdf) | Original URDF retained; physical FK alignment still requires acceptance |
| `week4/task2/test_joint_mapping.py` | [`tests/test_course_joint_mapping.py`](../../tests/test_course_joint_mapping.py) | Migrated offline tests with package imports |
| `week4/task2/test_keyboard_control.py` | [`tests/test_course_keyboard_control.py`](../../tests/test_course_keyboard_control.py) | Migrated offline tests with package imports |
| `week4/task2/test_steps3_to5.py` | [`tests/test_course_steps3_to5.py`](../../tests/test_course_steps3_to5.py) | Migrated offline tests with package imports |
| `week4/task2/test_urdf_fk.py` | [`tests/test_course_urdf_fk.py`](../../tests/test_course_urdf_fk.py) | Migrated offline tests with package imports |
| `week4/task2/urdf_fk.py` | [`src/eai_robot/control/kinematics/urdf_fk.py`](../../src/eai_robot/control/kinematics/urdf_fk.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
