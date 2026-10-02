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
| `calibration/scs215_com5.reference.json` | [`src/eai_robot/course/poses/poses.reference.json`](../../src/eai_robot/course/poses/poses.reference.json) | Original reference retained in the calibration binding; active hardware snapshot is separate |
| `setup_windows.cmd` | [`scripts/setup_windows.cmd`](../../scripts/setup_windows.cmd) | Replaced by locked uv control setup; CUDA setup has its own installer |
| `vision/README.md` | [`docs/course/source/vision/README.md`](../../docs/course/source/vision/README.md) | Historical document; current commands are in the shared course/environment guides |
| `vision/REPORT.md` | [`docs/course/source/vision/REPORT.md`](../../docs/course/source/vision/REPORT.md) | Historical document; current commands are in the shared course/environment guides |
| `vision/ball_detector.py` | [`src/eai_robot/course/vision/ball_detector.py`](../../src/eai_robot/course/vision/ball_detector.py) | Migrated detector/math or adapted robot/keyboard imports; camera/motion acceptance pending |
| `vision/black_ball_target.html` | [`src/eai_robot/course/vision/black_ball_target.html`](../../src/eai_robot/course/vision/black_ball_target.html) | Original target page retained |
| `vision/test_vision.py` | [`tests/test_course_vision.py`](../../tests/test_course_vision.py) | Migrated offline tests with package imports |
| `vision/visual_closed_loop.py` | [`src/eai_robot/course/vision/visual_closed_loop.py`](../../src/eai_robot/course/vision/visual_closed_loop.py) | Migrated detector/math or adapted robot/keyboard imports; camera/motion acceptance pending |
| `vision/visual_math.py` | [`src/eai_robot/course/vision/visual_math.py`](../../src/eai_robot/course/vision/visual_math.py) | Migrated detector/math or adapted robot/keyboard imports; camera/motion acceptance pending |
| `voice/README.md` | [`docs/course/source/voice/README.md`](../../docs/course/source/voice/README.md) | Historical document; current commands are in the shared course/environment guides |
| `week4/README.md` | [`docs/course/source/week4/README.md`](../../docs/course/source/week4/README.md) | Historical document; current commands are in the shared course/environment guides |
| `week4/task1/README.md` | [`docs/course/source/week4/task1/README.md`](../../docs/course/source/week4/task1/README.md) | Historical document; current commands are in the shared course/environment guides |
| `week4/task1/arm_config.py` | [`src/eai_robot/course/poses/arm_config.py`](../../src/eai_robot/course/poses/arm_config.py) | Adapted to shared control, read-only capture and portable terminal input |
| `week4/task1/control_presets.py` | [`src/eai_robot/course/poses/control_presets.py`](../../src/eai_robot/course/poses/control_presets.py) | Adapted to shared control, read-only capture and portable terminal input |
| `week4/task1/poses.json` | [`src/eai_robot/course/poses/poses.reference.json`](../../src/eai_robot/course/poses/poses.reference.json) | Original poses retained and bound to reference calibration; playback preserves raw targets |
| `week4/task1/record_pose.py` | [`src/eai_robot/course/poses/record_pose.py`](../../src/eai_robot/course/poses/record_pose.py) | Adapted to shared control, read-only capture and portable terminal input |
| `week4/task2/README.md` | [`docs/course/source/week4/task2/README.md`](../../docs/course/source/week4/task2/README.md) | Historical document; current commands are in the shared course/environment guides |
| `week4/task2/cartesian_planner.py` | [`src/eai_robot/course/kinematics/cartesian_planner.py`](../../src/eai_robot/course/kinematics/cartesian_planner.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/check_step1.py` | [`src/eai_robot/course/kinematics/check_step1.py`](../../src/eai_robot/course/kinematics/check_step1.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/check_step2_fk.py` | [`src/eai_robot/course/kinematics/check_step2_fk.py`](../../src/eai_robot/course/kinematics/check_step2_fk.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/joint_mapping.py` | [`src/eai_robot/course/kinematics/joint_mapping.py`](../../src/eai_robot/course/kinematics/joint_mapping.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/keyboard_control.py` | [`src/eai_robot/course/kinematics/keyboard_control.py`](../../src/eai_robot/course/kinematics/keyboard_control.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/kinematics_backend.py` | [`src/eai_robot/course/kinematics/kinematics_backend.py`](../../src/eai_robot/course/kinematics/kinematics_backend.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/position_ik.py` | [`src/eai_robot/course/kinematics/position_ik.py`](../../src/eai_robot/course/kinematics/position_ik.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/reindex_id5.py` | [`src/eai_robot/course/kinematics/reindex_id5.py`](../../src/eai_robot/course/kinematics/reindex_id5.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/run_steps3_to5.py` | [`src/eai_robot/course/kinematics/run_steps3_to5.py`](../../src/eai_robot/course/kinematics/run_steps3_to5.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
| `week4/task2/so101_new_calib.urdf` | [`src/eai_robot/course/kinematics/so101_new_calib.urdf`](../../src/eai_robot/course/kinematics/so101_new_calib.urdf) | Original URDF retained; physical FK alignment still requires acceptance |
| `week4/task2/test_joint_mapping.py` | [`tests/test_course_joint_mapping.py`](../../tests/test_course_joint_mapping.py) | Migrated offline tests with package imports |
| `week4/task2/test_keyboard_control.py` | [`tests/test_course_keyboard_control.py`](../../tests/test_course_keyboard_control.py) | Migrated offline tests with package imports |
| `week4/task2/test_steps3_to5.py` | [`tests/test_course_steps3_to5.py`](../../tests/test_course_steps3_to5.py) | Migrated offline tests with package imports |
| `week4/task2/test_urdf_fk.py` | [`tests/test_course_urdf_fk.py`](../../tests/test_course_urdf_fk.py) | Migrated offline tests with package imports |
| `week4/task2/urdf_fk.py` | [`src/eai_robot/course/kinematics/urdf_fk.py`](../../src/eai_robot/course/kinematics/urdf_fk.py) | Adapted package imports, shared calibration/ports and consistent NumPy backend |
