# Migration receipt — 2026-10-02

## Sources and destinations

- Application baseline before migration: `eai_project` commit `375cd10`.
- Collaborator source: `EAI-Course-2026/eai-course-lerobot` commit
  `6a077907c7989635218969ee78f5436f8faec92b` (LeRobot package 0.6.2).
- Week 4 Task 1 -> `src/eai_robot/course/poses/`.
- Week 4 Task 2 -> `src/eai_robot/course/kinematics/`, including its URDF.
- Vision -> `src/eai_robot/course/vision/`, including the black-ball target page.
- All five collaborator test files -> root `tests/test_course_*.py`.
- Original documentation and license -> `docs/course/source/`.
- Framework adaptation stays in the pinned fork dependency; this application
  repository does not duplicate the whole LeRobot framework.
- Original 0.6.1 source, lock and independent environment retained locally at
  `physical_intelligence/baselines/eai_project-375cd10/`, outside the active repo.

## Integration changes

The scripts now import a normal project package and share a single calibration
file instead of machine-specific HF cache files. COM defaults were replaced by
local port configuration. Keyboard hooks are loaded only for interactive use,
and pose control has portable terminal input. NumPy FK/IK is the consistent
default on both supported platforms.

Stored reference pose targets are preserved through raw coordinates when
calibration changes. Current software calibration matches the 2026-10-02 EEPROM
read; previous limits remain in history. The existing raw demo home is retained,
not claimed as re-taught or physically revalidated. Old and newly derived safe
range candidates remain inactive.

Course connections are read-only. Torque changes use verified writes without
unlocking EEPROM, and partial enable/release failures attempt cleanup across all
requested joints. Explicit wrist maintenance retains its confirmation workflow
and requires a local calibration copy before saving persistent limits.

## Validation

Local macOS arm64, Python 3.12.13:

| Check | Result |
| --- | --- |
| Real root `uv sync --locked` | Passed; installed 0.6.2 fork commit verified in `direct_url.json` |
| Complete application offline suite | 91 tests passed: 56 existing, 23 migrated, 12 integration |
| Separate original 0.6.1 environment | 56 original tests passed |
| Control and training dependency checks | No installed package conflicts |
| Separate training environment installation/import | Passed on Mac; same 0.6.2 commit, PyTorch 2.11.0, Accelerate 1.15.0 |
| All eight migrated command help paths | Passed without desktop keyboard hooks or hardware |
| Mapping, 10 mm offline plan, pose preview | Passed; plan residual approximately 0.023 mm |
| Wheel build and packaged URDF/pose/HTML assets | Passed |
| Native serial and LeRobot read-only inspection | Same IDs/models/limits; all torque 0, status 0 |
| Migrated hardware FK read | Passed; no movement or motor configuration writes |
| macOS / Windows hosted checks | Defined by `.github/workflows/tests.yml`; see PR checks for actual results |

Current stored limits are `183..897`, `64..737`, `196..771`, `70..681`,
`47..974`, `237..644`. Latest feedback was `498, 59, 773, 66, 249, 256`;
IDs 2/3/4 were slightly outside the stored ranges with torque off. FK output
clips these values for diagnostic computation; that is not a measured TCP pose
or authorization for Cartesian motion from that starting state.

## What this baseline permits

After the application PR's supported-platform checks pass and it is merged,
this is a reproducible baseline for developing the common scheduler, GUI/vision
integration and future voice functionality. Changes have a common source,
locked environment, calibration contract and offline regression suite.

Acceptance still requires physical joint direction/FK alignment checks,
migrated GUI/keyboard/visual motion checks under the current calibration,
camera behavior on both machines, and Windows CUDA/driver and training-job
verification. Recording and policy evaluation remain unverified. No motor
motion, calibration EEPROM writes, camera capture or training run was performed
as part of this migration receipt.
