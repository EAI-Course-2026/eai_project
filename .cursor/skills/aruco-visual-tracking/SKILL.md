---
name: aruco-visual-tracking
description: Uses the robot Agent's bounded ArUco detection and centre-tracking tools in dry-run or explicitly operator-armed hardware mode. Use when asked to find an ArUco marker, start or inspect tracking, stop tracking, or prepare a real-arm ArUco acceptance.
---

# ArUco Visual Tracking

Default to dry-run. Hardware mode is allowed only when the current process reports
`mode=aruco_center_hardware` and the local operator completed the repository's hardware checklist.

## Workflow

1. Call `get_robot_state`. Stop on an error or an unexpected mode.
2. Call `detect_object` with a description of an ArUco marker.
3. Continue only when `found` is true and the returned `object_id` is unexpired.
4. Call `start_tracking` with that exact `object_id`; never invent or reuse an ID from another session.
5. Treat `accepted` or `running` as an in-progress task. When status is requested, call
   `get_task_status` with the returned `task_id`.
6. Call `stop_tracking` when the user asks to stop tracking, the target becomes invalid,
   a tool reports an error, or the rehearsal is complete.

Use `stop_motion` instead when the user asks to stop all motion or issues an emergency stop.
Do not retry rejected calls automatically, guess joint or motor values, open a camera, or bypass
the registered tools.

In dry-run, report `hardware_sent=false` and never claim a physical observation or movement.
In hardware mode, claim motion only after `running`, `hardware_sent=true`, and
`motion_confirmed_count>0`. On target loss,
feedback error, unknown state, stop timeout, or any rejection, stop and do not resume. A stop leaves
torque holding: tell the local operator to support the arm and complete the terminal release prompt.
Never claim torque was released unless the tool/runtime reports `torque_enabled=false`.

## Example

User: “找到 ArUco marker，开始追踪，查询状态后停止。”

Tool order: `get_robot_state` → `detect_object` → `start_tracking` →
`get_task_status` → `stop_tracking`.
