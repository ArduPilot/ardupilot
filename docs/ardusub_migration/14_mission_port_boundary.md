# Mission dashboard port boundary

The ROS 2 reference's Mission Builder is a companion-side queue of local north/east/depth/yaw setpoints. Its `/api/mission/set_queue`, `/start`, `/pause`, `/abort`, and `/skip` routes update Python state and publish ROS controller setpoints (`ATAL_controller/web_dashboard/app.py`, around lines 1736–1982). Its `status` response includes distance and hold timing calculated by that worker. Those values are not ArduSub mission telemetry.

ArduSub's `MISSION_CURRENT` reports current item, total, mission state, and mode (`libraries/GCS_MAVLink/GCS_Common.cpp`, `send_mission_current`). The new card uses only that report. ArduSub emits it in the extended-status stream (`libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp`, `STREAM_EXTENDED_STATUS_msgs`); the software replay sets `SR3_EXT_STAT=2`. It does not retrieve waypoint definitions or calculate distance/hold time.

Do not map the ROS queue directly to AUTO yet. `ModeAuto::init` rejects entry unless `position_ok()` and a mission are present (`ArduSub/mode_auto.cpp`). `MAV_CMD_MISSION_START` also requires motors armed and a successful AUTO mode transition (`ArduSub/GCS_MAVLink_Sub.cpp`). The present virtual DVL profile selects external horizontal *velocity* only (`integration/atal_dvl_ekf_replay.parm`); it does not provide a validated position estimate. The generic mission-item parser supports global/relative/terrain frames, not the ROS queue's local NED frame (`libraries/AP_Mission/AP_Mission.cpp`, `mavlink_int_to_mission_cmd`). A coordinate-origin and depth-frame policy needs explicit validation before any conversion or upload.

The combined virtual Xsens/Bar30/A50 replay has reported both `EKF_CONST_POS_MODE` with attitude/velocity flags and `EKF_UNINITIALIZED` across 15-second runs; neither had an absolute horizontal-position flag. That variability and missing position are direct software evidence for keeping mission commands disabled. The read-only dashboard displays these report flags without claiming to implement `position_ok()`.

## ROS waypoint contract to preserve

The reference dashboard creates each queued waypoint with `north`, `east`, `depth`, `yaw`, `max_vel`, `tolerance`, `hold_sec`, and a display-only `status` (`web_dashboard/templates/index.html`, mission builder). The worker treats north/east as local metres, depth as positive-down metres, yaw as degrees, `max_vel` as the ROS position controller's horizontal speed limit, `tolerance` as horizontal arrival distance in metres, and `hold_sec` as time spent inside that tolerance. The worker's arrival check currently uses **horizontal distance only**; it does not gate hold completion on depth or yaw. The port should preserve or deliberately revise that behavior, rather than silently copying fields into a MAVLink mission item.

| ROS field | Required ArduSub decision before upload |
| --- | --- |
| `north`, `east` | A verified local-to-global origin and yaw alignment; ArduSub's mission parser does not accept the local NED queue directly. |
| `depth` | Positive-down reference and conversion to the mission frame's altitude sign/reference, verified against Bar30 and EKF origin. |
| `yaw` | Whether a separate yaw command is needed at each waypoint, with ACK and completion checks; do not assume a waypoint item preserves it. |
| `max_vel` | A bounded speed command and restoration policy; not an intrinsic waypoint coordinate. |
| `tolerance`, `hold_sec` | Horizontal arrival and dwell semantics matched to the old worker; decide whether depth/yaw gates should be added, with timeout/cancel behavior. |
| `status` | Companion display state only; it must come from ArduSub mission/current/ACK telemetry after porting. |

The A50 `position_local` report is **not** a ready-made origin: [Water Linked's dead-reckoning guide](https://docs.waterlinked.com/dvl/dead-reckoning/) says its frame is reset/startup-relative, its yaw zeroes on reset, and in-air drift can be significant. The bridge now parses and can display that report for alignment work, but never forwards it as ArduSub position. A mission transfer client remains blocked on a validated navigation frame, estimator readiness and a concrete mapping of these behavioral fields.

`integration/mission_queue.py` is the first non-actuating contract boundary. It requires all seven behavior fields, finite numbers, positive speed/tolerance, nonnegative depth/dwell, and no unrecognized keys; the old UI's `status: pending` may be present but is not a vehicle command. It returns a complete local waypoint queue for later conversion. `POST /api/mission/validate` exposes this check on the local dashboard API and explicitly returns `upload_ready: false`; it stores nothing or sends MAVLink. These are structural checks, not vehicle-specific speed/depth limits or proof that the queue can be uploaded. The dashboard Mission Builder now uses this route for local add/edit/import/export drafts; it still sends no mission commands.

Next software-only steps:

1. Establish a replayed, observable navigation source and verify `position_ok()` and estimator origin behavior without real motors. Do not infer position health from fresh `LOCAL_POSITION_NED` alone.
2. Define a mission data contract for each ROS waypoint field: frame/origin, depth sign and reference, yaw, tolerance, hold duration, and speed. Reject unsupported fields rather than silently dropping them.
3. Implement a separate, explicit ArduSub mission transfer client with sequence, acknowledgement, timeout, rollback, and read-back checks. Keep upload distinct from execution and from the read-only dashboard API.
4. Exercise upload/read-back and start/pause/abort/skip behavior in SITL with disarmed and fault cases. Verify mode/arming acknowledgements and neutral/failsafe behavior before surfacing active controls in the familiar dashboard shell.

Physical sensor mounting, actuator wiring, E-stop behavior, and wet mission commissioning remain the final supervised hardware phase. The current card is operational status only.

## 2026-11-25 software implementation update

`integration/mission_client.py` (`local_ned_to_global`, `ros_waypoints_to_mission_items`, `MissionClient`, `GuidedMissionRunner`) implements ArduSub 4.7 mission upload/readback (`MAV_FRAME_GLOBAL_INT` + `MAV_CMD_NAV_WAYPOINT`) and GUIDED local-NED waypoint sequencing (`WP_SPD` in m/s), wired into `dashboard_api.py --enable-control` (`/api/mission/upload`, `/readback`, `/set_queue`, `/start`, `/pause`, `/abort`, `/skip`, `/status`). `test_ardusub_control_and_mission.py`, `test_native_end_to_end.py` (4-item native upload/readback), and `test_closed_loop_dynamics.py` (`GuidedMissionRunner` arrival in 6-DoF sim) verify the software workflow; wet pool verification remains hardware-stage work.
