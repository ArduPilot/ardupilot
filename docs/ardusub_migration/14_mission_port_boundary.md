# Mission dashboard port boundary

The ROS 2 reference's Mission Builder is a companion-side queue of local north/east/depth/yaw setpoints. Its `/api/mission/set_queue`, `/start`, `/pause`, `/abort`, and `/skip` routes update Python state and publish ROS controller setpoints (`ATAL_controller/web_dashboard/app.py`, around lines 1736–1982). Its `status` response includes distance and hold timing calculated by that worker. Those values are not ArduSub mission telemetry.

ArduSub's `MISSION_CURRENT` reports current item, total, mission state, and mode (`libraries/GCS_MAVLink/GCS_Common.cpp`, `send_mission_current`). The new card uses only that report. ArduSub emits it in the extended-status stream (`libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp`, `STREAM_EXTENDED_STATUS_msgs`); the software replay sets `SR3_EXT_STAT=2`. It does not retrieve waypoint definitions or calculate distance/hold time.

Do not map the ROS queue directly to AUTO yet. `ModeAuto::init` rejects entry unless `position_ok()` and a mission are present (`ArduSub/mode_auto.cpp`). `MAV_CMD_MISSION_START` also requires motors armed and a successful AUTO mode transition (`ArduSub/GCS_MAVLink_Sub.cpp`). The present virtual DVL profile selects external horizontal *velocity* only (`integration/atal_dvl_ekf_replay.parm`); it does not provide a validated position estimate. The generic mission-item parser supports global/relative/terrain frames, not the ROS queue's local NED frame (`libraries/AP_Mission/AP_Mission.cpp`, `mavlink_int_to_mission_cmd`). A coordinate-origin and depth-frame policy needs explicit validation before any conversion or upload.

The combined virtual Xsens/Bar30/A50 replay has reported both `EKF_CONST_POS_MODE` with attitude/velocity flags and `EKF_UNINITIALIZED` across 15-second runs; neither had an absolute horizontal-position flag. That variability and missing position are direct software evidence for keeping mission commands disabled. The read-only dashboard displays these report flags without claiming to implement `position_ok()`.

Next software-only steps:

1. Establish a replayed, observable navigation source and verify `position_ok()` and estimator origin behavior without real motors. Do not infer position health from fresh `LOCAL_POSITION_NED` alone.
2. Define a mission data contract for each ROS waypoint field: frame/origin, depth sign and reference, yaw, tolerance, hold duration, and speed. Reject unsupported fields rather than silently dropping them.
3. Implement a separate, explicit ArduSub mission transfer client with sequence, acknowledgement, timeout, rollback, and read-back checks. Keep upload distinct from execution and from the read-only dashboard API.
4. Exercise upload/read-back and start/pause/abort/skip behavior in SITL with disarmed and fault cases. Verify mode/arming acknowledgements and neutral/failsafe behavior before surfacing active controls in the familiar dashboard shell.

Physical sensor mounting, actuator wiring, E-stop behavior, and wet mission commissioning remain the final supervised hardware phase. The current card is operational status only.
