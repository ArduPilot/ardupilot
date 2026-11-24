# Migration progress

## 2026-11-20 through 2026-11-24

- Established independent ArduPilot checkout at `4ca1bc74fc088242931054cac963cdfd91589b51` on local branch `feature/atal-ardusub-migration`; audited read-only ROS 2 reference `AUV-IITK/ATAL_controller@9eadb1a92a9635c706c2f5ec6710167a6ab512e7`.
- Added native Xsens MTi-300 `AP_ExternalAHRS_Xsens` (`EAHRS_TYPE=12`) and ESP32 Bar30 `AP_Baro_ATAL_Depth` (`SERIAL2_PROTOCOL=51`) serial backends with strict parsers and virtual PTY native replays.
- Built Vectored-6DOF motor mapping (`integration/check_layout.py`, `integration/atal_motor_defaults.parm`), `RCOutput_ATAL_Loopback`, `ActuatorAdapter`, `actuator_serial_bridge.py`, and guarded ESP32 parser/sketch with `actuators.enabled=false`.
- Built initial read-only `dashboard_api.py`, telemetry mapper, CSV recorder, proposal-only ROS gate companion (`atal_companion.launch.py`), and preserved exact ROS UI templates under `integration/reference_dashboard/`.

## 2026-11-25 software integration and verification

- **Optional Xsens magnetometer (`0xC020`):** Updated `AP_ExternalAHRS_Xsens_Parser` and `AP_ExternalAHRS_Xsens` so MTData2 `0xC020` magnetometer packets are decoded when present while remaining optional; all 4/4 `test_xsens_parser` gtest checks pass.
- **EKF3 relative-position readiness with DVL A50:** Root-caused `EKF_UNINITIALIZED` (`1024`) to `VISION_SPEED_ESTIMATE` not entering `AID_RELATIVE` (`readyToUseBodyOdm()`). Updated `dvl_velocity.py` and `dvl_bridge.py` to default to `--mode body_delta` (`VISION_POSITION_DELTA`) plus `--status-udp` health events. `test_dvl_native_chain.py` verified transition to `AID_RELATIVE` (`303`), velocity tracking, and recovery from 5 s lock loss (`[303]` -> `[167, 303]`).
- **ArduSub control, mission, runtime & companion stack:** Added `ardusub_control.py`, `mission_client.py`, `timed_mission.py`, `flight_plan_runner.py`, `gate_bridge.py`, `atal_runtime.py`, and aligned parameter names/units to ArduSub 4.7 (`WP_SPD` m/s, `WP_SPD_UP`, `WP_SPD_DN`, `WP_RADIUS_M`, `PSC_D_*`, `PSC_NE_*`).
- **Classic ROS UI wiring & E2E verification:** Wired `dashboard_api.py --enable-control`, `classic_dashboard.py`, and `classic_adapter.js` so `/classic` and `/debug` connect to ArduSub commands, PID/profile tuning, mission upload/readback, Guided waypoint queue, Timed Missions, Flight Plan, and Gate Bridge while keeping read-only mode active by default (`57/60` literal reference paths shared). `test_native_end_to_end.py` passed live EKF readiness, mode switches (`GUIDED`, `ALT_HOLD`, `STABILIZE`), `WP_SPD` param roundtrip, mission upload/readback (4 items), and kill/disarm on `build/linux/bin/ardusub`.
- **Closed-loop 6-DoF physics & release suite:** Added `vehicle_dynamics_sim.py`, `test_closed_loop_dynamics.py`, and `run_validation_suite.py`. Verified `STABILIZE` attitude recovery, `ALT_HOLD` dive + heave restoration, `GUIDED` waypoint arrival, neutral disarm, and `test_rcoutput_native_loopback.py --active --guard-sim --serial-pty`, then restored the ordinary `--board=linux` build and passed all checks in `run_validation_suite.py`.

## Remaining work

Software and virtual/loopback verification gates are complete; physical vehicle qualification (real sensor logs/mounting offsets, Arduino ESP32 compilation and GPIO/ESC/E-stop qualification, and restrained/wet tests) remains before retiring the ROS 2 fallback. See `16_remaining_work.md`.
