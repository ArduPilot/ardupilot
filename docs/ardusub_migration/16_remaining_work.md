# Remaining work against the original migration brief

Updated 2026-11-25 against M1–M9 and current verified software/simulation evidence (`integration/run_validation_suite.py`, `docs/ardusub_migration/08_testing_and_validation.md`). The ROS 2 checkout (`ATAL_controller`) remains unmodified as the deployed fallback; no changes have been pushed or deployed to vehicle hardware.

## Completed software and simulation gates

- **Native build & sensor drivers (M1–M2):** Ordinary Linux `build/linux/bin/ardusub` builds cleanly; Xsens MTData2 IMU + optional `0xC020` mag (`AP_ExternalAHRS_Xsens`) and ESP32 Bar30 (`AP_Baro_ATAL_Depth`) pass parser unit tests (`4/4`) and PTY native replays.
- **DVL A50 & EKF3 relative position (M3–M5 software path):** `dvl_bridge.py` (`VISION_POSITION_DELTA` + `atal_dvl_status`) drives native EKF3 into `AID_RELATIVE` (`303` / `EKF_POS_HORIZ_REL`), tracks velocity, and recovers across 5 s lock loss (`[303] -> [167, 303]`).
- **Actuator mapping, guard & closed-loop 6-DoF control (M6–M7 software path):** Layout check (`rank 6`), source-derived allocation comparison, `ActuatorGuard` C++ & `esp32_guarded` host parser tests, `test_rcoutput_native_loopback.py --active --guard-sim --serial-pty`, and `test_closed_loop_dynamics.py` (`STABILIZE`, `ALT_HOLD`, `GUIDED` waypoint, neutral disarm) all pass. Checked-in `actuators.enabled` stays `false`.
- **Control, missions, companion & classic dashboard (M8–M9 software path):** `ArduSubCommander`, `MissionClient`, `GuidedMissionRunner`, `TimedMissionRunner`, `FlightPlanRunner`, `GateGuidanceBridge`, `RuntimeSupervisor`, and `dashboard_api.py --enable-control` (`/classic`, `/debug`, `57/60` reference API paths, `WP_SPD`/`PSC_D_*`/`PSC_NE_*` ArduSub 4.7 params) pass unit (`42/42`) and `test_native_end_to_end.py` native checks.
- **Reproducible suite:** `python3 integration/run_validation_suite.py` (ordinary build) and `--with-loopback` (loopback `6-DoF` + guard PTY + ordinary rebuild) provide single-command verification.

## Remaining hardware and vehicle-dependent qualification

1. **ESP32 Arduino toolchain & transport fault tests (M7):** Compile `esp32_guarded/esp32_guarded.ino` with the target Arduino/ESP32 toolchain (host stub tests pass, Arduino build not yet run), then qualify USB disconnect, watchdog recovery, and fault-latch behavior on hardware.
2. **Real sensor logs & physical mounting geometry:** Measure Xsens/A50 orientations & lever arms, Bar30 offset & water density, and thruster directions/GPIO wiring; replace placeholders in `integration/vehicle_defaults.json` and `.parm` files. Verify real Xsens startup, Bar30 surface zeroing, real A50 bottom-lock rate/loss/recovery, and EKF3 readiness with propulsion isolated.
3. **Physical actuator & E-stop commissioning:** Under human supervision, verify independent actuator power isolation / E-stop, motor spin directions (`MOT_1..8_DIRECTION`), ESC neutral bounds (`1300..1700` us), and re-test restrained dry and wet control tuning before retiring `ATAL_controller`.
