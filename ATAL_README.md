# ATAL ArduSub migration

This is an independent ArduPilot checkout for migrating the ATAL AUV from the read-only ROS 2 `ATAL_controller` reference (`9eadb1a92a9635c706c2f5ec6710167a6ab512e7`). The upstream base is pinned at `4ca1bc74fc088242931054cac963cdfd91589b51`; ATAL work is on local branch `feature/atal-ardusub-migration`. Nothing in this repository has been pushed or deployed, and `ATAL_controller` remains unmodified.

## Current state

Software/simulation gates are complete and verified against the native Linux `ardusub` binary, while physical vehicle commissioning remains intentionally locked out:

- **Native firmware & sensors:** Linux `ardusub` builds cleanly (`./waf configure --board=linux && ./waf sub -j4`). `AP_ExternalAHRS_Xsens` (`0x36` MTData2 IMU + optional `0xC020` mag) and `AP_Baro_ATAL_Depth` (`SERIAL2_PROTOCOL=51`) run over virtual serial PTYs.
- **DVL & EKF3 readiness:** `integration/dvl_bridge.py` defaults to `VISION_POSITION_DELTA` (`writeBodyFrameOdom`), bringing EKF3 cleanly into `AID_RELATIVE` (`EKF_POS_HORIZ_REL` / `303`) and recovering from 5 s bottom-lock loss (`[303] -> [167, 303]`).
- **Control, missions & companion:** `ardusub_control.py`, `mission_client.py`, `timed_mission.py`, `flight_plan_runner.py`, `gate_bridge.py`, and `atal_runtime.py` implement ArduSub mode/arm/manual-deadman/parameter (`WP_SPD`, `PSC_D_*`, `PSC_NE_*`), MAVLink mission upload/readback, Guided waypoint runner, 20 Hz Timed Missions, Flight Plan sequencer/CSV/SRT logging, bounded Gate Guidance bridge, and serial-ownership runtime checks.
- **Dashboard:** `integration/dashboard_api.py` is read-only by default (`/` and `/classic`), and with `--enable-control` wires `/classic` (preserved ROS UI shell) and `/debug` to ArduSub MAVLink control, mission, parameter, recording, and diagnostics routes (`check_dashboard_parity.py`: 57/60 literal reference routes wired; `/api/flash_firmware` and `/api/flight_plan/arm_untethered` stay disabled with HTTP 403).
- **Actuator safety boundary:** `integration/vehicle_defaults.json` keeps `actuators.enabled: false`; the ordinary build uses `Empty::RCOutput`. Hardware GPIO/ESC output, physical ESP32 flashing, real sensor mounting calibration, and wet testing remain pending supervised commissioning.

Start with [editable defaults](integration/README.md), [migration progress](docs/ardusub_migration/10_migration_progress.md), and [remaining work](docs/ardusub_migration/16_remaining_work.md).

## Reproducible validation suite

After building the ordinary Linux binary (`./waf configure --board=linux && ./waf sub -j4`):

```bash
python3 integration/run_validation_suite.py
```

To also run the opt-in `atal_loopback_output.hwdef` closed-loop 6-DoF dynamics (`STABILIZE`, `ALT_HOLD`, `GUIDED`) and guarded serial PTY tests before automatically restoring the ordinary build:

```bash
python3 integration/run_validation_suite.py --with-loopback
```

For full scope and limits, see [testing and validation](docs/ardusub_migration/08_testing_and_validation.md).
