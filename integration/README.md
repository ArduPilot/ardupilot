# Editable vehicle defaults and software checks

This directory contains provisional values copied from the read-only ROS 2 `ATAL_controller` checkout. They are starting points, not measured installation data. Keep changes in this ArduSub repository; the ROS 2 checkout remains the fallback.

| Change | Edit here | Check |
|---|---|---|
| Xsens mount and serial rate | `vehicle_defaults.json` (`xsens`) and `atal_sensor_defaults.parm` (`AHRS_ORIENTATION`, `SERIAL1_BAUD`); choose the actual serial device when launching | Native virtual-sensor replay, then measured installation check |
| A50 address, mount rotation, lever arm, FOM, and timeout | `vehicle_defaults.json` (`dvl_a50`); the bridge loads this JSON with `--config` | `test_dvl_mavlink_replay.py` and `test_dvl_native_chain.py` |
| Bar30 serial rate and water density | `vehicle_defaults.json` (`bar30`) and `atal_sensor_defaults.parm` (`SERIAL2_BAUD`, `BARO_SPEC_GRAV`); choose the actual serial device when launching | `test_depth_native_replay.py`, then surface-zero/calibration check |
| Thruster factors, numbering, polarity, GPIOs, PWM limits | `vehicle_defaults.json` (`actuators`) and matching `atal_motor_defaults.parm` | `check_layout.py` and `simulate_allocation.py` |

The JSON is an editable bridge/audit input, not an automatic ArduSub parameter loader. Keep matching ArduSub `.parm` values in sync. The Xsens and Bar30 `offset_m_frd` fields are recorded placeholders; no native correction currently consumes them. The A50 bridge does consume its mount rotation and lever arm from JSON. `atal_dvl_ekf_replay.parm` selects MAVLink VisualOdom and EKF3 external horizontal velocity for **virtual-sensor tests only**; it supplies no global position or yaw. The measured Xsens and A50 rotations/offsets remain identity/zero placeholders. `actuators.enabled` remains `false`, and the generic Linux build uses `Empty::RCOutput`.

After edits, run from the repository root:

```bash
python3 integration/check_layout.py
python3 integration/simulate_allocation.py
python3 -m unittest discover -s integration -p 'test_*.py' -v
python3 integration/test_dvl_mavlink_replay.py
python3 integration/test_depth_native_replay.py
python3 integration/test_dvl_native_ingest.py
python3 integration/test_dvl_native_chain.py
```

The native replays require `build/linux/bin/ardusub` from `./waf configure --board=linux && ./waf sub -j4`. They create temporary loopback TCP/UDP links and pseudo-terminals; they do not open physical sensor or actuator ports. Read `docs/ardusub_migration/08_testing_and_validation.md` for each check's scope and outstanding hardware evidence.

## Provisional actuator protocol

`actuator_guard.h` is a platform-independent ESP32 command validator and state machine. `actuator_frames.py` encodes the matching host frames. Neither file is wired to ArduSub, serial, GPIO, or deployed firmware yet. Frames are complete ASCII lines without CR/LF: `A1,COMMAND,SESSION,SEQUENCE[,PWM1,...,PWM8],CRC`. `SESSION` is eight hex digits; `SEQUENCE` is a strictly increasing decimal uint32; PWM values are 1300–1700 µs in the provisional ROS motor order; `CRC` is four hex digits of CRC-16/CCITT-FALSE over the preceding bytes. The proposed MCU firmware must generate and expose a fresh session nonce on each boot, read complete bounded lines, call `tick()` even during USB silence, and apply `outputs()` only after validating the entire frame. Faults latch neutral until an explicit `RESET` followed by `ARM`. A separate physical actuator-power isolation mechanism is still required.

Run `g++ -std=c++11 -Wall -Wextra -Werror -I integration integration/test_actuator_guard.cpp -o .atal-baseline/test_actuator_guard && .atal-baseline/test_actuator_guard` and `python3 -m unittest discover -s integration -p 'test_actuator_frames.py' -v` for software-only checks. These do not authorize attaching an ESC or flashing an ESP32.

## Dashboard telemetry adapter

`dashboard_api.py` is a read-only local HTTP API backed by ArduSub MAVLink messages. Start it with `python3 integration/dashboard_api.py --mavlink udpin:127.0.0.1:14650` and read `http://127.0.0.1:8088/api/state` or `/api/health`. Route a copy of ArduSub telemetry to UDP port 14650 in the software environment. This is the initial data adapter for the existing ROS dashboard's `/api/state` schema; the existing dashboard UI and its control, mission, and camera functions have **not** been ported. It does not send MAVLink commands. `armed_known` and freshness flags distinguish a disconnected telemetry link from a known disarmed state. `LOCAL_POSITION_NED` is used only as available ArduSub telemetry; real navigation validity remains to be established. Velocity is converted from NED into body axes only when attitude is fresh. `SERVO_OUTPUT_RAW` displays the first eight channels provisionally; physical motor-channel mapping must be checked later.

`python3 integration/test_dashboard_native_replay.py` proves a separate native `SERIAL3_PROTOCOL=2` MAVLink stream can feed the dashboard adapter while `SERIAL0` remains available to another consumer. The replay sets `SR3_EXTRA1`, `SR3_EXTRA2`, and `SR3_RAW_SENS` in a temporary parameter file and uses virtual Xsens/depth inputs. For a real deployment, put the matching stream configuration in the deployment parameters, choose an unoccupied serial link, and verify source system ID and rate. The replay does not establish the full dashboard UI or physical vehicle readiness.
