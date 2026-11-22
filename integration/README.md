# Editable vehicle defaults and software checks

This directory contains provisional values copied from the read-only ROS 2 `ATAL_controller` checkout. They are starting points, not measured installation data. Keep changes in this ArduSub repository; the ROS 2 checkout remains the fallback.

| Change | Edit here | Check |
|---|---|---|
| Xsens mount and serial rate | `vehicle_defaults.json` (`xsens`) and `atal_sensor_defaults.parm` (`AHRS_ORIENTATION`, `SERIAL1_BAUD`); choose the actual serial device when launching | Native virtual-sensor replay, then measured installation check |
| A50 address, mount rotation, lever arm, FOM, and timeout | `vehicle_defaults.json` (`dvl_a50`); the bridge loads this JSON with `--config` | `test_dvl_mavlink_replay.py` and `test_dvl_native_chain.py` |
| Bar30 serial rate and water density | `vehicle_defaults.json` (`bar30`) and `atal_sensor_defaults.parm` (`SERIAL2_BAUD`, `BARO_SPEC_GRAV`); choose the actual serial device when launching | `test_depth_native_replay.py`, then surface-zero/calibration check |
| Thruster factors, numbering, polarity, GPIOs, PWM limits | `vehicle_defaults.json` (`actuators`) and matching `atal_motor_defaults.parm` | `check_layout.py` and `simulate_allocation.py` |

The JSON is an editable bridge/audit input, not an automatic ArduSub parameter loader. Keep matching ArduSub `.parm` values in sync. The Xsens and Bar30 `offset_m_frd` fields are recorded placeholders; no native correction currently consumes them. The A50 bridge does consume its mount rotation and lever arm from JSON. `atal_dvl_ekf_replay.parm` selects MAVLink VisualOdom and EKF3 external horizontal velocity for **virtual-sensor tests only**; it supplies no global position or yaw. The measured Xsens and A50 rotations/offsets remain identity/zero placeholders. `actuators.enabled` remains `false`. The ordinary generic Linux build uses `Empty::RCOutput`; the opt-in localhost observer below captures ArduSub output for software checks.

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

`dashboard_api.py` is a local HTTP API backed by read-only ArduSub MAVLink messages. Start it with `python3 integration/dashboard_api.py --mavlink udpin:127.0.0.1:14650` and open `http://127.0.0.1:8088/` for the first ROS-style operator dashboard port. The earlier compact instrument page is at `/diagnostics`. `/api/state`, `/api/health`, and the `/api/stream` Server-Sent Events feed expose telemetry. Route a copy of ArduSub telemetry to UDP port 14650 in the software environment. The control, mission, and camera functions from the ROS 2 dashboard have **not** been ported; their navigation labels are inert. This dashboard and API send no MAVLink commands. `armed_known` and freshness flags distinguish a disconnected telemetry link from a known disarmed state. `LOCAL_POSITION_NED` is used only as available ArduSub telemetry; real navigation validity remains to be established. Depth and velocity clear when position messages become stale. Velocity is converted from NED into body axes only when attitude is fresh. `SERVO_OUTPUT_RAW` displays the first eight channels provisionally; physical motor-channel mapping must be checked later.

`python3 integration/test_dashboard_native_replay.py` proves a separate native `SERIAL3_PROTOCOL=2` MAVLink stream reaches the running dashboard HTTP service while `SERIAL0` remains available to another consumer. The replay sets `SR3_EXTRA1`, `SR3_EXTRA2`, and `SR3_RAW_SENS` in a temporary parameter file and uses virtual Xsens/depth inputs. It checks `/api/state`, `/api/stream`, and the operator page through HTTP. For a real deployment, put the matching stream configuration in the deployment parameters, choose an unoccupied serial link, and verify source system ID and rate. The replay does not establish full dashboard command behavior or physical vehicle readiness.
The same native replay starts a CSV recording and verifies a row contains fresh ArduSub link, IMU, and pressure data before stopping it. All files are temporary; it still sends no vehicle commands.

`python3 integration/test_dashboard_api_loopback.py` starts the actual local HTTP service, sends synthetic MAVLink from autopilot system/component 1/1 over UDP, and verifies `/api/state` receives it. It opens only loopback sockets.

The dashboard's Telemetry Recording card starts and stops a local CSV at 10 Hz. `GET /api/recording` reports its status; `POST /api/recording/start` and `/api/recording/stop` operate only on local files and require `X-ATAL-Local-Action: recording`. `GET /api/recording/download` downloads the current or last CSV. By default the service saves under ignored `integration/logs/dashboard/`; pass `--recording-dir` to change the destination. Rows contain UTC time, normalized telemetry, freshness flags and eight reported servo channels. Unknown/stale values remain blank rather than becoming zero. Recording continues if the page closes while the service runs, and stops when the service exits. The CSV is telemetry only, with no ROS controller setpoints or video synchronization. Run `python3 -m unittest discover -s integration -p 'test_dashboard_recorder.py' -v` for isolated CSV and HTTP checks.

## Native motor output observer

Build the optional Linux profile and run the virtual sensor replay:

```bash
./waf configure --board=linux --extra-hwdef integration/atal_loopback_output.hwdef
./waf sub -j4
python3 integration/test_rcoutput_native_loopback.py
python3 integration/test_rcoutput_native_loopback.py --active
```

`RCOutput_ATAL_Loopback` sends the raw 16 `SERVO` channel values and ArduSub soft-armed state as `ATAL_RCOUT,1,armed,ch1,...,ch16` UDP datagrams to `127.0.0.1:14660`. The first test verifies neutral, disarmed motor channels with virtual Xsens and Bar30 data. The `--active` run sets `ARMING_CHECK=0` only in its temporary parameter file, sends a MAVLink arm and forward command, checks the native mixer response, then verifies neutral output after disarm. This test profile is software only and cannot demonstrate safe vehicle arming. `atal_motor_defaults.parm` explicitly assigns `SERVO1_FUNCTION=33` through `SERVO8_FUNCTION=40`; `check_layout.py` checks these assignments. The datagrams are an observation format, not ESP32 actuator commands. There is no listener that drives ESCs, and the physical actuator path has not been validated.

`rcoutput_mapping.py` validates complete observation datagrams and derives the AP motor to ESP32 ID permutation from `vehicle_defaults.json` and the pinned Vectored-6DOF frame. `test_rcoutput_mapping.py` checks a forward-output example against the existing `A1` frame encoder; the native active test uses the same mapping. This mapping is provisional. No code forwards these observations to the ESP32.

For a software-only guard check, run `python3 integration/test_rcoutput_native_loopback.py --active --guard-sim` after building the optional Linux profile above. This takes the actual native mixer response, maps it into provisional ESP32 order, encodes `A1` commands, and feeds them to `actuator_guard_sim.cpp`, a stdin/stdout harness around `ActuatorGuard`. It checks arm, PWM, disarm, and stale-command neutralization. `python3 -m unittest discover -s integration -p 'test_actuator_guard_sim.py' -v` additionally covers malformed-frame fault and recovery without starting ArduSub. The simulator has no serial, network, GPIO, or ESC output. Its fixed test session is not a deployed session-negotiation design.
