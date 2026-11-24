# Editable vehicle defaults and software checks

`atal_companion.launch.py` starts only the ROS 2 gate link and proposal-only guidance nodes. Source ROS Humble and a build of `auv_interfaces_ros2` plus `auv_control_ros2` from the read-only reference, then run `ros2 launch integration/atal_companion.launch.py`. It does not start the reference `control_system.launch.py`, open sensor/actuator ports, or send MAVLink commands. Nano gate detections are an independent input; no ArduSub guidance-command bridge exists yet.

To show those ROS proposals on the read-only dashboard, start `dashboard_api.py` from the same sourced shell with `--gate-ros`. Without it, the Gate Guidance card remains unavailable. The optional subscriber reads only the gate link and guidance topics; it sends no MAVLink or ROS commands. Proposal axes clear after 0.5 s without a fresh valid linked message. A disposable ROS package build and live companion/API smoke test confirmed the card's backing API receives `WAIT_FOR_LINK` while no Nano detections exist.
A synthetic Nano `GateDetection` replay also reached the companion and dashboard as a valid `ALIGN` proposal with bounded lateral correction. The dashboard still does not act on it.
The local CSV includes gate link health, state, validity, and proposal axes; expired or invalid axes are blank.

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

`esp32_guarded/esp32_guarded.ino` now runs the platform-independent `ActuatorGuard` on an ESP32 serial endpoint. It has **no GPIO/LEDC/ESC output code**; flashing it cannot drive thrusters. The guard accepts complete ASCII lines: `A1,COMMAND,SESSION,SEQUENCE[,PWM1,...,PWM8],CRC`. `SESSION` is eight hex digits; `SEQUENCE` is a strictly increasing decimal uint32; PWM values are 1300–1700 µs in provisional ESP32 motor-ID order; `CRC` is four hex digits of CRC-16/CCITT-FALSE over the preceding bytes. The host sends `A1,HELLO,CHALLENGE` with an eight-digit hex challenge. That forces neutral, creates a new session and returns `A1,SESSION,SESSION,CHALLENGE`; an old or unrelated response cannot start the stream. Malformed frames and a 250 ms armed timeout latch neutral and emit `A1,FAULT`; recovery requires `RESET` then `ARM`, or a new HELLO session. Physical output wiring and actuator-power isolation remain hardware-stage work.

Run `g++ -std=c++11 -Wall -Wextra -Werror -I integration integration/test_actuator_guard.cpp -o /tmp/test_actuator_guard && /tmp/test_actuator_guard`, `g++ -std=c++11 -Wall -Wextra -Werror integration/test_esp32_guarded.cpp -o /tmp/test_esp32_guarded && /tmp/test_esp32_guarded`, and `python3 -m unittest discover -s integration -p 'test_actuator*.py' -v` for software-only checks. The C++ sketch test uses a fake serial port; no Arduino toolchain or physical ESP32 has been tested yet.

## Dashboard telemetry adapter

`dashboard_api.py` is a local HTTP API backed by read-only ArduSub MAVLink messages. Start it with `python3 integration/dashboard_api.py --mavlink udpin:127.0.0.1:14650` and open `http://127.0.0.1:8088/` for the first ROS-style operator dashboard port. The earlier compact instrument page is at `/diagnostics`. `/api/state`, `/api/health`, and the `/api/stream` Server-Sent Events feed expose telemetry. Route a copy of ArduSub telemetry to UDP port 14650 in the software environment. The ROS 2 dashboard control, mission-command, and GoPro transport functions have **not** been ported; their navigation labels are inert. The optional browser camera preview is described below. This dashboard and API send no MAVLink commands. `armed_known` and freshness flags distinguish a disconnected telemetry link from a known disarmed state. `LOCAL_POSITION_NED` is used only as available ArduSub telemetry; real navigation validity remains to be established. Attitude, pressure, position, body velocity, and output values clear when their source messages become stale. Velocity is converted from NED into body axes only when attitude is fresh. `SERVO_OUTPUT_RAW` displays the first eight channels provisionally; physical motor-channel mapping must be checked later.

`python3 integration/test_dashboard_native_replay.py` proves a separate native `SERIAL3_PROTOCOL=2` MAVLink stream reaches the running dashboard HTTP service while `SERIAL0` remains available to another consumer. The replay sets `SR3_EXTRA1`, `SR3_EXTRA2`, and `SR3_RAW_SENS` in a temporary parameter file and uses virtual Xsens/depth inputs. It checks `/api/state`, `/api/stream`, and the operator page through HTTP. For a real deployment, put the matching stream configuration in the deployment parameters, choose an unoccupied serial link, and verify source system ID and rate. The replay does not establish full dashboard command behavior or physical vehicle readiness.
The same native replay starts a CSV recording and verifies a row contains fresh ArduSub link, IMU, and pressure data before stopping it. All files are temporary; it still sends no vehicle commands.

`python3 integration/test_dashboard_api_loopback.py` starts the actual local HTTP service, sends synthetic MAVLink from autopilot system/component 1/1 over UDP, and verifies `/api/state` receives it. It opens only loopback sockets.

The Mission Status card displays only fresh ArduSub `MISSION_CURRENT` telemetry. Set the dashboard link's extended-status stream rate (for example `SR3_EXT_STAT=2` for the replay's SERIAL3 link) to receive item, total, state, and mode. This display cannot start a mission or establish navigation validity. The flight-plan and mission-builder command labels remain inert.

The dashboard's Telemetry Recording card starts and stops a local CSV at 10 Hz. `GET /api/recording` reports its status; `POST /api/recording/start` and `/api/recording/stop` operate only on local files and require `X-ATAL-Local-Action: recording`. `GET /api/recording/download` downloads the current or last CSV. By default the service saves under ignored `integration/logs/dashboard/`; pass `--recording-dir` to change the destination. Rows contain UTC time, normalized telemetry including mission status, freshness flags and eight reported servo channels. Values with unknown or expired source data are blank rather than zero, including arming state after heartbeat loss. Recording continues if the page closes while the service runs, and stops when the service exits. The CSV is telemetry only, with no ROS controller setpoints or video synchronization. Run `python3 -m unittest discover -s integration -p 'test_dashboard_recorder.py' -v` for isolated CSV and HTTP checks.

## Native motor output observer

Build the optional Linux profile and run the virtual sensor replay:

```bash
./waf configure --board=linux --extra-hwdef integration/atal_loopback_output.hwdef
./waf sub -j4
python3 integration/test_rcoutput_native_loopback.py
python3 integration/test_rcoutput_native_loopback.py --active
python3 integration/test_rcoutput_native_loopback.py --active --guard-sim --serial-pty
```

`RCOutput_ATAL_Loopback` sends the raw 16 `SERVO` channel values and ArduSub soft-armed state as `ATAL_RCOUT,1,armed,ch1,...,ch16` UDP datagrams to `127.0.0.1:14660`. The first test verifies neutral, disarmed motor channels with virtual Xsens and Bar30 data. The `--active` run sets `ARMING_CHECK=0` only in its temporary parameter file, sends a MAVLink arm and forward command, checks the native mixer response, then verifies neutral output after disarm. This test profile is software only and cannot demonstrate safe vehicle arming. `atal_motor_defaults.parm` explicitly assigns `SERVO1_FUNCTION=33` through `SERVO8_FUNCTION=40`; `check_layout.py` checks these assignments. The datagrams are an observation format, not ESP32 actuator commands. There is no listener that drives ESCs, and the physical actuator path has not been validated.

`rcoutput_mapping.py` validates complete observation datagrams and derives the AP motor to ESP32 ID permutation from `vehicle_defaults.json` and the pinned Vectored-6DOF frame. `test_rcoutput_mapping.py` checks a forward-output example against the existing `A1` frame encoder; the native active test uses the same mapping. This mapping is provisional; the serial bridge is disabled by default.

For a software-only guard check, run `python3 integration/test_rcoutput_native_loopback.py --active --guard-sim` after building the optional Linux profile above. This takes the actual native mixer response, maps it into provisional ESP32 order, encodes `A1` commands, and feeds them to `actuator_guard_sim.cpp`, a stdin/stdout harness around `ActuatorGuard`. It checks arm, mapped PWM, and disarm against real native output. `python3 -m unittest discover -s integration -p 'test_actuator_guard_sim.py' -v` additionally covers malformed-frame fault and recovery without starting ArduSub. The simulator has no serial, network, GPIO, or ESC output; it uses a fixed test session, while the serial bridge uses the HELLO exchange.

`python3 integration/actuator_adapter.py` listens to optional localhost observation port `14660`, validates and maps eight native motor channels, and writes CRC-checked `A1` ARM/PWM/DISARM lines to stdout. It requires a disarmed observation before arming; invalid data or a 200 ms observation gap disarms the stream and requires another disarmed observation before resuming. The native `--active --guard-sim` replay passes actual mixer observations through this adapter and into the isolated C++ guard.

`actuator_serial_bridge.py` is the opt-in host transport for that same adapter. It opens an explicitly named serial device at 115200 baud, performs the HELLO challenge/session exchange, consumes localhost native observations, and writes A1 frames. It limits serial PWM traffic to 50 Hz using the newest observation; DISARM is immediate. It refuses to open the port unless both `--enable-hardware` and `actuators.enabled=true` in the chosen JSON config are set; the checked-in config remains `false`. A pseudo-terminal test covers the handshake, mapped ARM/PWM frames, and shutdown DISARM without hardware. The bridge exits on an ESP32 fault or changed session. Do not enable this path on the vehicle until the ESP32 firmware, motor order, GPIO mapping, ESC behavior, independent power isolation, and emergency stop have been verified on hardware. The staged sketch cannot actuate even if the bridge is enabled.

The native `--active --guard-sim --serial-pty` replay also sends live ArduSub mixer observations through the actual serial bridge into a fake ESP32 port, checks mapped non-neutral PWM and disarm, and replays those exact serial frames through the C++ guard. It uses a temporary enabled config and pseudo-terminal only; the checked-in actuator switch remains off.

The operator dashboard includes an optional camera preview. Enter a browser-reachable HTTP image or MJPEG URL and select Connect; the URL is saved in that browser for later editing, but the feed does not reconnect automatically. Disconnect stops loading it. The camera server and GoPro transport are not launched by this dashboard; an RTSP camera needs a separate web-stream bridge. Test with any local HTTP image before connecting real hardware.

The Power card reads ArduSub `SYS_STATUS` voltage (mV), current (cA), and reported remaining percentage. It clears values when the message or vehicle link expires, and treats MAVLink unknown sentinels as unavailable. `battery_alive` and these normalized values are also exposed through the read-only API and CSV recorder. This is reported telemetry, not a calibrated energy estimate.
