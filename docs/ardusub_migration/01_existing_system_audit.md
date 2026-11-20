# Existing system audit

Reference: AUV-IITK/ATAL_controller at `9eadb1a92a9635c706c2f5ec6710167a6ab512e7`, master, clean at inspection. Paths below are relative to that reference. The original checkout is read-only. A git archive in `.atal-baseline/reference` was used for building and tests; it is not a runtime dependency or a deliverable.

## Architecture and complete data flow

`src/auv_control_ros2` is an ament_python package and `src/auv_interfaces_ros2` an ament_cmake/rosidl message package. `README.md`, `setup.py`, `package.xml`, and `.github/workflows/ci.yml` describe Ubuntu 22.04 / ROS 2 Humble. Additional Python imports include NumPy, SciPy, pyserial, PyYAML, optional PyUSB, and Flask. These are not all declared in package.xml. No checked-in systemd units or recorded ROS bags were found in the tracked-file inventory.

All node paths in this audit use `src/auv_control_ros2/auv_control_ros2/` unless qualified.

* Xsens USB/serial → `xsens_mti_driver_node.py:XsensMtiDriver` → `/imu/data` → `attitude_controller_node.py:AttitudeController.imu_callback` → `/auv/control/attitude_output` → `motor_mixer_node.py:MotorMixer` → `/auv/control/thrusters` → `esp32_bridge_node.py:ESP32Bridge` → indexed ASCII USB serial → ESP32 LEDC → eight ESC channels.
* Bar30 → ESP32 I²C → `DEPTH:p,t,d` → bridge `/auv/depth` → attitude controller depth loop → same mixer/output chain.
* A50 TCP JSON → `dvl_driver_node.py` → `/dvl/data`, optional `/dvl/position` → `position_controller_node.py` → `/auv/control/xy_output` → attitude controller. **The position node is commented out in the default `launch/control_system.launch.py`.** Existence of its code does not prove the default launch performs position hold.
* Flask dashboard → `dashboard_bridge_node.py` → `/auv/control/command`, `/auv/arm`, parameter/service calls. `timed_mission.py` supplies mission sequencing; it does not replace the low-level control loops.

## Sensor interfaces, timing and estimation

### Xsens MTi

`xsens_mti_driver_node.py` supports termios at 115200 and PyUSB VID 0x2639, OUT 0x02 / IN 0x83. It sends GoToConfig, SetOutputConfiguration and GoToMeasurement. Requested 100 Hz fields are quaternion 0x2010, rate 0x8020, acceleration 0x4020 and packet counter 0x1020. Xbus short/extended framing and checksum are parsed, including a special preamble-less USB variant. Payloads use big-endian floats. The decoder ignores the packet counter, masks frame/precision low bits, timestamps on ROS receipt, and publishes whenever quaternion is present, even when gyro/acceleration are absent. No acknowledgement validation, continuity tracking or status-word check was found in this driver. These limitations must not be propagated into estimator health claims. The bundled proprietary MT Manager distribution is not needed for the new runtime and will not be copied.

`attitude_controller_node.py:imu_callback` consumes Xsens orientation directly, rotates by inverse mount quaternion, subtracts manually calibrated gyro bias, filters gyro and computes a receipt-time dt. It uses relative level/yaw offsets and configurable pitch/yaw sign inversions. This is not a global underwater position estimator. `position_controller_node.py` uses body velocities, yaw/mount corrections, lever-arm compensation and local position integration/DR. `config/sensor_config.yaml` labels FRD/NED but contains stale Nortek A30 serial and native I²C entries inconsistent with the active A50 TCP/ESP32 implementation. Treat launch and drivers as stronger evidence than that YAML.

### Water Linked A50

`dvl_driver_node.py:_tcp_client_loop` reads newline JSON at 192.168.194.95:16171 with reconnect. `_parse_and_publish_json` accepts a message containing vx and altitude, extracts vx/vy/vz (m/s), fom, altitude, velocity_valid and status. It defaults missing validity to true and missing vector components to zero. The field `time` is retained but treated as a timestamp by the ROS message; the [manufacturer's A50 JSON protocol](https://docs.waterlinked.com/dvl/dvl-json-protocol/) defines it as **milliseconds since the previous velocity report**. The actual `time_of_validity` is a separate Unix-microsecond field that this driver discards. The ROS header is receipt time. DR is published only if x/y are in the velocity message; separate `position_local` reports are not handled here. `position_controller_node.py` rejects invalid velocity/high FOM and has a 0.5 s freshness setting. Full covariance is not retained by `DVL.msg`. Simulation is explicitly opt-in; no invented global position may be used in migration.

### Depth

`firmware/esp32_depth/depth_esp_final.ino`: separate ESP32, SDA22/SCL21, 100 kHz I²C, 115200 USB serial, nominal 20 Hz. MS5837 library supplies mbar and Celsius. Depth = (pressure - surfacePressure)*100/(1000*9.80665), clamped at -1 m. Boot/tare averages ten near-atmospheric samples; at least five are required, otherwise 1013.25 mbar is used. `cal`, `zero`, `tare` cause recalibration; `id` replies IDENT:DEPTH. Initialization retries after failure, but ongoing sensor read validity, sequence and measurement timestamps are absent from telemetry. Host pressure ingestion must use absolute pressure converted to Pa and independently track freshness; do not re-use firmware depth as absolute altitude. No second physical I²C connection is required.

## Control, allocation and output protocol

`attitude_controller_node.py` has quaternion angle error → sqrt-controller desired rates → `ac_pid.py` rate PID, depth/vertical-velocity loops, saturation feedback, master/axis gates and command timeouts. The default launch uses identity IMU mount, 2 s IMU timeout and 0.5 s depth timeout. Geometry/gains are not evidence of measured physical calibration.

`motor_mixer_node.py` separates vertical RPT and horizontal YFL normalization, applies motor reversals, thrust expo/deadband/reverse scale, and normally clamps 1300–1700 around 1500. Disarmed/disabled output is neutral. Attitude and test overrides expire at 0.5 s. The config has four horizontal and four vertical motors. Factors are a permutation of a Vectored-6DOF table, but actual mounting confirmation is still required. `config/thruster_layout.yaml` entries override pin comments: logical motor GPIOs are [14,13,12,27,18,19,4,5], versus MCU indexed GPIOs [27,12,18,19,4,14,5,13], yielding logical-to-MCU IDs [6,8,2,1,3,4,5,7]. M4 and M5 are reversed. Positions appear nominal rather than measured.

`esp32_bridge_node.py` supports shared or separate depth/thruster ports. It sends `id:pwm` comma-separated lines, sorted by MCU ID. Writer checks every 20 ms and refreshes unchanged commands at 300 ms. It can reconnect; shutdown sends `1500`. Bridge allows 1100–1900, wider than the mixer. Its launch watchdog is 1.5 s, not the 0.5 s claimed in CONTRIBUTING.md.

`firmware/esp32_thrusters/thruster_esp_final.ino` configures eight 50 Hz LEDC channels. Neutral is sent at boot; commands become enabled automatically after 1500 ms. `1500`, stop/s are neutral commands. Indexed partial updates, PWM lists and all commands are accepted; parsing uses atoi and updates valid tokens individually. There is no atomic complete-frame requirement, sequence, checksum, explicit arm session, fault latch or re-arm handshake. Timeout is 2500 ms, despite a host comment claiming 2 s. It neutralizes but does not revoke esc_armed. A newer/alternate `firmware/thruster_esp_v5` and legacy firmware also exist; deployed firmware identity is unverified. These observations describe source, not a hardware safety certification.

## Useful functionality and migration ownership

Retain perception validation and guidance (`gate_link_logic.py`, `gate_link_node.py`, `gate_guidance_logic.py`, `gate_guidance_node.py`, `gate_detection_publisher_node.py`), high-level mission logic (`timed_mission.py`), diagnostics and telemetry concepts. `docs/nuc_nano_link.md` documents Nano-to-NUC ROS topics and stale detection behavior. Guidance proposes commands rather than writing motors. `web_dashboard/flight_logger.py` produces CSV, subtitles and summaries; ROS logging and dashboard `/rosout` are present. Dashboard bridges and autotune currently manipulate the old controller and cannot be launched unchanged against the new autopilot. Low-level attitude/position controllers, mixer and actuator bridge must be superseded by ArduSub, with one owner of each hardware port. Do not copy the old control launch into a deployment launcher.

## Build/test baseline and limitations

On Ubuntu 22.04.5, Python 3.10.12 and ROS Humble: building both packages from a disposable git archive succeeded (11.6 s, only disabled-bytecompile warnings). With that overlay sourced, `python3 -m pytest src/auv_control_ros2/test -q -p no:cacheprovider` passed **147 tests in 2.58 s**. Logs reside in ignored `.atal-baseline/build.log` and `tests.log`. No nodes, bench scripts or physical devices were run. Some protocol/failsafe tests reproduce equations instead of invoking production implementations (`test_esp32_protocol.py`, `test_failsafes.py`, `test_verification.py`); passing them does not establish production fault behavior. The root run_tests.sh builds in-place and was deliberately not run in the original.

## Open evidence needs

Confirm deployed firmware variant, actual thruster geometry/polarity, Xsens sensor-to-FRD mounting and earth-frame configuration, DVL mounting/lever arm and recorded valid/invalid packets, timing/latency, depth sensor health behavior, hardware emergency-stop wiring, and desired high-level modes. No claim of physical actuation or estimator validation is made. ROS package metadata declares MIT with AUV-IITK Team attribution; no repository-wide license covering firmware/vendor binaries was found. Preserve provenance and resolve redistribution terms before publishing copied non-package code.
