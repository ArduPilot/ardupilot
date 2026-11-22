# ROS 2 and dashboard coexistence

This is the integration boundary for the independent ArduSub repository. The read-only ROS 2 reference checkout remains unchanged. Its `control_system.launch.py` starts `dvl_driver`, `xsens_mti_driver`, `attitude_controller`, `motor_mixer`, and `esp32_bridge`; starting that launch alongside ArduSub would create competing sensor and motor ownership. It is not an ArduSub companion launch.

| Reference component | ArduSub migration role |
|---|---|
| `xsens_mti_driver` | Superseded for low-level estimation by native Xsens backend; retain only for offline comparison/replay. Do not open the same serial device twice. |
| `dvl_driver` | Superseded for EKF input by `integration/dvl_bridge.py`; retain ROS DVL data only if a high-level consumer needs it, from a single upstream stream/fan-out. |
| `attitude_controller`, `position_controller`, `motor_mixer`, `esp32_bridge`, motor test and autotune output nodes | Excluded from ArduSub runtime. ArduSub owns estimation, control, allocation, and eventual ESP32 output. |
| Gate detection, gate link, and proposal-only gate guidance | Candidates for a separate ROS companion launch. `gate_guidance.launch.py` is proposal-only and publishes `/auv/mission/gate_guidance`; any future bridge must check command validity, freshness, mode and bounds before sending a high-level MAVLink command. |
| Existing Flask dashboard and `dashboard_bridge_node` | The old bridge publishes `/auv/arm`, `/auv/control/command`, and motor-test topics; it must not run unchanged with ArduSub. `integration/dashboard_api.py` is the initial read-only MAVLink `/api/state` backend. Port the UI and replace each control endpoint with an ArduSub-specific, gated high-level action only after its behavior is defined and tested. |

The current dashboard API reads ArduSub heartbeat, attitude, pressure, local position and output telemetry. Its only POST routes start and stop a local CSV recording; it has no arm, test, mission, camera or parameter routes. The operator page now serves the first ROS-style shell, while control-dependent sections remain unavailable. `armed_known=false` on link loss prevents treating missing telemetry as a confirmed disarm. `SERVO_OUTPUT_RAW` is display telemetry; it is not the chosen authoritative actuator transport. A future native Linux `RCOutput` backend should carry the final atomic motor frame after allocation, with separate ESP32 validation. The generic Linux build still uses `Empty::RCOutput`.

Before enabling any companion command path, establish one source of truth for ArduSub mode, arm state, and failsafes; map the old dashboard's actions to supported ArduSub modes/parameters; reject legacy direct PWM and duplicate controller topics; and prove the operator UI reports stale/unknown states correctly. Real camera and mission workflows require their own tests. Hardware-dependent calibration and actuator deployment remain separate final checks.
