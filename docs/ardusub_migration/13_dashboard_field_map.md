# ROS dashboard field map for the ArduSub port

This inventory comes from the reference `web_dashboard/templates/index.html` `updateUI(s)` in `ATAL_controller@9eadb1a92a9635c706c2f5ec6710167a6ab512e7`. The new `operator_dashboard.html` deliberately uses only the fields that `DashboardTelemetry` can back with real ArduSub MAVLink messages. Do not fill missing fields with zero or copy the ROS control backend to make a panel appear functional.

| Reference fields | Current source or status | Operator presentation |
|---|---|---|
| `roll`, `pitch`, `yaw`, `gyro`, `imu_hz`, `imu_alive` | `ATTITUDE` and freshness in `DashboardTelemetry` | Orientation and readouts appear only while IMU data is fresh |
| `armed` | `HEARTBEAT`; use `armed_known` as well | Unknown, armed, or disarmed; a missing heartbeat never implies known disarmed |
| `pwm` | `SERVO_OUTPUT_RAW` channels 1–8; channel-to-motor wiring provisional | Show reported µs values only while output data is fresh |
| `pos_n`, `pos_e`, `depth_m`, `vel_fwd`, `vel_lat`, `vel_z` | `LOCAL_POSITION_NED`; body velocity also needs fresh attitude | Show while fresh; freshness alone does not establish valid navigation |
| Water pressure and temperature | `SCALED_PRESSURE` (`pressure_mbar`, `water_temp` in new API) | Show only while pressure data is fresh; these are not the ROS `depth` controller value |
| `dvl_locked`, `dvl_hz`, `dvl_altitude`, `dvl_fom`, `dvl_simulating` | Not available from the current ArduSub telemetry stream; new API sets `dvl_locked: null` | Do not show a healthy or failed DVL state until a companion/A50 source publishes it |
| `target_roll`, `target_pitch`, `target_yaw`, `target_depth`, `target_pos_n`, `target_pos_e` | ROS controller targets; no ArduSub dashboard adapter yet | Do not render target markers or setpoint controls |
| `ctrl_mode`, `ctrl_output`, `depth_output`, `fwd_effort`, `lat_effort`, `pid_debug` | ROS controller internals; direct equivalent needs a defined ArduSub message/log contract | PID and control panels remain unavailable |
| `motor_enabled`, `motor_directions`, `motor_weights`, `thrust`, `thruster_hz` | ROS/ESP32 command and setting state; no validated ArduSub-to-ESP32 command path | Motor configuration and test controls remain unavailable |
| `gyro_bias_ready`, `offset_roll`, `offset_pitch`, `offset_yaw` | ROS calibration fields; no equivalent dashboard contract | Calibration indicators remain unavailable |
| `cpu_temp`, `ambient_temp`, `logs` | Companion/system sources not connected to this API | Do not substitute water temperature or invented logs |

The reference page also contains functions that call `/api/arm`, `/api/kill_all`, motor test, parameter, mission, camera, and flight-plan routes. The new API exposes no such routes. Before enabling any of those panels, define its actual ArduSub or companion source, command acknowledgement, timeout and fault behavior, then verify the workflow with software replay. Hardware validation remains the final phase.
