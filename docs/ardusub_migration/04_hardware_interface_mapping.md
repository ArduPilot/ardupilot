# Hardware interface mapping

These values are editable placeholders from the read-only ROS 2 reference, not measured commissioning data. Edit `integration/vehicle_defaults.json` before any supervised hardware qualification. Current actuators.enabled is `false`.

| Device | Reference source | Provisional new value | Intended ArduSub interface | Status |
|---|---|---|---|---|
| Xsens MTi-300 | `config/sensor_config.yaml`; `xsens_mti_driver_node.py` | `/dev/ttyUSB0`, 115200, 100 Hz, identity mount, zero lever arm | Native Xsens raw IMU/attitude backend, likely through `AP_ExternalAHRS` and `AP_InertialSensor_ExternalAHRS` | No backend yet. ROS driver validates only checksum and quaternion presence; packet counter is requested but unused. |
| A50 | Active `launch/control_system.launch.py`, `dvl_driver_node.py` | 192.168.194.95:16171, identity mount/zero lever arm; FOM max 0.25 m/s; 0.75 s adapter timeout | ExternalNav NED velocity via MAVLink `VISION_SPEED_ESTIMATE` | Pure parser and frame transform implemented; MAVLink transport and estimator test pending. |
| Bar30 on dedicated ESP32 | `firmware/esp32_depth/depth_esp_final.ino`, `esp32_bridge_node.py`; sensor offset from `config/sensor_config.yaml` | Port auto, 115200; freshwater 1000 kg/m3; offset [-0.238205,0.040534,-0.023156] m FRD | Custom serial pressure backend, or compatible ArduPilot external barometer backend; keep existing ESP32 wiring | Path selection pending. Do not treat surface-relative depth as raw pressure. |
| Eight ESCs on thruster ESP32 | `config/thruster_layout.yaml`, `firmware/esp32_thrusters/thruster_esp_final.ino`, `esp32_bridge_node.py` | `firmware_esp32_id_to_gpio` [27,12,18,19,4,14,5,13], logical layout in JSON | ArduSub `AP_Motors6DOF`, future Linux RCOutput/ESP32 serial path | **Disabled**. No output backend or serial port open. |

Source paths in this table are relative to `AUV-IITK/ATAL_controller@9eadb1a...`. The new runtime must use only files in this repository and pinned dependencies.

The reference's `config/sensor_config.yaml` still describes a Nortek A30 and native pressure I²C, whereas the active launch and device driver use an A50 TCP connection and depth ESP32. The active paths take precedence here. Actual Xsens/DVL rotations, lever arms, polarity and MCU firmware identity require later measurement. The user's direction is to retain reference defaults as easily edited placeholders until then.
