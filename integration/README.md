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
