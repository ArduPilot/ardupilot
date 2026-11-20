# Build and deployment

Pinned source: ArduPilot `4ca1bc74fc088242931054cac963cdfd91589b51`, shallow upstream `master`, local branch `feature/atal-ardusub-migration`. Working tree is an independent checkout. No remote fork/push has been created.

Development host: Ubuntu 22.04.5 x86-64, Python 3.10.12, g++ 11.4.0. Build prerequisites and all submodules are managed by ArduPilot's `BUILD.md` and `.gitmodules`. On a fresh clone use `git submodule update --init --recursive` (record exact submodule SHAs from the pinned superproject); then:

```bash
./waf configure --board=linux
./waf sub -j4
```

`libraries/AP_HAL_Linux/hwdef/linux/hwdef.dat` selects native toolchain and `HAL_INS_NONE`. `HAL_Linux_Class.cpp` chooses `Empty::RCOutput` on this generic board. Build success does **not** make a deployable real-vehicle autopilot: Xsens/Bar30 backends and ESP32 output are not yet integrated, and no estimator initialization has been verified. Use this build only for software investigation with actuator power isolated.

Provisional hardware constants live in `integration/vehicle_defaults.json`; change that file and re-run `python3 integration/check_layout.py`. This is not yet loaded into ArduSub parameters automatically. No deployment launcher, firmware flashing or startup-service changes exist. The old ROS 2 deployment remains the rollback path, subject to its normal operator procedure.

For development of the A50 velocity path, install the pinned `modules/mavlink/pymavlink` Python package and run the native binary with `--serial0 udp:127.0.0.1:14551` and private log/storage/terrain directories. Run `python3 integration/dvl_bridge.py --connect-a50` separately only when access to the A50 sensor is intended. The bridge accepts attitude from ArduSub system/component 1/1 and sends velocity-only MAVLink 2 on loopback; it has no actuator output. The standalone binary still lacks a real IMU and pressure source, so this is an interface exercise, not a usable autopilot launch or verified DVL fusion.
