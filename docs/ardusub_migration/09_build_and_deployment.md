# Build and deployment

Pinned source: ArduPilot `4ca1bc74fc088242931054cac963cdfd91589b51`, shallow upstream `master`, local branch `feature/atal-ardusub-migration`. Working tree is an independent checkout. No remote fork/push has been created.

Development host: Ubuntu 22.04.5 x86-64, Python 3.10.12, g++ 11.4.0. Build prerequisites and all submodules are managed by ArduPilot's `BUILD.md` and `.gitmodules`. On a fresh clone use `git submodule update --init --recursive` (record exact submodule SHAs from the pinned superproject); then:

```bash
./waf configure --board=linux
./waf sub -j4
```

`libraries/AP_HAL_Linux/hwdef/linux/hwdef.dat` selects native toolchain and `HAL_INS_NONE`. `HAL_Linux_Class.cpp` chooses `Empty::RCOutput` on this generic board. The Xsens and Bar30 serial backends have passed synthetic pseudo-terminal replay, and a separate native DVL replay produced EKF3 velocity states with external velocity selected. The opt-in RCOutput observer and guarded serial bridge passed a native mixer-to-pseudo-terminal replay, but the staged ESP32 sketch has no GPIO/ESC output and no physical actuator path has been qualified. Physical-sensor and navigation readiness remain unverified. Use this build only for software investigation with actuator power isolated.

`integration/atal_sensor_defaults.parm` is an editable provisional parameter template. `SERIAL1_PROTOCOL=36` selects Xsens and `SERIAL2_PROTOCOL=51` selects the separate ESP32 depth stream; both use the ROS 2 reference's 115200 baud default (`SERIALx_BAUD=115`). `AHRS_ORIENTATION=0` retains the reference's identity mounting placeholder, and `BARO_SPEC_GRAV=1.0` is its provisional freshwater setting. Assign the real port paths and measured values before any vehicle deployment. `integration/README.md` lists every provisional edit point and the checks to rerun after changing it. To reproduce the software-only combined replay on the normal Linux build:

```bash
python3 integration/test_depth_native_replay.py
```

For a software-only Xsens startup check on generic Linux, build with the test-only barometer exception and run the pseudo-terminal harness:

```bash
./waf configure --board=linux --extra-hwdef integration/atal_linux_test.hwdef
./waf sub -j4
python3 integration/test_xsens_native_startup.py
```

The generic Linux board has no barometer and otherwise stops before IMU startup. This profile permits the process to start for virtual-serial Xsens command and sample replay; it has no pressure source and must never be used for deployment. The harness imports `pymavlink` from the checked-out `modules/mavlink` submodule. It retains generic Linux `Empty::RCOutput`. Restore the ordinary build with `./waf configure --board=linux` and `./waf sub -j4` when the test profile is no longer needed.

The dashboard API can read a separate native MAVLink stream on `SERIAL3` while `SERIAL0` remains assigned to another consumer. `python3 integration/test_dashboard_native_replay.py` proves this with temporary `SERIAL3_PROTOCOL=2`, `SR3_EXTRA1=10`, `SR3_EXTRA2=5`, and `SR3_RAW_SENS=5` settings and virtual sensors; it also checks HTTP, SSE, and local CSV recording. `python3 integration/test_dashboard_api_loopback.py` checks the UDP-to-HTTP service path. The operator page at `/` is the first ROS-style UI port, with control-dependent panels still unavailable. For a later vehicle configuration, use an explicit serial endpoint and verified stream rates. Neither replay opens a vehicle port or proves physical readiness.

Provisional vehicle geometry lives in `integration/vehicle_defaults.json`, with the matching ArduSub frame/direction/PWM parameter template in `integration/atal_motor_defaults.parm`. Change both when measured geometry or polarity changes, then re-run `python3 integration/check_layout.py`; the validator checks motor direction and `SERVO1`–`SERVO8` function assignments. The JSON is not loaded into ArduSub automatically. The native replay normally uses generic Linux `Empty::RCOutput`. For software-only output observation, `integration/atal_loopback_output.hwdef` selects `RCOutput_ATAL_Loopback`, which publishes raw channel values to localhost UDP port 14660; `python3 integration/test_rcoutput_native_loopback.py` exercises it with virtual sensors. No deployment launcher, firmware flashing or startup-service changes exist. The old ROS 2 deployment remains the rollback path, subject to its normal operator procedure.

For development of the A50 velocity path, install the pinned `modules/mavlink/pymavlink` Python package and run the native binary with `--serial0 udp:127.0.0.1:14551` and private log/storage/terrain directories. Run `python3 integration/dvl_bridge.py --connect-a50` separately only when access to the A50 sensor is intended. The bridge accepts attitude from ArduSub system/component 1/1 and sends velocity-only MAVLink 2 on loopback; it has no actuator output. Physical Xsens and pressure inputs are still untested, so this is an interface exercise, not a usable autopilot launch or verified DVL fusion.
