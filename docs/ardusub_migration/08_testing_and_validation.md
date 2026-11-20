# Testing and validation

| Check | Evidence | Result / scope |
|---|---|---|
| Reference build | `.atal-baseline/build.log`, git archive of reference `9eadb1a...`; ROS Humble / Ubuntu 22.04.5 | Both packages built; warnings only for disabled bytecompile. Original checkout untouched. |
| Reference unit suite | `.atal-baseline/tests.log` | 147 passed. Some tests mirror formulas and do not verify real device behavior. |
| Upstream Linux configuration | `.atal-baseline/waf-configure.log` | `./waf configure --board=linux` passed; native g++ 11.4.0. |
| Upstream native Sub build | `.atal-baseline/waf-sub.log` | Passed; x86-64 ELF produced at `build/linux/bin/ardusub`, shared libraries resolved by `ldd`. `--help` passed; eight-second loopback-only startup ended by timeout and created a local storage file. No sensor health or MAVLink telemetry verified. |
| Xsens parser unit suite | `./waf test --test-filter=AP_ExternalAHRS_Xsens` | Three parser checks passed in the previous session; synthetic packets only. |
| Native Sub rebuild with Xsens | `.atal-baseline/xsens-sub-build.log`; `./waf sub -j4` | Passed; `build/linux/bin/ardusub` links the Xsens backend. No sensor or output validation. |
| Generic Linux Xsens startup diagnosis | GDB breakpoint and PTY with `EAHRS_TYPE=12`, `SERIAL1_PROTOCOL=36` | `AP_Baro::init()` enters configuration-error handling before `ins.init()` because no barometer exists. This explains the initial zero-byte result. |
| Software-only Xsens command startup | `./waf configure --board=linux --extra-hwdef integration/atal_linux_test.hwdef`; `./waf sub -j4`; `python3 integration/test_xsens_native_startup.py` | Passed: three Xbus configuration commands observed on a pseudo-terminal, then synthetic 100 Hz MTData2 samples appeared in `RAW_IMU` with matching acceleration and gyro values. The test profile allows no barometer for development only; real-sensor and EKF health are not verified. |
| Layout validator | `python3 integration/check_layout.py` | Eight unique GPIOs, rank 6, unique AP factor mapping; no output. |
| A50 parser/frame checks | `python3 -m unittest discover -s integration -p 'test_*.py' -v` | Five checks pass: FRD→NED, bad lock/status/quality, staleness and lever arm, bridge freshness gate. No MAVLink packet or real DVL test. |

No SITL, estimator initialization, real sensor, replay log, fault-injection system test, physical output or wet commissioning has passed. Required future evidence: native binary runtime without ports; Xsens packet replay and EKF health; pressure replay and zeroing; DVL covariance/time/lock-loss replay and fusion; software output mapping and MCU protocol fault tests; ROS single-owner integration; hardware E-stop inspection under human supervision.
