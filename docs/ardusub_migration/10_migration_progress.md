# Migration progress

## 2026-11-20

- Created an independent working checkout rooted at ArduPilot, remote `https://github.com/ArduPilot/ardupilot.git`, commit `4ca1bc74fc088242931054cac963cdfd91589b51`, branch `feature/atal-ardusub-migration`. Waf Linux configuration and native `sub` build passed after required submodules were initialized. The resulting `build/linux/bin/ardusub` is an x86-64 ELF with all shared libraries resolved by `ldd`; `--help` works; the initial eight-second loopback run created local storage but stopped at a missing-barometer configuration error before IMU startup. The later software-test profile successfully reached virtual-serial Xsens and MAVLink telemetry.
- Audited `AUV-IITK/ATAL_controller` at `9eadb1a92a9635c706c2f5ec6710167a6ab512e7`; source checkout remains unmodified.
- Built both ROS 2 packages from a disposable archive on Ubuntu 22.04.5 / ROS Humble; both completed. 147 pytest cases passed. No vehicle or physical serial interface was opened.
- Wrote existing-system audit and initial architecture decision / implementation plan. These are evidence-based but architecture remains pending upstream Linux/driver inspection.
- User directed that physical parameters remain editable placeholders initialized from ROS 2 repository defaults.

- Added strict MTData2 parsing and a native Xsens MTi-300 ExternalAHRS IMU backend in local commits `891daa8` and `a2c368d`. Three synthetic parser tests and a fresh native Sub build pass. The initial PTY startup check saw no configuration bytes because the generic Linux board halted at barometer initialization. A test-only no-barometer hwdef now allows startup, and the PTY check observes all three Xsens configuration commands and `RAW_IMU` telemetry that matches synthetic MTData2 acceleration and gyro samples. Real sensor and estimator health remain unverified; a manual replay returned EKF flags `0`. No physical sensor was opened.
- Added the dedicated ESP32 depth-line parser and native Bar30 serial backend. A normal Linux build passes without the test-only missing-barometer exception. A combined pseudo-terminal replay fed synthetic Xsens and ESP32 depth streams to ArduSub, observed matching IMU/pressure/temperature telemetry and the water-pressure health bit, then confirmed health cleared after malformed depth input. The firmware's depth estimate is diagnostic; native ArduSub uses absolute pressure. No physical sensor was opened.
- Exercised the A50 bridge with a loopback TCP server and simulated ArduSub MAVLink attitude. Correct velocity and covariance reached `VISION_SPEED_ESTIMATE`; invalid bottom lock, duplicate timestamps and stale attitude suppressed packets, then fresh attitude restored output. Native ArduSub estimator fusion remains unverified.
- Compared the ROS 2 and ArduSub Vectored-6DOF source paths beyond factor mapping. Their normalized outputs diverge at saturation, and their thrust-to-PWM curves differ. The concrete roll/heave case is recorded in `06_thruster_allocation.md`; allocation equivalence remains an explicit gate before any output driver.
- Added provisional `FRAME_CONFIG`, eight motor directions and ROS PWM limits in `integration/atal_motor_defaults.parm`. The layout validator checks the directions against the editable JSON and the normal Linux virtual-sensor replay passes with the template loaded. No ESP32 or physical output path exists.

## Current state

M1 reference baseline: complete. M2 native build and software-test runtime: complete; physical sensor and output paths are unverified. The generic Linux board has no default IMU and an `Empty::RCOutput` driver. `integration/vehicle_defaults.json` and `integration/atal_sensor_defaults.parm` preserve editable provisional ROS defaults; `integration/check_layout.py` validates the geometry's six-axis rank and unique mapping to upstream Vectored-6DOF factors. A non-actuating A50 velocity bridge has been added; live end-to-end fusion remains unverified. No actuator driver, firmware change or real vehicle operation has been performed. Physical actuator interface remains absent.

## Next evidence

Verify Xsens timing and estimator input health with a real sensor; calibrate/zero the water-pressure sensor at the surface and check its measured medium; then verify EKF initialization and DVL velocity fusion with replay after enabling the MAVLink VisualOdom backend. Resolve the documented ROS-versus-ArduSub saturation and PWM differences in a software allocation test before implementing a gated ESP32 output path. Keep physical output disabled during development.
