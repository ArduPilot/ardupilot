# Migration progress

## 2026-11-20

- Created an independent working checkout rooted at ArduPilot, remote `https://github.com/ArduPilot/ardupilot.git`, commit `4ca1bc74fc088242931054cac963cdfd91589b51`, branch `feature/atal-ardusub-migration`. Waf Linux configuration and native `sub` build passed after required submodules were initialized. The resulting `build/linux/bin/ardusub` is an x86-64 ELF with all shared libraries resolved by `ldd`; `--help` works; the initial eight-second loopback run created local storage but stopped at a missing-barometer configuration error before IMU startup. The later software-test profile successfully reached virtual-serial Xsens and MAVLink telemetry.
- Audited `AUV-IITK/ATAL_controller` at `9eadb1a92a9635c706c2f5ec6710167a6ab512e7`; source checkout remains unmodified.
- Built both ROS 2 packages from a disposable archive on Ubuntu 22.04.5 / ROS Humble; both completed. 147 pytest cases passed. No vehicle or physical serial interface was opened.
- Wrote existing-system audit and initial architecture decision / implementation plan. These are evidence-based but architecture remains pending upstream Linux/driver inspection.
- User directed that physical parameters remain editable placeholders initialized from ROS 2 repository defaults.

- Added strict MTData2 parsing and a native Xsens MTi-300 ExternalAHRS IMU backend in local commits `891daa8` and `a2c368d`. Three synthetic parser tests and a fresh native Sub build pass. The initial PTY startup check saw no configuration bytes because the generic Linux board halted at barometer initialization. A test-only no-barometer hwdef now allows startup, and the PTY check observes all three Xsens configuration commands and `RAW_IMU` telemetry that matches synthetic MTData2 acceleration and gyro samples. Real sensor and estimator health remain unverified; a manual replay returned EKF flags `0`. No physical sensor was opened.

## Current state

M1 reference baseline: complete. M2 native build and software-test runtime: complete; deployed sensor and output paths are not integrated. The generic Linux board has no default IMU and an `Empty::RCOutput` driver. The new `integration/vehicle_defaults.json` preserves editable provisional ROS defaults, and `integration/check_layout.py` validates its six-axis rank and unique mapping to upstream Vectored-6DOF factors. A non-actuating A50 velocity bridge has been added; live end-to-end fusion remains unverified. No actuator driver, firmware change or real vehicle operation has been performed. Physical actuator interface remains absent.

## Next evidence

Verify Xsens timing and estimator input health with a real sensor, then integrate ESP32 Bar30 measurements with the native executable, then verify EKF initialization and DVL velocity fusion with replay. Implement a gated ESP32 output path only after sensor and estimation validation; keep physical output disabled during development.
