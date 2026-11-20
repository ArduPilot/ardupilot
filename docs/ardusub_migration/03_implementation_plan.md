# Implementation plan

Use small verifiable milestones inside this repository; do not change the ROS 2 reference.

As of the current local commits, M1–M4 have software-only build/parser/replay evidence, M5 has bridge wire replay without native EKF fusion, and M6 has an algebraic motor mapping plus a documented saturation difference. M7–M9 remain open. The detailed evidence is in `08_testing_and_validation.md` and `10_migration_progress.md`.

1. **M1 baseline:** done for reference snapshot; packages build on Ubuntu 22.04/ROS Humble and 147 tests pass. Record local upstream ArduPilot commit once clone completes; preserve reference commit and rollback instructions.
2. **M2 Linux ArduSub:** build official ArduSub for upstream Linux board. Inspect runtime options and dependencies. Add only a local non-actuating invocation/config; prove no actuator serial device is opened. SITL remains an independent simulator.
3. **M3 Xsens:** write a standalone parser/unit validation based on reference Xbus decoder only after resolving data format/units and finding the supported AP integration. Check quaternion, gyro, acceleration, counter, timestamps, invalid/truncated frames and continuity. Do not open physical USB by default.
4. **M4 Bar30:** parse existing ESP32 `DEPTH:` stream in software from supplied/replay lines. Carry validity and monotonic receipt time. Check Pa conversion, water-density/surface-zero semantics and age. Select ArduPilot pressure backend after upstream source review.
5. **M5 A50:** capture/replay reference JSON; parse documented fields, validity, status, FOM, clock and frame semantics. Publish only a supported external-navigation measurement with correct coordinate transform/covariance and a source timeout. Replay bottom-lock loss and stale packets.
6. **M6 allocation:** reconcile AP_Motors6DOF motor order with editable reference factors, pins and polarity. Verify rank and expected force/moment signs in a pure math test. Require physical geometry confirmation before deployment.
7. **M7 output:** excluded in current implementation. First define atomic command frame, explicit arm handshake, watchdog, disconnect behavior and ESP32 independent E-stop. Future code uses a software-only sink until separately authorized hardware work.
8. **M8 ROS:** preserve high-level gate perception/guidance, missions, dashboard read-only telemetry and logging. Adapt them to ArduSub MAVLink APIs, remove old low-level outputs from the new launch and prove a single command owner.
9. **M9 validation:** no-actuation build, test suite, recorded sensor replay, injected invalid/stale/disconnect faults, simulated allocation and SITL comparisons. Physical commissioning remains separate and requires explicit authorization.

## Rollback

The existing `AUV-IITK/ATAL_controller` checkout remains an untouched independent fallback. New ArduSub changes are committed only in this repository. Switching deployments is not part of software-only development; restore the existing ROS 2 deployment only through its existing operator procedure and with vehicle outputs disabled.
