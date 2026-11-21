# Architecture decision (development architecture)

Status: selected architecture for development, not approved for physical deployment. Based on reference commit `9eadb1a92a9635c706c2f5ec6710167a6ab512e7` and official ArduPilot checkout `4ca1bc74fc088242931054cac963cdfd91589b51`. The native Linux `sub` target builds and runs with empty RC output. Synthetic Xsens/Bar30 and DVL-to-EKF replays pass; physical sensor timing, calibration and navigation readiness are still unverified.

## Options

| Option | Strengths | Risks / missing pieces | Decision |
|---|---|---|---|
| A. Native ArduSub Linux HAL with bespoke hardware drivers | One process owns estimation/control/mixing; no SITL transport in the control path; straightforward deployment model if Linux board supports all required services. | Xsens MTi-300 has no verified upstream native backend; Linux x86 board/peripheral support must be built and run verified. Direct ESP32 SRV output integration is custom and safety critical. | Keep as a candidate for the base process, but avoid custom HAL sensor/motor drivers until capabilities are proved. |
| B. ArduSub plus external sensor adapters | Reuses existing Xsens/DVL/depth communication and isolates device protocols. MAVLink ExternalNav/odometry is a documented EKF input path; adapters can be replay tested. | Xsens orientation alone is not IMU acceleration/gyro integration; External AHRS upstream drivers list does not include Xsens MTi-300. Sensor timing, frames, covariance and EKF source configuration need explicit validation. Bar30 is pressure, not an arbitrary depth setpoint. | Select for first migration integration: start ArduSub with native Linux HAL, feed DVL odometry through supported MAVLink, assess an upstream supported Xsens route before writing an INS backend. Keep depth via ESP32 and choose a proper ArduPilot pressure input path only after checking the version APIs. |
| C. ArduSub + ROS 2 high-level companion | Preserves vision, gate guidance, mission logic, dashboard and logs while assigning low-level authority to ArduSub. The reference already separates these high-level nodes from the low-level pipeline. | Existing dashboard ROS bridge publishes legacy arm/test/control topics and must be adapted; ROS 2 must never retain a second motor-output owner. | Select as a composition with B. Initially bridge telemetry and bounded high-level MAVLink commands only; leave legacy controller, mixer, and serial bridge out of the ArduSub runtime. |

## Decision

Develop an upstream ArduPilot checkout at repository root and a thin `integration/` layer here. Do not maintain a parallel rewritten autopilot fork if a documented interface can carry the measurements. Preserve upstream history and isolate local changes in integration files or small upstream patches.

For estimator development, keep the Xsens sample as inertial source of record. A focused `AP_ExternalAHRS_Xsens` backend now injects raw gyro/acceleration through `AP_InertialSensor_ExternalAHRS`; a separate `AP_Baro_ATAL_Depth` backend consumes the ESP32 pressure port. Both pass synthetic native replay, while physical timing, alignment, calibration and EKF initialization are unverified. The generic external navigation MAVLink path carries pose/velocity and cannot replace the high-rate inertial sensor. Do not fuse ROS's receipt-time quaternion as if it were independent global position.

For DVL, the reference provides body velocity but does not correctly parse the A50's separate `position_local` packets or preserve `time_of_validity` and covariance. This checkout's `GCS_MAVLink/GCS_Common.cpp:handle_vision_speed_estimate` accepts NED velocity, while `handle_odometry` also supplies pose. Use `VISION_SPEED_ESTIMATE` for velocity-only input once A50 body FRD to NED rotation is based on a fresh, validated attitude. Do not synthesize global position from velocity in the migration adapter without tested integration and reset behavior. Manufacturer axes and clock semantics are documented at [Water Linked A50 JSON protocol](https://docs.waterlinked.com/dvl/dvl-json-protocol/) and [axes](https://docs.waterlinked.com/dvl/axes/).

For depth, retain the current ESP32-to-host serial connection and its Bar30 sensor. The dedicated backend consumes mbar and Celsius as water pressure and temperature; surface-relative metres are diagnostic. The line has no device timestamp, sequence or health quality, so host receipt age drives freshness. Synthetic replay checks pressure and stale-input health, but surface zero and real pressure calibration remain open.

For propulsion, ArduPilot `AP_Motors6DOF` is the preferred allocator only if the actual eight actuator vectors and motor numbering map to a supported frame. Reference factors are a provisional permutation of Vectored-6DOF; placeholder positions and contradictory firmware comments mean physical geometry is not established. Keep output disabled and compare exact AP_Motors mapping against the reference config before allowing software output tests.

## Rejected shortcuts

* SITL success alone does not demonstrate the native Linux HAL hardware runtime.
* `SIM_*` hardware simulation or an assumed Cube/Pixhawk does not fit the x86 PC design.
* External AHRS's existing supported vendor list does not include Xsens MTi-300, so configuration alone is not an integration.
* Velocity-only DVL is not global position; arbitrary integration would drift and must not be described as position hold quality.
* Existing ESP32 firmware automatically accepts partial motor updates and enables command parsing after boot. It is not ready to become an ArduPilot actuator endpoint unchanged.

## Consequences and acceptance gates

1. Build and run the upstream `sub` Linux target with simulated/disconnected outputs before adapter work; record commit and exact commands.
2. Keep all actuator interfaces absent in development launch/config. Prove the default run cannot open a tty or write a motor packet.
3. Add independent, replayable protocol parsers and freshness checks before sensor fusion.
4. Assign ArduSub the only low-level control authority. Migrate ROS high-level functions behind explicit MAVLink/ROS bridge APIs.
5. Keep hardware geometry, rotations, lever arms and physical dimensions editable, initialized from the reference defaults and clearly marked provisional.
6. Do not use the standalone ESP32 actuator port or flash its firmware during this phase.
