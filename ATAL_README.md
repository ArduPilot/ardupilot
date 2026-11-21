# ATAL ArduSub migration

This is an independent ArduPilot checkout for migrating the ATAL AUV from the read-only ROS 2 `ATAL_controller` reference. The upstream base is pinned at `4ca1bc74fc088242931054cac963cdfd91589b51`; ATAL work is on local branch `feature/atal-ardusub-migration`. Nothing in this repository has been pushed or deployed.

The native Linux `ardusub` build, synthetic Xsens/Bar30 serial replay, A50-to-EKF velocity replay, and provisional eight-motor mapping have software evidence. The generic Linux build still uses `Empty::RCOutput`, so it has no physical thruster output. The ESP32 command guard and read-only dashboard telemetry API are standalone integration pieces, not connected to deployed firmware or the original dashboard UI. A fully operational vehicle repo remains the goal; measured mounting, actual sensors, actuator wiring, and full dashboard workflows still require integration and supervised hardware validation.

Start with [editable defaults](integration/README.md) and the [migration progress](docs/ardusub_migration/10_migration_progress.md). The [ROS/dashboard ownership map](docs/ardusub_migration/11_ros_dashboard_coexistence.md) explains which original nodes cannot run alongside ArduSub.

For local software checks after the native build:

```bash
python3 integration/check_layout.py
python3 integration/simulate_allocation.py
python3 -m unittest discover -s integration -p 'test_*.py' -v
g++ -std=c++11 -Wall -Wextra -Werror -I integration integration/test_actuator_guard.cpp -o .atal-baseline/test_actuator_guard
.atal-baseline/test_actuator_guard
python3 integration/test_depth_native_replay.py
python3 integration/test_dvl_native_chain.py
```

The native replays use loopback links and pseudo-terminals. They do not open vehicle serial ports. The full test matrix and limitations are in [testing and validation](docs/ardusub_migration/08_testing_and_validation.md).
