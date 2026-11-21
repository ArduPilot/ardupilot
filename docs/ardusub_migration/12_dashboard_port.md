# Port the existing ATAL dashboard

The final ArduSub dashboard should closely retain the UI from the read-only ROS 2 reference `AUV-IITK/ATAL_controller@9eadb1a92a9635c706c2f5ec6710167a6ab512e7`. The temporary `integration/dashboard_view.html` is only an instrumentation page for software tests. It is not the visual target.

The reference presentation lives in `web_dashboard/templates/index.html`, companion templates for autotune, timed missions, and Control Lab, and `web_dashboard/static/dashboard.css`, `style.css`, scripts, and logos. Its navigation includes Dashboard, Throttle Control, Flight Plan, Mission Builder, Nano Perception, Gate Guidance, Waypoint Navigation, Plots, Raw Data, Tuning Guide, Debug, and Autotune. Keep this structure and its dark/light theme when porting the frontend; replace ROS-specific data and command assumptions behind it. Copying the original Flask `app.py` unchanged would also copy ROS node control, firmware flashing, and motor commands, so that backend needs deliberate replacement.

| Existing dashboard contract | ArduSub migration state | Next implementation boundary |
|---|---|---|
| `GET /api/state` and `GET /api/stream` | Implemented as read-only MAVLink snapshots and Server-Sent Events in `integration/dashboard_api.py`; native `SERIAL3` → API → HTTP replay passes | Compare every field used by the original `updateUI()` with the new mapper; display unavailable fields honestly |
| Dashboard shell, assets, theme, plots, raw data | Reference UI exists in ROS checkout; temporary read-only view exists here | Move the original presentation and its reusable client-side code into this repo, then bind to the new telemetry feed |
| Launch/node status, parameters, profiles, calibration | ROS routes remain in the reference backend | Replace each with an ArduSub parameter or process-management contract before enabling its UI action |
| Arm, kill, motor test, raw PWM, throttle and mode controls | No dashboard command routes exist here | Define request/acknowledgement, timeout, disarm, and fault behavior against ArduSub and the eventual guarded actuator path; verify with software replay before hardware |
| Missions, Flight Plan, waypoint navigation, gate guidance and autotune | ROS-specific services and controls remain in the reference | Reconcile each feature with available ArduSub modes, mission protocol, and companion services; retain the familiar UI only where behavior is implemented |
| Camera and Nano perception | Existing frontend and services remain in the reference | Port independently of the autopilot command path, then reconnect status and video surfaces |

The current telemetry mapper covers attitude, gyro, PWM, armed state, pressure, temperature, local position/depth, body velocity, and freshness. The reference `updateUI()` also reads controller targets and effort, motor settings, DVL status, thruster rate, logger state, CPU/ambient temperature, gate guidance, and mission/perception fields. Those fields must come from real ArduSub or companion data sources before the corresponding original panels are enabled; returning invented zeros would make a disconnected feature look healthy.

The original `GET /api/state` updates a dashboard heartbeat used by its ROS deadman watchdog. The new telemetry GET and SSE routes have no vehicle side effects. A future ArduSub control path must define its own explicit link-loss and command-loss behavior rather than restoring a hidden safety dependency on page polling.

The existing software checks are `python3 integration/test_dashboard_api_loopback.py` and `python3 integration/test_dashboard_native_replay.py`. Both use local UDP/HTTP and virtual sensors. They demonstrate the data path, not the final frontend or command behavior. Physical output and sensor commissioning remain the last integration phase.
