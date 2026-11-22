# Port the existing ATAL dashboard

The final ArduSub dashboard should closely retain the UI from the read-only ROS 2 reference `AUV-IITK/ATAL_controller@9eadb1a92a9635c706c2f5ec6710167a6ab512e7`. The first port is `integration/operator_dashboard.html` at `/`: it uses the reference CSS and logos, header, navigation vocabulary, dark/light theme, orientation/depth/thruster layout, plots, and raw data. The earlier `integration/dashboard_view.html` remains at `/diagnostics` as a software-test instrument page. The UI port is incomplete; command-dependent sections are visibly unavailable.

The reference presentation lives in `web_dashboard/templates/index.html`, companion templates for autotune, timed missions, and Control Lab, and `web_dashboard/static/dashboard.css`, `style.css`, scripts, and logos. Its navigation includes Dashboard, Throttle Control, Flight Plan, Mission Builder, Nano Perception, Gate Guidance, Waypoint Navigation, Plots, Raw Data, Tuning Guide, Debug, and Autotune. Keep this structure and its dark/light theme when porting the frontend; replace ROS-specific data and command assumptions behind it. Copying the original Flask `app.py` unchanged would also copy ROS node control, firmware flashing, and motor commands, so that backend needs deliberate replacement.

The per-field comparison for the reference `updateUI(s)` is in [13_dashboard_field_map.md](13_dashboard_field_map.md).

The mission-command dependency and software validation order are in [14_mission_port_boundary.md](14_mission_port_boundary.md).
The separate Flight Plan sequencer, untethered launch, and logger boundaries are in [15_flight_plan_port_boundary.md](15_flight_plan_port_boundary.md).

| Existing dashboard contract | ArduSub migration state | Next implementation boundary |
|---|---|---|
| `GET /api/state` and `GET /api/stream` | Implemented as read-only MAVLink snapshots and Server-Sent Events in `integration/dashboard_api.py`; native `SERIAL3` → API → HTTP replay passes | Compare every field used by the original `updateUI()` with the new mapper; display unavailable fields honestly |
| Dashboard shell, assets, theme, plots, raw data | First read-only port of reference shell and assets at `/`, bound to MAVLink telemetry | Continue porting remaining original panels only with real data and behavior; retain original UI structure |
| Launch/node status, parameters, profiles, calibration | ROS routes remain in the reference backend | Replace each with an ArduSub parameter or process-management contract before enabling its UI action |
| Arm, kill, motor test, raw PWM, throttle and mode controls | No dashboard command routes exist here | Define request/acknowledgement, timeout, disarm, and fault behavior against ArduSub and the eventual guarded actuator path; verify with software replay before hardware |
| Missions, Flight Plan, waypoint navigation, gate guidance and autotune | Read-only `MISSION_CURRENT` status now appears on the dashboard; ROS-specific controls remain in the reference | Reconcile each command workflow with ArduSub's mission protocol and companion services before enabling it |
| Camera and Nano perception | Existing frontend and services remain in the reference | Port independently of the autopilot command path, then reconnect status and video surfaces |
| Telemetry recording | Local 10 Hz CSV start/stop and status are available on the operator page | Add mission/video alignment only when corresponding workflows have been ported |

The current telemetry mapper covers attitude, gyro, PWM, armed state, pressure, temperature, local position/depth, body velocity, and freshness. Its `dvl_locked` field is `null` because the available ArduSub stream does not report the A50's bottom-lock state; a fresh local-position message alone does not prove navigation validity. The reference `updateUI()` also reads controller targets and effort, motor settings, DVL status, thruster rate, ROS flight-plan logger state, CPU/ambient temperature, gate guidance, and mission/perception fields. Those fields must come from real ArduSub or companion data sources before the corresponding original panels are enabled; returning invented zeros would make a disconnected feature look healthy.

The original `GET /api/state` updates a dashboard heartbeat used by its ROS deadman watchdog. The new telemetry GET and SSE routes have no vehicle side effects. A future ArduSub control path must define its own explicit link-loss and command-loss behavior rather than restoring a hidden safety dependency on page polling.

The existing software checks are `python3 integration/test_dashboard_api_loopback.py` and `python3 integration/test_dashboard_native_replay.py`. Both use local UDP/HTTP and virtual sensors. Browser review checked waiting, live, stale, plots, raw data, and light theme. The depth gauge has a presentation-only `data-display-range-m` in `operator_dashboard.html`, initially 10 m, so it is easy to change without touching sensor mapping. These checks demonstrate the data path and first UI port, not command behavior. Physical output and sensor commissioning remain the last integration phase.

The local recorder writes normalized snapshots under ignored `integration/logs/dashboard/` by default, with a configurable `--recording-dir`. It preserves stale and unknown states in the CSV. It does not implement the ROS flight-plan logger's mission milestones, setpoint errors, subtitle/video alignment, or automatic flight-plan trigger.

The Mission Status card reads `MISSION_CURRENT` from ArduSub's extended-status stream. The native replay sets `SR3_EXT_STAT=2` and verifies the empty-vehicle state. Current item, total, state, and mode are unavailable when the message is stale or the field is unsupported. The card does not validate mission execution or navigation and exposes no mission command.
