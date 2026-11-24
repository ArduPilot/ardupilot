# Flight Plan and logger port boundary

The ROS 2 Flight Plan panel combines three separate behaviors: a companion-side step sequencer, an untethered launch trigger, and synchronized evidence files. The reference `web_dashboard/app.py` owns `/api/flight_plan/start`, `/stop`, `/status`, `/arm_untethered`, `/cancel_untethered`, `/logs`, `/download`, and `/resync_srt`; `web_dashboard/flight_logger.py` writes CSV, SRT, and milestone reports. Copying those routes would also copy ROS setpoint and tether-disconnect behavior into a system with different control ownership.

The current ArduSub dashboard implements only manual 10 Hz telemetry CSV recording. It has start/stop/status/download routes for local files and records fresh vehicle telemetry, including reported mission status. It does not claim a flight plan is running, trigger an untethered launch, create video timecodes, or calculate control errors from unavailable ROS setpoints.

Port order for software work:

1. Keep the passive recorder independent of command execution. Add session metadata and video timecode/SRT export only after the camera clock source and alignment rule are defined; test offset and sample gaps with synthetic clocks.
2. Define each Flight Plan step as an explicit ArduSub-supported command or mission item, with mode/arming prerequisites, acknowledgements, timeout, cancellation, and neutral/failsafe behavior. The ROS step name and timer alone do not prove that ArduSub executed the action.
3. Replay the sequencer and fault cases in SITL with a validated navigation source. Gate dashboard Start/Stop controls on observed state rather than local Python flags.
4. Treat untethered launch as a separate safety-critical feature. It needs an independent trigger, power and communications design, explicit cancel path, and supervised hardware validation. It must not be enabled by copying the ROS Ethernet-carrier watcher.

The companion logger may eventually reuse the familiar ROS panel and report format, but source values must be ArduSub telemetry or explicit companion observations. Unknown controller targets, effort, DVL lock, and mission milestones stay blank until those producers exist.

## 2026-11-25 software implementation update

`integration/timed_mission.py` adapts the validated `TimedMission` engine and `integration/flight_plan_runner.py` implements `TimedMissionRunner` and `FlightPlanRunner` (CSV/SRT/summary logs + ArduSub mode/failsafe sequencing), wired under `dashboard_api.py --enable-control`. `/api/flight_plan/arm_untethered` remains intentionally blocked with HTTP `403` pending hardware E-stop and tether-drop failsafe qualification.
