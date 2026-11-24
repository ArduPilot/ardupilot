# Port the existing ATAL dashboard

The preserved ROS 2 UI templates and assets in `integration/reference_dashboard/` are served on `/classic` via `integration/classic_dashboard.py`, while `integration/operator_dashboard.html` remains available at `/` and `/diagnostics` serves `integration/dashboard_view.html`. The `/debug` route serves the preserved thruster diagnostics UI backed by `/api/debug_state`.

## Default read-only vs `--enable-control`

- **Default (`python3 integration/dashboard_api.py --mavlink udpin:127.0.0.1:14650`):** Strictly read-only. Exposes `/`, `/classic`, `/diagnostics`, `/debug`, `/api/state`, `/api/health`, `/api/stream`, `/api/recording*`, and `/api/mission/validate`; command POST routes return 404/405.
- **Interactive (`--enable-control`):** Attaches `ArduSubCommander`, `MissionClient`, `GuidedMissionRunner`, `TimedMissionRunner`, `FlightPlanRunner`, `RuntimeSupervisor`, and `GateGuidanceBridge`. `classic_adapter.js` and `timed_mission.js` enable mode, arm/kill, manual deadman control, PID/profile parameter tuning (`WP_SPD`, `PSC_D_*`, `PSC_NE_*`, `ATC_RAT_*`), mission upload/readback, Guided waypoint runner, Timed Missions, Flight Plan logging, Gate Guidance bridge toggle, and runtime supervision.
- **Deliberate safety lockouts:** `/api/flash_firmware` and `/api/flight_plan/arm_untethered` return HTTP `403`. `/api/motor_test_pwm` / motor-test routes remain blocked (`403`) unless both `--allow-software-motor-test` (or `actuators.enabled=true`) is explicitly supplied. `/api/autotune/state` reports honest `supported: false` status explaining ArduSub parameter tuning replaces ROS AutoTune.

## API parity & verification

`python3 integration/check_dashboard_parity.py` reports 60 literal reference API paths, 85 current dashboard API paths, and 57 shared paths (`node --check integration/static/classic_adapter.js` passes cleanly). Remaining prefix literals in the parity scanner (`/api/autotune/`, `/api/control-lab/`, `/api/timed-missions/`) are JS prefix builders covered by explicit routes (`/api/autotune/state`, `/api/timed-missions/*`).

See [13_dashboard_field_map.md](13_dashboard_field_map.md), [14_mission_port_boundary.md](14_mission_port_boundary.md), and [15_flight_plan_port_boundary.md](15_flight_plan_port_boundary.md) for field semantics and boundaries.
