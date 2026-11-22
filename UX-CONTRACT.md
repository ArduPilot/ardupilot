# ATAL ArduSub dashboard UX contract

`/` serves the first ROS-style operator dashboard port, and `/diagnostics` serves the original compact instrument page. Both are read-only. `integration/dashboard_telemetry.py` owns MAVLink mapping and freshness; `/api/state` is their data source. Each page polls every 500 ms, times out at two seconds, and retries. Link loss and stale inputs remove values from the visible operator instruments. `/api/stream` emits the same snapshot as read-only Server-Sent Events; client connections do not affect the vehicle.

The operator dashboard reuses the reference topbar, color scheme, theme, and navigation vocabulary. Dashboard, Plots, Raw Data, theme toggle, and the diagnostic link are active. ROS-dependent control destinations are inert labels with an explicit integration-pending notice. There are no arm, kill, motor, parameter, mission, or calibration controls yet. Any future command UI must define acknowledgement, arming, timeout, and fault behavior before activation. Browser-native buttons, links, focus, and scrolling are canonical; the theme choice is stored locally.

## Canonical UI Map

| Capability | Canonical owner | Source of truth | Allowed variants | Verification |
|---|---|---|---|---|
| Telemetry mapping and freshness | `DashboardTelemetry` | `integration/dashboard_telemetry.py` | Live, stale, unavailable | `test_dashboard_telemetry.py`, browser live/stale check |
| Operator page, tabs, theme, plots | `operator_dashboard.html` and its CSS/JS | `integration/dashboard_api.py`, `DESIGN.md` | Waiting, connected, lost, request error; dark/light | API loopback, native replay, browser review |
| Diagnostic page and polling | `dashboard_view.html` | `integration/dashboard_api.py` | Waiting, connected, lost, request error | API loopback, browser review |
| Telemetry event stream | `dashboard_api.py` | `integration/dashboard_api.py` | Connected, stale | `test_dashboard_telemetry.py` |
