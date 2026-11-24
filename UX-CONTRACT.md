# ATAL ArduSub dashboard UX contract

`/` serves the first ROS-style operator dashboard port, and `/diagnostics` serves the original compact instrument page. Both are read-only. `integration/dashboard_telemetry.py` owns MAVLink mapping and freshness; `/api/state` is their data source. Each page polls every 500 ms, times out at two seconds, and retries. Link loss and stale inputs remove values from the visible operator instruments. `/api/stream` emits the same snapshot as read-only Server-Sent Events; client connections do not affect the vehicle.

`/classic` serves the exact original ROS 2 layout through a read-only adapter. It consumes `/api/stream` and never loads the original ROS script. Dashboard, Plots, Raw Data, mission status, optional gate proposals, and the local theme switch are available. Unsupported tabs explain their state; their original controls remain disabled. The stream-error state clears live values and shows a disconnected banner. This route is the visual source for future ports, while `/` retains the interim draft and recording features.

The operator dashboard reuses the reference topbar, color scheme, theme, and navigation vocabulary. Dashboard, Plots, Raw Data, Mission Builder, theme toggle, and the diagnostic link are active. Mission Builder is a browser-local draft editor with server-side structural validation and JSON import/export; it cannot upload or execute. Other ROS-dependent control destinations are inert labels with an explicit integration-pending notice. There are no arm, kill, motor, parameter, mission-command, or calibration controls yet. Any future command UI must define acknowledgement, arming, timeout, and fault behavior before activation. Browser-native buttons, links, focus, and scrolling are canonical; the theme choice and mission draft are stored locally.

The System Control EKF row displays fresh `EKF_STATUS_REPORT` flags as text. Missing reports become unavailable; an uninitialized flag is explicit. A position flag is never labelled mission-ready because `Sub::position_ok()` also depends on estimator and failsafe state unavailable to this read-only page.

## Canonical UI Map

| Capability | Canonical owner | Source of truth | Allowed variants | Verification |
|---|---|---|---|---|
| Telemetry mapping and freshness | `DashboardTelemetry` | `integration/dashboard_telemetry.py` | Live, stale, unavailable | `test_dashboard_telemetry.py`, browser live/stale check |
| Operator page, tabs, theme, plots | `operator_dashboard.html` and its CSS/JS | `integration/dashboard_api.py`, `DESIGN.md` | Waiting, connected, lost, request error; dark/light | API loopback, native replay, browser review |
| Original-layout read-only port | `reference_dashboard/` plus `classic_adapter.js` | `/api/stream` and `DESIGN.md` | Demo, connected, disconnected; supported and unported panels | API loopback, native replay, browser review |
| Diagnostic page and polling | `dashboard_view.html` | `integration/dashboard_api.py` | Waiting, connected, lost, request error | API loopback, browser review |
| Telemetry event stream | `dashboard_api.py` | `integration/dashboard_api.py` | Connected, stale | `test_dashboard_telemetry.py` |
