# ATAL ArduSub dashboard UX contract

The current dashboard route `/` is a read-only telemetry page. `integration/dashboard_telemetry.py` owns the mapping and freshness rules; `/api/state` is the page's data source. The page polls it every 500 ms, waits at most two seconds for a response, and retries after failure. A failed request shows a connection error and unavailable readings. Link loss or stale sensor messages remove values that would otherwise look current, including local position, depth, and velocity. `/api/stream` emits the same snapshot as read-only Server-Sent Events for the eventual ROS dashboard migration; client connections do not affect vehicle state.

There are no application controls, forms, selection lists, destructive actions, or account states on this page. The browser's ordinary scrolling and focus behavior are canonical. The existing ROS dashboard is a reference for content and palette, but its ROS control routes are not part of this page. Any future command UI must define its own command acknowledgement, arming, timeout, and failure behavior before activation.

## Canonical UI Map

| Capability | Canonical owner | Source of truth | Allowed variants | Verification |
|---|---|---|---|---|
| Telemetry mapping and freshness | `DashboardTelemetry` | `integration/dashboard_telemetry.py` | Live, stale, unavailable | `test_dashboard_telemetry.py`, browser live/stale check |
| Read-only page and polling | `dashboard_view.html` | `integration/dashboard_api.py` and `integration/dashboard_view.html` | Waiting, connected, lost, request error | `test_dashboard_api_loopback.py`, browser check |
| Telemetry event stream | `dashboard_api.py` | `integration/dashboard_api.py` | Connected, stale | `test_dashboard_telemetry.py` |
