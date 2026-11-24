# Canonical ROS 2 dashboard source snapshot

This directory is a byte-for-byte copy of `ATAL_controller/web_dashboard/templates` and `web_dashboard/static` at commit `9eadb1a92a9635c706c2f5ec6710167a6ab512e7`. The sibling ROS 2 checkout remains unchanged and is still the deployment fallback.

These files are the UI source for the eventual ArduSub dashboard. `/classic` now renders the original templates and CSS through `classic_dashboard.py`, but strips **all original scripts** and loads `classic_adapter.js` instead. The ROS controller, motor, firmware, mission, and flight-plan JavaScript is never served on that route. A Content Security Policy additionally blocks inline script execution. The source files in this directory remain untouched and byte-for-byte identical to the ROS checkout.

The `/classic` adapter currently displays live ArduSub attitude, pressure, water temperature, mode, armed state, EKF, motor PWM, mission status, a rolling attitude/depth plot, raw snapshots, and optional companion gate proposals. Unsupported controls are disabled and their panels are labelled as unported. It is a read-only software milestone, not a vehicle-command dashboard. Continue adapting the original UI against verified ArduSub/companion contracts; only enable actions after request, acknowledgement, timeout, failure, and cancel paths pass appropriate checks.

The current usable telemetry page remains `integration/operator_dashboard.html`. See `docs/ardusub_migration/12_dashboard_port.md` and `13_dashboard_field_map.md` for the existing field and route inventory.

Run `python3 integration/check_dashboard_parity.py` to see the literal API paths called by this snapshot and which paths the new API currently exposes. A shared URL is not proof of equivalent fields or behavior; dynamic routes also need manual inspection.
