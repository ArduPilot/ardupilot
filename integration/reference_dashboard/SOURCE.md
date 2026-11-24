# Canonical ROS 2 dashboard source snapshot

This directory is a byte-for-byte copy of `ATAL_controller/web_dashboard/templates` and `web_dashboard/static` at commit `9eadb1a92a9635c706c2f5ec6710167a6ab512e7`. The sibling ROS 2 checkout remains unchanged and is still the deployment fallback.

These files are the UI source for the eventual ArduSub dashboard. They are **not served or executed** by `dashboard_api.py` yet: the original JavaScript invokes ROS controller, motor, firmware flashing, mission, and flight-plan routes that the ArduSub API does not provide. Serving the page unchanged would expose controls with wrong or missing behavior. Adapt this copy against a verified ArduSub/companion API contract, preserving its layout and interactions wherever the semantics match. Keep unavailable command actions visibly disabled until their request, acknowledgement, timeout, failure, and cancel paths pass software and hardware checks.

The current usable telemetry page remains `integration/operator_dashboard.html`. See `docs/ardusub_migration/12_dashboard_port.md` and `13_dashboard_field_map.md` for the existing field and route inventory.
