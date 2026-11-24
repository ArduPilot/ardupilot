"""Render the preserved ROS dashboard with an ArduSub adapter.

The source snapshot under reference_dashboard/ is intentionally never edited.
Its ROS-specific inline scripts are removed before delivery; when
control_enabled=True, classic_adapter.js and timed_mission.js wire the exact
UI panels to the ArduSub MAVLink control and mission APIs.
"""

import re

from flask import render_template


SCRIPT = re.compile(r"<script\b[^>]*>.*?</script\s*>", re.IGNORECASE | re.DOTALL)


def render_classic_dashboard(control_enabled=False):
    page = render_template("index.html", initial_tab="")
    page = SCRIPT.sub("", page)
    page = page.replace("/static/", "/reference-static/")
    page = page.replace(
        "<body>",
        f'<body data-atal-control="{"1" if control_enabled else "0"}">',
        1,
    )
    page = page.replace(
        "</head>",
        '<link rel="stylesheet" href="/static/classic_adapter.css"></head>',
        1,
    )
    scripts = '<script src="/static/classic_adapter.js"></script>'
    if control_enabled:
        scripts += '<script src="/reference-static/timed_mission.js"></script>'
    page = page.replace("</body>", f"{scripts}</body>", 1)
    return page
