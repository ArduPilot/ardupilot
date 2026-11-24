"""Render the preserved ROS dashboard with an isolated ArduSub read-only adapter.

The source snapshot is intentionally never edited. Its ROS script is removed before
delivery because it contains controller, motor, and mission commands.
"""

import re

from flask import render_template


SCRIPT = re.compile(r"<script\b[^>]*>.*?</script\s*>", re.IGNORECASE | re.DOTALL)


def render_classic_dashboard():
    page = render_template("index.html", initial_tab="")
    page = SCRIPT.sub("", page)
    page = page.replace("/static/", "/reference-static/")
    page = page.replace("</head>", '<link rel="stylesheet" href="/static/classic_adapter.css"></head>', 1)
    page = page.replace("</body>", '<script src="/static/classic_adapter.js"></script></body>', 1)
    return page
