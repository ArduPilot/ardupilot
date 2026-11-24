#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""List literal ROS dashboard API calls missing from the ArduSub dashboard API."""

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parent
REFERENCE = ROOT / "reference_dashboard"
FETCH = re.compile(r"fetch\(\s*['\"](/api/[^'\"?]+)")
EVENT_SOURCE = re.compile(r"new EventSource\(\s*['\"](/api/[^'\"?]+)")
ROUTE = re.compile(r'@app\.(?:get|post)\("([^"]+)')


def routes():
    sources = list((REFERENCE / "templates").glob("*.html"))
    sources += list((REFERENCE / "static").glob("*.js"))
    original = set()
    for path in sources:
        source = path.read_text()
        original.update(FETCH.findall(source))
        original.update(EVENT_SOURCE.findall(source))
    current = set(ROUTE.findall((ROOT / "dashboard_api.py").read_text()))
    return sorted(original), sorted(current), sorted(original - current)


def main():
    original, current, missing = routes()
    print(f"Reference UI literal API paths: {len(original)}")
    print(f"Current dashboard API paths: {len(current)}")
    print(f"Shared paths: {len(set(original) & set(current))}")
    print("Unported reference paths:")
    for path in missing:
        print(f"  {path}")
    print("Dynamic paths and field semantics still require manual review.")


if __name__ == "__main__":
    main()
