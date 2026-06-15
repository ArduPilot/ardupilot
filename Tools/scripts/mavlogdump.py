#!/usr/bin/env python3

"""Repo-local wrapper for pymavlink's mavlogdump.py."""

import runpy
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "modules" / "mavlink"))
runpy.run_path(str(ROOT / "modules" / "mavlink" / "pymavlink" / "tools" / "mavlogdump.py"), run_name="__main__")
