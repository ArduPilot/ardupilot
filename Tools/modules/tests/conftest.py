# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Make the module-building tools importable by their tests."""

import sys

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
