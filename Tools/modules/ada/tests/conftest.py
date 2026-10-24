# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
import sys

from pathlib import Path

HERE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / 'tests'))
