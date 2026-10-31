# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
import sys

from pathlib import Path

ROOT = Path(__file__).resolve().parents[6]
sys.path.insert(0, str(ROOT / 'Tools/modules'))
sys.path.insert(0, str(ROOT / 'Tools/modules/tests'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(ROOT / 'Tools/modules/rust/tests'))
