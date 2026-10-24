# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Use MAVLink 2 so integration tests can reassemble chunked diagnostics."""

import os

os.environ.setdefault('MAVLINK20', '1')
