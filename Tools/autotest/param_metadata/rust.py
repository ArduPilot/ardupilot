# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Discover Rust module parameter documentation using the standard @Param tags."""

import re

from pathlib import Path

# Rust declarations can start with any identifier, including a lowercase variant.
# Unlike C++ tables and Lua comment blocks, they need no special terminator.
PARAM_RE = re.compile(
    r'@Param(?:{([^}]+)})?: (\w+)[^\n]*'
    r'((?:\n[ \t]*// @\w+(?:{[^}]+})?:[^\r\n]*)+)', re.MULTILINE)


def parameter_paths(apm_path):
    """Return paths relative to libraries/, as expected by process_library()."""
    root = Path(apm_path) / 'libraries'
    return [p.relative_to(root).as_posix()
            for directory in ('applets', 'drivers', 'examples')
            for p in sorted((root / 'AP_Module' / directory).rglob('*.rs'))]


def parameter_matches(text):
    return PARAM_RE.findall(text)
