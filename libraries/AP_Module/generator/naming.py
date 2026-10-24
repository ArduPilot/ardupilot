# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Stable readable identifiers derived from public binding names."""

import re

RUST_KEYWORDS = {
    'as', 'break', 'const', 'continue', 'crate', 'else', 'enum', 'extern', 'false', 'fn', 'for', 'if', 'impl',
    'in', 'let', 'loop', 'match', 'mod', 'move', 'mut', 'pub', 'ref', 'return', 'self', 'Self', 'static', 'struct',
    'super', 'trait', 'true', 'type', 'unsafe', 'use', 'where', 'while', 'async', 'await', 'dyn', 'abstract',
    'become', 'box', 'do', 'final', 'macro', 'override', 'priv', 'typeof', 'unsized', 'virtual', 'yield', 'try',
}


def snake_case(name):
    """Convert C++/Lua-style names to Rust-style snake case."""
    name = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\1_\2', name)
    name = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', name)
    return re.sub(r'[^A-Za-z0-9]+', '_', name).strip('_').lower()


def binding_name(public_name):
    """Return the common readable stem used by C++ and Rust symbols."""
    name = snake_case(public_name)
    if not name or name[0].isdigit():
        raise ValueError(f'invalid public binding name: {public_name}')
    return name


def binding_constant(public_name):
    return binding_name(public_name).upper()


def rust_identifier(name):
    name = snake_case(name)
    return 'r#' + name if name in RUST_KEYWORDS else name


def rust_type(name):
    return ''.join(part[0].upper() + part[1:] for part in snake_case(name).split('_'))
