# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned wire value for scripting battery-monitor updates."""

from native_templates import render

CELL_COUNT = 32
WIRE_SIZE = 90
POLICY = ('v1:complete-owned-copy,no-retained-module-pointers,all-f32-bits,'
          'u8-255-and-u16-65535-mean-unavailable,nan-means-unavailable-or-auto-integrate,'
          'cell-count-max32,zero-pad-none')


def specification(first_id):
    return dict(id=first_id, name='battery.handle_scripting',
                arguments=[dict(type='u8', direction='in'),
                           dict(type=dict(name='BattMonitorScriptState', fields=[
                               ['voltage', 'f32-le'], ['healthy', 'bool-u8'], ['cell_count', 'u8'],
                               ['capacity_remaining_pct', 'u8,255-unavailable'],
                               ['state_of_health_pct', 'u8,255-unavailable'],
                               ['cell_voltages', 'u16-le[32]'], ['cycle_count', 'u16-le,65535-unavailable'],
                               ['current_amps', 'f32-le,nan-unavailable'],
                               ['consumed_mah', 'f32-le,nan-auto-integrate'],
                               ['consumed_wh', 'f32-le,nan-unavailable'],
                               ['temperature', 'f32-le,nan-unavailable'],
                           ]), direction='in')], result='bool-u8', policy=POLICY)


def rust(method, notice):
    return notice + render(
        'rust/battery.rs',
        cell_count=CELL_COUNT,
        wire_size_plus_1=WIRE_SIZE + 1,
        method_id=method['id'],
    )
