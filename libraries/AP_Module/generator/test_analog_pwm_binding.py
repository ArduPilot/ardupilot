# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate opaque analog and lifecycle-owned PWM source bindings."""

import copy

from pathlib import Path

import analog_pwm_binding
import bindings
import pytest


def test_analog_pwm_selection_is_fail_closed():
    ir = bindings.read_ir()
    spec = analog_pwm_binding.select(ir, 900)
    assert [method['id'] for method in spec['methods']] == list(range(900, 910))
    changed = copy.deepcopy(ir)
    source = next(item for item in changed['ap_objects']
                  if item['name'] == analog_pwm_binding.PWM_SOURCE)
    method = next(item for item in source['methods'] if item['name'] == 'set_pin')
    method['arguments'][0]['skip_check'] = False
    with pytest.raises(ValueError, match='unaudited PWMSource'):
        analog_pwm_binding.select(changed, 900)


def test_analog_pwm_sdk_lifecycle_coverage_and_dispatch():
    selected = analog_pwm_binding.select(bindings.read_ir(), 900)
    rust = analog_pwm_binding.rust(selected, '')
    assert 'pub struct AnalogSource' in rust
    assert 'pub struct PwmSource' in rust
    assert 'pub fn close(self' in rust
    assert 'Option<core::num::NonZeroU32>' not in rust

    root = Path(__file__).resolve().parents[1]
    runtime = (root / 'Runtime.cpp').read_text()
    cleanup = runtime.index('binding_analog_pwm_cleanup(&slot);')
    release = runtime.index('Module::release(slot.module);', cleanup)
    assert cleanup < release
    host = (root / 'BindingAnalogPWM.cpp').read_text()
    assert 'NEW_NOTHROW AP_HAL::PWMSource()' in host
    assert 'entry.source->set_pin(ANALOG_INPUT_NONE)' in host
    assert 'delete entry.source;' in host

    spec = bindings.specification()
    coverage = {(entry['owner'], entry['name']): entry for entry in spec['coverage']['entries']}
    expected = {
        (analog_pwm_binding.ANALOG_OWNER, 'channel'),
        *((analog_pwm_binding.ANALOG_SOURCE, name) for name in (
            'set_pin', 'voltage_average', 'voltage_latest', 'voltage_average_ratiometric')),
        *((analog_pwm_binding.PWM_SOURCE, name) for name in (
            'set_pin', 'get_pwm_us', 'get_pwm_avg_us')),
    }
    for key in expected:
        assert coverage[key]['status'] == 'generated'
        assert coverage[key]['reason'] == 'module-lifecycle-owned-analog-pwm-adapter'
    source = bindings.emit(spec)['Bindings_generated.cpp']
    for call in (
            'analog_open', 'analog_set_pin', 'analog_voltage_average', 'analog_voltage_latest',
            'analog_voltage_average_ratiometric', 'pwm_open', 'pwm_close', 'pwm_set_pin',
            'pwm_get_us', 'pwm_get_average_us'):
        assert f'return binding_{call}' in source
