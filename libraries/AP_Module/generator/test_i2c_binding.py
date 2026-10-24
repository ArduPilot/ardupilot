# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for the lifecycle-owned I2C binding catalogue."""

from pathlib import Path

import bindings
import i2c_binding


def test_i2c_selection_sdk_and_lifecycle_wiring():
    spec = i2c_binding.select(bindings.read_ir(), 800)
    assert [method['name'] for method in spec['methods']] == [
        'i2c.open', 'i2c.close', 'i2c.set_retries', 'i2c.set_address',
        'i2c.write_register', 'i2c.read_registers', 'i2c.transfer']
    assert [method['id'] for method in spec['methods']] == list(range(800, 807))
    rust = i2c_binding.rust(spec, '')
    assert 'pub struct I2cDevice' in rust
    assert 'send.len() > 255' in rust
    assert 'receive.copy_from_slice' in rust
    root = Path(__file__).resolve().parents[1]
    runtime = (root / 'Runtime.cpp').read_text()
    assert 'binding_i2c_cleanup(&slot);' in runtime


def test_i2c_coverage_and_dispatch_are_wired():
    spec = bindings.specification()
    coverage = {(entry['owner'], entry['name']): entry for entry in spec['coverage']['entries']}
    for owner, name in [
            ('i2c', 'get_device'), ('AP_HAL::I2CDevice', 'set_retries'),
            ('AP_HAL::I2CDevice', 'set_address'), ('AP_HAL::I2CDevice', 'write_register'),
            ('AP_HAL::I2CDevice', 'read_registers'), ('AP_HAL::I2CDevice', 'transfer')]:
        assert coverage[(owner, name)]['status'] == 'generated'
    source = bindings.emit(spec)['Bindings_generated.cpp']
    for call in ('open', 'close', 'set_retries', 'set_address', 'write_register',
                 'read_registers', 'transfer'):
        assert f'return binding_i2c_{call}' in source
