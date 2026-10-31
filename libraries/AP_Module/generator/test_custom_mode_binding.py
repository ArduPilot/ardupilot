# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate lifecycle-owned custom vehicle mode bindings."""

from pathlib import Path

import bindings
import custom_mode_binding


def test_custom_mode_selection_sdk_and_lifecycle_wiring():
    spec = custom_mode_binding.select(bindings.read_ir(), 900)
    assert [method['name'] for method in spec['methods']] == [
        'vehicle.register_custom_mode', 'CustomMode.allow_entry', 'CustomMode.set_allow_entry']
    assert [method['id'] for method in spec['methods']] == [900, 901, 902]
    assert 'module-context-owned-state-handle' in spec['methods'][0]['policy']
    assert 'cleanup-forces-allow-entry-false' in spec['methods'][0]['policy']
    rust = custom_mode_binding.rust(spec, '')
    assert 'pub struct CustomMode' in rust
    assert "impl crate::bindings::Vehicle<'_, '_>" in rust
    assert 'pub fn register_custom_mode' in rust
    assert 'full.contains(&0)' in rust

    root = Path(__file__).resolve().parents[1]
    runtime = (root / 'Runtime.cpp').read_text()
    cleanup = runtime.index('binding_custom_mode_cleanup(&slot);')
    release = runtime.index('Module::release(slot.module);', cleanup)
    assert cleanup < release
    host = (root / 'BindingCustomMode.cpp').read_text()
    cleanup = host.index('void binding_custom_mode_cleanup(void *context)')
    assert 'entry.state->allow_entry = false;' in host[cleanup:]
    assert 'entry = {};' in host[cleanup:]


def test_custom_mode_coverage_and_dispatch_are_wired():
    spec = bindings.specification()
    coverage = {(entry['owner'], entry['name']): entry for entry in spec['coverage']['entries']}
    registration = coverage[('AP_Vehicle', 'register_custom_mode')]
    assert registration['status'] == 'generated'
    assert registration['reason'] == 'module-lifecycle-owned-custom-mode-adapter'
    source = bindings.emit(spec)['Bindings_generated.cpp']
    for call in ('register', 'get_allow_entry', 'set_allow_entry'):
        assert f'return binding_custom_mode_{call}' in source
