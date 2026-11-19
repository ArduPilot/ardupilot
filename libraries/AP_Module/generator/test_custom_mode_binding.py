# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate lifecycle-owned custom vehicle mode bindings."""

from pathlib import Path

import bindings
import custom_mode_binding


def test_custom_mode_selection_sdk_and_lifecycle_wiring():
    spec = custom_mode_binding.select(bindings.read_ir(), 900)
    assert [method['name'] for method in spec['methods']] == [
        'vehicle.register_custom_mode', 'CustomMode.allow_entry', 'CustomMode.set_allow_entry',
        'CustomMode.set_request', 'CustomMode.status', 'CustomMode.set_options', 'CustomMode.set_arming_ready',
        'CustomMode.set_supervision', 'CustomMode.keepalive', 'CustomMode.ack_takeoff_event',
        'CustomMode.request_arm', 'CustomMode.set_target_posvelaccel']
    assert [method['id'] for method in spec['methods']] == list(range(900, 912))
    assert 'module-context-owned-state-handle' in spec['methods'][0]['policy']
    assert 'cleanup-forces-allow-entry-false' in spec['methods'][0]['policy']
    # the original contracts keep their policy, and so their hashes
    assert all(method['policy'] == custom_mode_binding.POLICY for method in spec['methods'][:3])
    # vehicle-wide EKF failsafe disarm changed the set_options contract
    assert spec['methods'][5]['policy'] == custom_mode_binding.OPTIONS_POLICY
    for method in spec['methods'][3:]:
        assert 'owner-checked' in method['policy']
        assert 'supervision-opt-in-latched-owner-loss' in method['policy']
    rust = custom_mode_binding.rust(spec, '')
    assert 'pub struct CustomMode' in rust
    assert "impl crate::bindings::Vehicle<'_, '_>" in rust
    assert 'pub fn register_custom_mode' in rust
    assert 'full.contains(&0)' in rust
    for name in ('set_request', 'status', 'set_options', 'set_arming_ready', 'set_supervision', 'keepalive',
                 'ack_takeoff_event', 'request_arm', 'set_target_posvelaccel'):
        assert f'pub fn {name}(' in rust
    assert 'pub enum CustomModeRequest' in rust

    root = Path(__file__).resolve().parents[1]
    runtime = (root / 'Runtime.cpp').read_text()
    cleanup = runtime.index('binding_custom_mode_cleanup(&slot);')
    release = runtime.index('Module::release(slot.module);', cleanup)
    assert cleanup < release
    host = (root / 'BindingCustomMode.cpp').read_text()
    cleanup = host.index('void binding_custom_mode_cleanup(void *context)')
    assert 'entry.state->allow_entry = false;' in host[cleanup:]
    assert 'entry.state->owner_loss_event = entry.state->supervision_ms != 0;' in host[cleanup:]
    assert 'entry = {};' in host[cleanup:]


def test_custom_mode_coverage_and_dispatch_are_wired():
    spec = bindings.specification()
    coverage = {(entry['owner'], entry['name']): entry for entry in spec['coverage']['entries']}
    registration = coverage[('AP_Vehicle', 'register_custom_mode')]
    assert registration['status'] == 'generated'
    assert registration['reason'] == 'module-lifecycle-owned-custom-mode-adapter'
    source = bindings.emit(spec)['Bindings_generated.cpp']
    for call in ('register', 'get_allow_entry', 'set_allow_entry', 'set_request', 'status', 'set_options',
                 'set_arming_ready', 'set_supervision', 'keepalive', 'ack_takeoff_event', 'request_arm',
                 'set_target_posvelaccel'):
        assert f'return binding_custom_mode_{call}' in source
    # controller requests are appended at the ABI tail
    methods = spec['custom_mode']['methods']
    tail = [method['id'] for method in methods[3:]]
    assert tail == list(range(tail[0], tail[0] + 9))
    assert tail[0] > max(method['id'] for method in methods[:3])
