# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for owned camera example values."""

import bindings
import camera_binding


def test_camera_selection_emission_and_coverage():
    camera = camera_binding.select(bindings.read_ir(), 900)
    assert [method['name'] for method in camera['methods']] == [
        'camera.get_state', 'camera.set_information']
    assert [method['id'] for method in camera['methods']] == [900, 901]
    cpp = '\n'.join(camera_binding.cpp(camera))
    assert 'AP_Camera::camera_state_t value{}' in cpp
    assert 'mavlink_camera_information_t value{}' in cpp
    rust = camera_binding.rust(camera, '')
    assert 'pub struct CameraState' in rust
    assert 'pub struct CameraInformation' in rust

    spec = bindings.specification()
    coverage = {(entry['owner'], entry['name']): entry for entry in spec['coverage']['entries']}
    assert coverage[('AP_Camera', 'get_state')]['status'] == 'generated'
    assert coverage[('AP_Camera', 'set_camera_information')]['status'] == 'generated'
