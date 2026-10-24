# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned camera values used by the AP_Scripting examples."""

from native_templates import render

OWNER = 'AP_Camera'
STATE = 'AP_Camera::camera_state_t'
INFORMATION = 'mavlink_camera_information_t'
VENDOR_SIZE = 32
MODEL_SIZE = 32
URI_SIZE = 140
STATE_WIRE_SIZE = 31
INFORMATION_WIRE_SIZE = 231
POLICY = ('v1:owned-camera-values,explicit-fields,no-struct-padding,camera-instance-u8,'
          'blocking-camera-semaphore,no-retained-module-pointers')


def _shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _method(owner, name, result, arguments, depends=None):
    matches = [method for method in owner['methods'] if method['name'] == name]
    if len(matches) != 1:
        raise ValueError(f'missing {OWNER}.{name}')
    method = matches[0]
    actual = (_shape(method['return']), [_shape(argument) for argument in method['arguments']])
    if (method['rename'] is not None or method['depends'] != depends or method['deprecate'] is not None or
            actual != (result, arguments)):
        raise ValueError(f'unaudited {OWNER}.{name} declaration')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected = dict(rename='camera', depends='AP_CAMERA_ENABLED && (AP_CAMERA_SCRIPTING_ENABLED == 1)',
                    get=None, literal=False, reference=False, semaphore=True,
                    scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited camera owner metadata')
    _method(owner, 'get_state', ('boolean', None, False, False, False, False, None),
            [('uint8_t', None, False, False, False, True, None),
             ('userdata', STATE, True, False, False, False, None)])
    _method(owner, 'set_camera_information', ('void', None, False, False, False, False, None),
            [('uint8_t', None, False, False, False, True, None),
             ('userdata', INFORMATION, False, False, False, False, None)],
            depends='AP_CAMERA_INFO_FROM_SCRIPT_ENABLED')
    return specification(first_id)


def specification(first_id):
    return dict(methods=[
        dict(id=first_id, name='camera.get_state', arguments=[dict(type='u8', name='instance')],
             result='optional-camera-state31', policy=POLICY),
        dict(id=first_id + 1, name='camera.set_information',
             arguments=[dict(type='u8', name='instance'),
                        dict(type='owned-camera-information230', name='information')],
             result='void', policy=POLICY),
    ], vendor_size=VENDOR_SIZE, model_size=MODEL_SIZE, uri_size=URI_SIZE)


def cpp(spec):
    state, information = spec['methods']
    return [
        f'// {state["name"]}',
        'static __attribute__((noinline)) int32_t binding_camera_get_state(',
        '    const uint8_t *input, uint32_t, uint8_t *output, uint32_t)',
        '{',
        '    (void)input;',
        f'    memset(output, 0, {STATE_WIRE_SIZE});',
        '#if AP_CAMERA_ENABLED && (AP_CAMERA_SCRIPTING_ENABLED == 1)',
        '    auto *camera = AP_Camera::get_singleton();',
        '    if (camera == nullptr) { return AP_MODULE_STATUS_UNAVAILABLE; }',
        '    BindingLock lock(camera->get_semaphore());',
        '    AP_Camera::camera_state_t value{};',
        '    if (!camera->get_state(input[0], value)) { return AP_MODULE_STATUS_OK; }',
        '    output[0] = 1;',
        '    BindingWire::write(output + 1, value.take_pic_incr, 2);',
        '    output[3] = value.recording_video ? 1 : 0;',
        '    output[4] = value.zoom_type;',
        '    BindingWire::write_float(output + 5, value.zoom_value);',
        '    output[9] = value.focus_type;',
        '    BindingWire::write_float(output + 10, value.focus_value);',
        '    output[14] = value.tracking_type;',
        '    BindingWire::write_float(output + 15, value.tracking_p1.x);',
        '    BindingWire::write_float(output + 19, value.tracking_p1.y);',
        '    BindingWire::write_float(output + 23, value.tracking_p2.x);',
        '    BindingWire::write_float(output + 27, value.tracking_p2.y);',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
        f'// {information["name"]}',
        'static __attribute__((noinline)) int32_t binding_camera_set_information(',
        '    const uint8_t *input, uint32_t, uint8_t *, uint32_t)',
        '{',
        '    (void)input;',
        '#if AP_CAMERA_ENABLED && (AP_CAMERA_SCRIPTING_ENABLED == 1) && AP_CAMERA_INFO_FROM_SCRIPT_ENABLED',
        '    auto *camera = AP_Camera::get_singleton();',
        '    if (camera == nullptr) { return AP_MODULE_STATUS_UNAVAILABLE; }',
        '    mavlink_camera_information_t value{};',
        '    value.firmware_version = BindingWire::read(input + 1, 4);',
        '    value.focal_length = BindingWire::read_float(input + 5);',
        '    value.sensor_size_h = BindingWire::read_float(input + 9);',
        '    value.sensor_size_v = BindingWire::read_float(input + 13);',
        '    value.flags = BindingWire::read(input + 17, 4);',
        '    value.resolution_h = uint16_t(BindingWire::read(input + 21, 2));',
        '    value.resolution_v = uint16_t(BindingWire::read(input + 23, 2));',
        '    value.cam_definition_version = uint16_t(BindingWire::read(input + 25, 2));',
        f'    memcpy(value.vendor_name, input + 27, {VENDOR_SIZE});',
        f'    memcpy(value.model_name, input + 59, {MODEL_SIZE});',
        f'    memcpy(value.cam_definition_uri, input + 91, {URI_SIZE});',
        '    BindingLock lock(camera->get_semaphore());',
        '    camera->set_camera_information(input[0], value);',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
    ]


def rust(spec, notice):
    state, information = spec['methods']
    return notice + render(
        'rust/camera.rs',
        vendor_size=VENDOR_SIZE,
        model_size=MODEL_SIZE,
        uri_size=URI_SIZE,
        state_wire_size=STATE_WIRE_SIZE,
        state_id=state['id'],
        information_wire_size=INFORMATION_WIRE_SIZE,
        information_id=information['id'],
    )
