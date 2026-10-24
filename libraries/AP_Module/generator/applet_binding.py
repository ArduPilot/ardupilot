# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compact native adapters needed by remaining Lua applet and driver ports."""

from native_templates import render

GPS_OWNER = 'AP_GPS'
CAMERA_OWNER = 'AP_Camera'
CAMERA_VALUE = 'mavlink_video_stream_information_t'
NETWORKING_OWNER = 'AP_Networking'
SCRIPTING_OWNER = 'AP_Scripting'
GPS_INJECT_MAX = 1024
VIDEO_STREAM_NAME_SIZE = 32
VIDEO_STREAM_URI_SIZE = 160
VIDEO_STREAM_WIRE_SIZE = 215

GPS_POLICY = ('v1:gps-instance-checked,u64-little-endian,inject-nonempty-max1024,'
              'fixed-zero-padded-copy,no-retained-module-pointers')
CAMERA_POLICY = ('v1:owned-video-stream-value,all-lua-visible-fields-exact-width,'
                 'name-max32-uri-max160,nul-free-canonical-zero-padding,finite-nonnegative-framerate,'
                 'camera-device-id-host-owned,blocking-camera-semaphore,no-retained-module-pointers')
NETWORKING_POLICY = ('v1:persistent-route-shared-state,backend-and-interface-u8,destination-ip-u32-le,'
                     'prefix-length-0-to-32,no-exit-rollback,no-retained-module-pointers')
SCRIPTING_POLICY = ('v1:lua-backend-restart-request,caller-continues,rust-self-not-restarted,'
                    'unavailable-without-lua,no-retained-module-pointers')


def cpp_headers():
    """Extra include needed only when the Lua AP_Scripting class exists."""
    return ['#if AP_SCRIPTING_LUA_ENABLED', '#include <AP_Scripting/AP_Scripting.h>', '#endif']


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _owner(ir, name):
    owner = next((item for item in ir['singletons'] if item['name'] == name), None)
    if owner is None:
        raise ValueError(f'missing {name} singleton')
    return owner


def _check_owner(owner, expected):
    common = dict(get=None, literal=False, reference=False, scheduler_semaphore=False,
                  semaphore_pointer=False)
    common.update(expected)
    if any(owner[key] != value for key, value in common.items()):
        raise ValueError(f'unaudited {owner["name"]} owner metadata')


def _check_method(owner, name, result, arguments, depends=None):
    methods = [method for method in owner['methods'] if method['name'] == name]
    if len(methods) != 1:
        raise ValueError(f'missing {owner["name"]}.{name}')
    method = methods[0]
    actual = (_field_shape(method['return']), [_field_shape(argument) for argument in method['arguments']])
    if (method['rename'] is not None or method['depends'] != depends or method['deprecate'] is not None or
            actual != (result, arguments)):
        raise ValueError(f'unaudited {owner["name"]}.{name} declaration')


def _check_video_stream_value(ir):
    owner = next((item for item in ir['userdata'] if item['name'] == CAMERA_VALUE), None)
    expected_owner = dict(rename=None, depends='AP_CAMERA_INFO_FROM_SCRIPT_ENABLED', get=None, creation=None,
                          creation_manual=False, creation_args=0, has_fields=True, has_enums=False, operations=0,
                          operator_getter=None, operators_manual=False, operator_names=[], literal=False,
                          reference=False, semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected_owner.items()):
        raise ValueError('unaudited video-stream value metadata')
    expected = {
        'framerate': ('float', None),
        'bitrate': ('uint32_t', None),
        'flags': ('uint16_t', None),
        'resolution_h': ('uint16_t', None),
        'resolution_v': ('uint16_t', None),
        'rotation': ('uint16_t', None),
        'hfov': ('uint16_t', None),
        'stream_id': ('uint8_t', None),
        'count': ('uint8_t', None),
        'type': ('uint8_t', None),
        'name': ('uint8_t', str(VIDEO_STREAM_NAME_SIZE)),
        'uri': ('uint8_t', str(VIDEO_STREAM_URI_SIZE)),
        'encoding': ('uint8_t', None),
    }
    if {field['name'] for field in owner['fields']} != set(expected):
        raise ValueError('unaudited video-stream fields')
    for field in owner['fields']:
        kind, array_len = expected[field['name']]
        if (field['rename'] is not None or field['depends'] is not None or field['array_len'] != array_len or
                field['valid_mask_name'] is not None or field['valid_mask_value'] is not None or
                not field['read'] or not field['write'] or
                _field_shape(field['type']) != (kind, None, False, False, False, True, None)):
            raise ValueError(f'unaudited video-stream field {field["name"]}')


def select(ir, first_id):
    gps = _owner(ir, GPS_OWNER)
    _check_owner(gps, dict(rename='gps', depends='AP_GPS_ENABLED', semaphore=False))
    _check_method(gps, 'time_epoch_usec', ('userdata', 'uint64_t', False, False, False, False, None),
                  [('uint8_t', None, False, False, False, False, ('0', 'ud->num_sensors()'))])
    aliases = [alias for alias in gps['aliases'] if alias['alias'] == 'inject_data']
    if aliases != [dict(name='lua_gps_inject_data', alias='inject_data', depends=None, manual=True)]:
        raise ValueError('unaudited AP_GPS.inject_data declaration')

    camera = _owner(ir, CAMERA_OWNER)
    _check_owner(camera, dict(rename='camera', depends='AP_CAMERA_ENABLED && (AP_CAMERA_SCRIPTING_ENABLED == 1)',
                              semaphore=True))
    _check_method(camera, 'set_stream_information', ('void', None, False, False, False, False, None),
                  [('uint8_t', None, False, False, False, True, None),
                   ('userdata', CAMERA_VALUE, False, False, False, False, None)],
                  depends='AP_CAMERA_INFO_FROM_SCRIPT_ENABLED')
    _check_video_stream_value(ir)

    networking = _owner(ir, NETWORKING_OWNER)
    _check_owner(networking, dict(rename='networking', depends='AP_NETWORKING_ENABLED', semaphore=False))
    _check_method(networking, 'add_route', ('boolean', None, False, False, False, False, None),
                  [('uint8_t', None, False, False, False, True, None),
                   ('uint8_t', None, False, False, False, True, None),
                   ('uint32_t', None, False, False, False, True, None),
                   ('uint8_t', None, False, False, False, True, None)])

    scripting = _owner(ir, SCRIPTING_OWNER)
    _check_owner(scripting, dict(rename='scripting', depends=None, semaphore=False))
    _check_method(scripting, 'restart_all', ('void', None, False, False, False, False, None), [])
    return specification(first_id)


def specification(first_id):
    methods = [
        dict(id=first_id, name='gps.time_epoch_usec',
             arguments=[dict(type='u8,gps-instance', name='instance')], result='u64-le', policy=GPS_POLICY),
        dict(id=first_id + 1, name='gps.inject_data',
             arguments=[dict(type=f'nonempty-bytes-max{GPS_INJECT_MAX}-zero-pad{GPS_INJECT_MAX}', name='data')],
             result='void', policy=GPS_POLICY),
        dict(id=first_id + 2, name='camera.set_stream_information',
             arguments=[dict(type='u8', name='instance'),
                        dict(type='owned-video-stream-information', name='information')],
             result='void', policy=CAMERA_POLICY),
        dict(id=first_id + 3, name='networking.add_route',
             arguments=[dict(type='u8', name='backend_idx'), dict(type='u8', name='iface_idx'),
                        dict(type='u32-le', name='dest_ip'),
                        dict(type='u8', range=[0, 32], name='mask_len')],
             result='bool-u8', policy=NETWORKING_POLICY),
        dict(id=first_id + 4, name='scripting.restart_all', arguments=[], result='void', policy=SCRIPTING_POLICY),
    ]
    return dict(methods=methods, gps_inject_max=GPS_INJECT_MAX,
                video_stream_name_size=VIDEO_STREAM_NAME_SIZE,
                video_stream_uri_size=VIDEO_STREAM_URI_SIZE)


def cpp(spec):
    epoch, inject, camera, route, restart = spec['methods']
    return [
        'static bool binding_applet_valid_text(const uint8_t *data, uint16_t length)',
        '{',
        '    bool padding = false;',
        '    for (uint16_t i = 0; i < length; i++) {',
        '        if (padding && data[i] != 0) {',
        '            return false;',
        '        }',
        '        padding |= data[i] == 0;',
        '    }',
        '    return true;',
        '}',
        '',
        f'// {epoch["name"]}',
        'static __attribute__((noinline)) int32_t binding_applet_gps_time_epoch_usec(',
        '    const uint8_t *input, uint32_t, uint8_t *output, uint32_t)',
        '{',
        '    (void)input;',
        '    memset(output, 0, 8);',
        '#if AP_GPS_ENABLED',
        '    auto *gps = AP_GPS::get_singleton();',
        '    if (gps == nullptr) {',
        '        return AP_MODULE_STATUS_UNAVAILABLE;',
        '    }',
        '    if (input[0] >= gps->num_sensors()) {',
        '        return AP_MODULE_STATUS_INVALID;',
        '    }',
        '    const uint64_t value = gps->time_epoch_usec(input[0]);',
        '    BindingWire::write(output, uint32_t(value), 4);',
        '    BindingWire::write(output + 4, uint32_t(value >> 32), 4);',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
        f'// {inject["name"]}',
        'static __attribute__((noinline)) int32_t binding_applet_gps_inject_data(',
        '    const uint8_t *input, uint32_t, uint8_t *, uint32_t)',
        '{',
        '    const uint16_t length = uint16_t(BindingWire::read(input, 2));',
        f'    if (length == 0 || length > {GPS_INJECT_MAX}) {{',
        '        return AP_MODULE_STATUS_INVALID;',
        '    }',
        f'    for (uint16_t i = length; i < {GPS_INJECT_MAX}; i++) {{',
        '        if (input[2 + i] != 0) {',
        '            return AP_MODULE_STATUS_INVALID;',
        '        }',
        '    }',
        '#if AP_GPS_ENABLED',
        '    auto *gps = AP_GPS::get_singleton();',
        '    if (gps == nullptr) {',
        '        return AP_MODULE_STATUS_UNAVAILABLE;',
        '    }',
        '    gps->inject_data(input + 2, length);',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
        f'// {camera["name"]}',
        'static __attribute__((noinline)) int32_t binding_applet_camera_set_stream_information(',
        '    const uint8_t *input, uint32_t, uint8_t *, uint32_t)',
        '{',
        f'    if (!binding_applet_valid_text(input + 22, {VIDEO_STREAM_NAME_SIZE}) ||',
        f'        !binding_applet_valid_text(input + 54, {VIDEO_STREAM_URI_SIZE})) {{',
        '        return AP_MODULE_STATUS_INVALID;',
        '    }',
        '    mavlink_video_stream_information_t information{};',
        '    information.framerate = BindingWire::read_float(input + 1);',
        '    if (!std::isfinite(information.framerate) || information.framerate < 0) {',
        '        return AP_MODULE_STATUS_INVALID;',
        '    }',
        '    information.bitrate = BindingWire::read(input + 5, 4);',
        '    information.flags = uint16_t(BindingWire::read(input + 9, 2));',
        '    information.resolution_h = uint16_t(BindingWire::read(input + 11, 2));',
        '    information.resolution_v = uint16_t(BindingWire::read(input + 13, 2));',
        '    information.rotation = uint16_t(BindingWire::read(input + 15, 2));',
        '    information.hfov = uint16_t(BindingWire::read(input + 17, 2));',
        '    information.stream_id = input[19];',
        '    information.count = input[20];',
        '    information.type = input[21];',
        f'    memcpy(information.name, input + 22, {VIDEO_STREAM_NAME_SIZE});',
        f'    memcpy(information.uri, input + 54, {VIDEO_STREAM_URI_SIZE});',
        '    information.encoding = input[214];',
        '#if (AP_CAMERA_ENABLED && (AP_CAMERA_SCRIPTING_ENABLED == 1)) && AP_CAMERA_INFO_FROM_SCRIPT_ENABLED',
        '    auto *camera = AP_Camera::get_singleton();',
        '    if (camera == nullptr) {',
        '        return AP_MODULE_STATUS_UNAVAILABLE;',
        '    }',
        '    BindingLock lock(camera->get_semaphore());',
        '    camera->set_stream_information(input[0], information);',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
        f'// {route["name"]}',
        'static __attribute__((noinline)) int32_t binding_applet_networking_add_route(',
        '    const uint8_t *input, uint32_t, uint8_t *output, uint32_t)',
        '{',
        '    output[0] = 0;',
        '    if (input[6] > 32) {',
        '        return AP_MODULE_STATUS_INVALID;',
        '    }',
        '#if AP_NETWORKING_ENABLED',
        '    auto *networking = AP_Networking::get_singleton();',
        '    if (networking == nullptr) {',
        '        return AP_MODULE_STATUS_UNAVAILABLE;',
        '    }',
        '    output[0] = networking->add_route(input[0], input[1], BindingWire::read(input + 2, 4), input[6]);',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
        f'// {restart["name"]}',
        'static __attribute__((noinline)) int32_t binding_applet_scripting_restart_all(',
        '    const uint8_t *, uint32_t, uint8_t *, uint32_t)',
        '{',
        '#if AP_SCRIPTING_LUA_ENABLED',
        '    auto *scripting = AP_Scripting::get_singleton();',
        '    if (scripting == nullptr) {',
        '        return AP_MODULE_STATUS_UNAVAILABLE;',
        '    }',
        '    scripting->restart_all();',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
    ]


def rust(spec, notice):
    epoch, inject, camera, route, restart = spec['methods']
    return notice + render(
        'rust/applet.rs',
        gps_inject_max=GPS_INJECT_MAX,
        video_stream_name_size=VIDEO_STREAM_NAME_SIZE,
        video_stream_uri_size=VIDEO_STREAM_URI_SIZE,
        epoch_id=epoch['id'],
        inject_id=inject['id'],
        video_stream_wire_size=VIDEO_STREAM_WIRE_SIZE,
        camera_id=camera['id'],
        route_id=route['id'],
        restart_id=restart['id'],
    )
