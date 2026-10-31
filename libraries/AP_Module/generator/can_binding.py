# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Module-owned raw CAN buffers with an owned fixed-size frame value."""

from native_templates import render

OWNER = 'CAN'
BUFFER = 'ScriptingCANBuffer'
FRAME = 'AP_HAL::CANFrame'
DEPENDS = 'AP_SCRIPTING_CAN_SENSOR_ENABLED'
BUFFER_LENGTH_MAX = 25
BUFFER_COUNT_MAX = 16
FILTER_COUNT_MAX = 8
FRAME_DATA_MAX = 64
FRAME_WIRE_SIZE = 4 + 1 + 1 + FRAME_DATA_MAX
WRITE_TIMEOUT_MAX_US = 1_000_000
POLICY = ('v1:module-context-owned-buffer,max16-firmware-wide,buffer-length1-25,filter-max8,'
          'cleanup-after-worker-join,shared-sensor-with-lua,no-retained-module-pointers,'
          'token-revalidated-by-context,zero-skipped-live-token-collision-scan,'
          'owned-frame-data64,dlc-and-canfd-validated,'
          'write-timeout-max1000000us')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _method_shape(method):
    return (_field_shape(method['return']), tuple(_field_shape(arg) for arg in method['arguments']),
            method['rename'], method['depends'], method['deprecate'])


def _value_metadata(owner):
    keys = ('rename', 'depends', 'get', 'literal', 'reference', 'semaphore',
            'scheduler_semaphore', 'semaphore_pointer', 'creation', 'creation_args', 'creation_manual',
            'has_fields', 'has_enums', 'operations', 'operator_getter', 'operators_manual')
    return tuple(owner[key] for key in keys) + (tuple(owner['operator_names']),)


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = dict(rename=None, depends=DEPENDS, get=None, literal=False, reference=False,
                          semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    expected_aliases = {
        'get_device': ('lua_get_CAN_device', None, True),
        'get_device2': ('lua_get_CAN_device2', None, True),
    }
    if (owner is None or any(owner[key] != value for key, value in expected_owner.items()) or
            owner['methods'] or owner['fields'] or
            {item['alias']: (item['name'], item['depends'], item['manual'])
             for item in owner['aliases']} != expected_aliases or len(owner['aliases']) != len(expected_aliases)):
        raise ValueError('unaudited CAN owner metadata')

    frame = next((item for item in ir['userdata'] if item['name'] == FRAME), None)
    expected_frame_metadata = ('CANFrame', DEPENDS, None, False, False, False, False, False,
                               None, 0, False, True, False, 0, None, False, ())
    expected_fields = {
        'id': (('uint32_t', None, False, False, False, True, None), None),
        'data': (('uint8_t', None, False, False, False, True, None), 'int(ARRAY_SIZE(ud->data))'),
        'dlc': (('uint8_t', None, False, False, False, False,
                 ('0', 'int(ARRAY_SIZE(ud->data))')), None),
    }
    actual_fields = {
        field['name']: ((_field_shape(field['type'])), field['array_len'])
        for field in frame['fields']
    } if frame is not None else {}
    if (frame is None or _value_metadata(frame) != expected_frame_metadata or frame['aliases'] or
            len(frame['fields']) != len(expected_fields) or actual_fields != expected_fields or
            any(not field['read'] or not field['write'] or field['rename'] is not None or
                field['depends'] is not None or field['valid_mask_name'] is not None or
                field['valid_mask_value'] is not None for field in frame['fields'])):
        raise ValueError('unaudited CAN frame fields')
    expected_frame_methods = {
        'id_signed': (('int32_t', None, False, False, False, False, None), (), None, None, None),
        'isExtended': (('boolean', None, False, False, False, False, None), (), None, None, None),
        'isRemoteTransmissionRequest': (
            ('boolean', None, False, False, False, False, None), (), None, None, None),
        'isErrorFrame': (('boolean', None, False, False, False, False, None), (), None, None, None),
    }
    actual_frame_methods = {method['name']: _method_shape(method) for method in frame['methods']}
    if len(frame['methods']) != len(expected_frame_methods) or actual_frame_methods != expected_frame_methods:
        raise ValueError('unaudited CAN frame methods')

    buffer = next((item for item in ir['ap_objects'] if item['name'] == BUFFER), None)
    expected_buffer_metadata = (None, DEPENDS, None, False, False, False, False, False,
                                None, 0, False, False, False, 0, None, False, ())
    if (buffer is None or _value_metadata(buffer) != expected_buffer_metadata or buffer['aliases'] or
            buffer['fields']):
        raise ValueError('unaudited CAN buffer metadata')
    boolean = ('boolean', None, False, False, False, False, None)
    frame_value = ('userdata', FRAME, False, False, False, False, None)
    expected_buffer_methods = {
        'write_frame': (boolean, (frame_value, ('uint32_t', None, False, False, False, True, None)),
                        None, None, None),
        'read_frame': (boolean, (('userdata', FRAME, True, False, False, False, None),),
                       None, None, None),
        'add_filter': (boolean, (('uint32_t', None, False, False, False, True, None),
                                 ('uint32_t', None, False, False, False, True, None)), None, None, None),
    }
    actual_buffer_methods = {method['name']: _method_shape(method) for method in buffer['methods']}
    if len(buffer['methods']) != len(expected_buffer_methods) or actual_buffer_methods != expected_buffer_methods:
        raise ValueError('unaudited CAN buffer methods')
    return specification(first_id)


def specification(first_id):
    handle = dict(type='u32-le,nonzero', direction='module-owned-receiver')
    frame = dict(name='CanFrame', fields=[['id', 'u32-le'], ['dlc', 'u8'], ['can_fd', 'bool-u8'],
                                          ['data', f'bytes{FRAME_DATA_MAX}']])
    methods = [
        dict(id=first_id, name='CAN.get_device',
             arguments=[dict(type='u8', range=[1, BUFFER_LENGTH_MAX], name='buffer_length')],
             result=dict(presence='u32-le,zero-is-none', handle='CanBuffer'), policy=POLICY),
        dict(id=first_id + 1, name='CAN.get_device2',
             arguments=[dict(type='u8', range=[1, BUFFER_LENGTH_MAX], name='buffer_length')],
             result=dict(presence='u32-le,zero-is-none', handle='CanBuffer'), policy=POLICY),
        dict(id=first_id + 2, name='CanBuffer.read_frame', arguments=[handle],
             result=dict(presence='bool-u8', frame=frame), policy=POLICY),
        dict(id=first_id + 3, name='CanBuffer.write_frame',
             arguments=[handle, dict(type=frame, direction='in'),
                        dict(type='u32-le', name='timeout_us', range=[0, WRITE_TIMEOUT_MAX_US])],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 4, name='CanBuffer.add_filter',
             arguments=[handle, dict(type='u32-le', name='mask'), dict(type='u32-le', name='value')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 5, name='CanBuffer.close', arguments=[handle], result='void', policy=POLICY),
    ]
    return dict(methods=methods)


def rust(spec, notice):
    get, get2, read, write, add_filter, close = spec['methods']
    return notice + render(
        'rust/can.rs',
        frame_data_max=FRAME_DATA_MAX,
        buffer_length_max=BUFFER_LENGTH_MAX,
        write_timeout_max_us=WRITE_TIMEOUT_MAX_US,
        frame_wire_size=FRAME_WIRE_SIZE,
        get_id=get['id'],
        get2_id=get2['id'],
        frame_wire_size_plus_1=FRAME_WIRE_SIZE + 1,
        read_id=read['id'],
        n4_plus_frame_wire_size_plus_4=4 + FRAME_WIRE_SIZE + 4,
        n4_plus_frame_wire_size=4 + FRAME_WIRE_SIZE,
        write_id=write['id'],
        add_filter_id=add_filter['id'],
        close_id=close['id'],
    )
