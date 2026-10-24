# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned adapters for script-fed ESC data and measured servo position."""

from native_templates import render

ESC_OWNER = 'AP_ESC_Telem'
ESC_VALUE = 'AP_ESC_Telem_Backend::TelemetryData'
SERVO_OWNER = 'AP_Servo_Telem'
SERVO_VALUE = 'AP_Servo_Telem::TelemetryData'
ESC_DEPENDS = 'HAL_WITH_ESC_TELEM == 1'
SERVO_DEPENDS = 'AP_SERVO_TELEM_ENABLED'
ESC_WIRE_SIZE = 12
SERVO_WIRE_SIZE = 6
SERVO_UPDATE_WIRE_SIZE = 33
POLICY = ('v1:owned-value-copy,esc-options-map-to-temperature-voltage-current-validity-bits,'
          'esc-nonempty-and-index-bounded,servo-overall-presence-plus-measured-position-validity,'
          'all-f32-bits-preserved,no-retained-firmware-pointers')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _method_shape(method):
    return (_field_shape(method['return']), tuple(_field_shape(arg) for arg in method['arguments']),
            method['rename'], method['depends'], method['deprecate'])


def _owner(ir, group, name, rename, depends):
    owner = next((item for item in ir[group] if item['name'] == name), None)
    expected = dict(rename=rename, depends=depends, get=None, literal=False, reference=False,
                    semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()) or owner['aliases']:
        raise ValueError(f'unaudited {name} metadata')
    return owner


def _method(owner, name, expected):
    methods = [method for method in owner['methods'] if method['name'] == name]
    if len(methods) != 1 or _method_shape(methods[0]) != expected:
        raise ValueError(f'unaudited {owner["name"]}.{name} method')


def _field(owner, name, expected):
    fields = [field for field in owner['fields'] if field['name'] == name]
    if len(fields) != 1:
        raise ValueError(f'unaudited {owner["name"]}.{name} field')
    field = fields[0]
    actual = (_field_shape(field['type']), field['read'], field['write'], field['rename'], field['depends'],
              field['array_len'], field['valid_mask_name'], field['valid_mask_value'])
    if actual != expected:
        raise ValueError(f'unaudited {owner["name"]}.{name} field')


def select(ir, first_id):
    esc = _owner(ir, 'singletons', ESC_OWNER, 'esc_telem', ESC_DEPENDS)
    _method(esc, 'update_telem_data', (
        ('void', None, False, False, False, False, None),
        (('uint8_t', None, False, False, False, False, ('0', 'ESC_TELEM_MAX_ESCS')),
         ('userdata', ESC_VALUE, False, False, False, False, None),
         ('uint16_t', None, False, False, False, True, None)), None, None, None))
    esc_value = _owner(ir, 'userdata', ESC_VALUE, 'ESCTelemetryData', f'({ESC_DEPENDS})')
    write_only = (False, True, None, None, None, None, None)
    _field(esc_value, 'temperature_cdeg',
           (('int16_t', None, False, False, False, True, None),) + write_only)
    for name in ('voltage', 'current'):
        _field(esc_value, name, (('float', None, False, False, False, True, None),) + write_only)

    servo = _owner(ir, 'singletons', SERVO_OWNER, 'servo_telem', SERVO_DEPENDS)
    _method(servo, 'get_telem', (
        ('boolean', None, False, False, False, False, None),
        (('uint8_t', None, False, False, False, True, None),
         ('userdata', SERVO_VALUE, True, False, False, False, None)), None, None, None))
    _method(servo, 'update_telem_data', (
        ('void', None, False, False, False, False, None),
        (('uint8_t', None, False, False, False, True, None),
         ('userdata', SERVO_VALUE, False, False, False, False, None)), None, None, None))
    servo_value = _owner(ir, 'userdata', SERVO_VALUE, 'ServoTelemetryData', SERVO_DEPENDS)
    servo_fields = {
        'command_position': ('float', 'COMMANDED_POSITION'),
        'measured_position': ('float', 'MEASURED_POSITION'),
        'force': ('float', 'FORCE'),
        'speed': ('float', 'SPEED'),
        'voltage': ('float', 'VOLTAGE'),
        'current': ('float', 'CURRENT'),
        'duty_cycle': ('uint8_t', 'DUTY_CYCLE'),
        'motor_temperature_cdeg': ('int16_t', 'MOTOR_TEMP'),
        'pcb_temperature_cdeg': ('int16_t', 'PCB_TEMP'),
        'status_flags': ('uint8_t', 'STATUS'),
    }
    for name, (kind, mask) in servo_fields.items():
        _field(servo_value, name, (
            (kind, None, False, False, False, True, None), True, True, None, None, None,
            'present_types', f'AP_Servo_Telem::TelemetryData::Types::{mask}'))
    _field(servo_value, 'last_update_ms', (
        ('uint32_t', None, False, False, False, True, None), True, False, None, None, None, None, None))
    return specification(first_id)


def specification(first_id):
    esc_value = dict(name='EscTelemetryUpdate', fields=[
        ['temperature_cdeg', 'optional-i16-le,presence-bit0'],
        ['voltage', 'optional-f32-le,presence-bit1'],
        ['current', 'optional-f32-le,presence-bit2'],
    ])
    servo_value = dict(name='ServoTelemetryData', fields=[
        ['measured_position', 'optional-f32-le,present-types-bit1'],
    ])
    return dict(methods=[
        dict(id=first_id, name='esc_telem.update_telem_data',
             arguments=[dict(type='u8', name='esc_index'), dict(type=esc_value, direction='in')],
             result='void', policy=POLICY),
        dict(id=first_id + 1, name='servo_telem.get_telem',
             arguments=[dict(type='u8', name='servo_index')],
             result=dict(presence='bool-u8', value=servo_value), policy=POLICY),
        dict(id=first_id + 2, name='servo_telem.update_telem_data',
             arguments=[dict(type='u8', name='servo_index'),
                        dict(type='ServoTelemetryUpdate,optional-fields-mask-u16', direction='in')],
             result='void', policy=POLICY),
    ])


def rust(spec, notice):
    esc, servo, servo_update = spec['methods']
    return notice + render(
        'rust/telemetry.rs',
        esc_wire_size=ESC_WIRE_SIZE,
        esc_id=esc['id'],
        servo_wire_size=SERVO_WIRE_SIZE,
        servo_id=servo['id'],
        servo_update_wire_size=SERVO_UPDATE_WIRE_SIZE,
        servo_update_id=servo_update['id'],
    )
