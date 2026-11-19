# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned EFI state and revalidated scripting-backend handle."""

from call_model import U8
from call_model import Call
from call_model import Field
from call_model import Handle
from call_model import Module
from call_model import SelectedHandle
from native_templates import render

STATE_WIRE_SIZE = 101
OWNER = 'AP_EFI'
BACKEND = 'AP_EFI_Backend'
STATE = 'EFI_State'
CYLINDER = 'Cylinder_Status'
DEPENDS = '(AP_EFI_SCRIPTING_ENABLED == 1)'
POLICY = ('v1:complete-owned-copy,no-retained-firmware-pointers,backend-index-revalidated-each-call,'
          'optional-f32-none-is-nan,ignition-voltage-none-is-minus-one,enum-ranges-validated')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _method_shape(method):
    return (_field_shape(method['return']), tuple(_field_shape(arg) for arg in method['arguments']),
            method['rename'], method['depends'], method['deprecate'])


def _owner_metadata(owner, *, fields=False):
    keys = ('rename', 'depends', 'get', 'literal', 'reference', 'semaphore',
            'scheduler_semaphore', 'semaphore_pointer')
    result = tuple(owner[key] for key in keys)
    if fields:
        result += (owner['creation'], owner['creation_args'], owner['creation_manual'], owner['has_fields'],
                   owner['has_enums'], owner['operations'], owner['operator_getter'], owner['operators_manual'],
                   tuple(owner['operator_names']))
    return result


def _userdata_fields(owner):
    return {
        field['name']: (_field_shape(field['type']), field['read'], field['write'], field['rename'],
                        field['depends'], field['array_len'], field['valid_mask_name'], field['valid_mask_value'])
        for field in owner['fields']
    }


def select(ir, first_id):
    """Select EFI only when the audited Lua owner and value shapes match exactly."""
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = ('efi', DEPENDS, None, False, False, False, False, False)
    if (owner is None or _owner_metadata(owner) != expected_owner or owner['aliases'] or owner['fields']):
        raise ValueError('unaudited EFI owner metadata')
    expected_owner_methods = {
        'get_backend': ((('ap_object', BACKEND, False, False, False, False, None),
                         (('uint8_t', None, False, False, False, True, None),), None, None, None)),
        'get_state': ((('void', None, False, False, False, False, None),
                       (('userdata', STATE, False, True, False, False, None),), None, None, None)),
        'get_last_update_ms': ((('uint32_t', None, False, False, False, False, None),
                                (), None, None, None)),
    }
    actual_owner_methods = {method['name']: _method_shape(method) for method in owner['methods']}
    if len(owner['methods']) != len(expected_owner_methods) or actual_owner_methods != expected_owner_methods:
        raise ValueError('unaudited EFI owner methods')

    backend = next((item for item in ir['ap_objects'] if item['name'] == BACKEND), None)
    expected_value_metadata = (None, DEPENDS, None, False, False, False, False, False,
                               None, 0, False, False, False, 0, None, False, ())
    if (backend is None or _owner_metadata(backend, fields=True) != expected_value_metadata or
            backend['aliases'] or backend['fields']):
        raise ValueError('unaudited EFI backend metadata')
    expected_backend_methods = {
        'handle_scripting': (('boolean', None, False, False, False, False, None),
                             (('userdata', STATE, False, False, False, False, None),), None, None, None),
    }
    actual_backend_methods = {method['name']: _method_shape(method) for method in backend['methods']}
    if len(backend['methods']) != len(expected_backend_methods) or actual_backend_methods != expected_backend_methods:
        raise ValueError('unaudited EFI backend methods')

    common_float = (('float', None, False, False, False, True, None), True, True, None, None, None, None, None)
    expected_cylinder_fields = {name: common_float for name in (
        'ignition_timing_deg', 'injection_time_ms', 'cylinder_head_temperature',
        'cylinder_head_temperature2', 'exhaust_gas_temperature', 'exhaust_gas_temperature2',
        'lambda_coefficient')}
    common_value_metadata = (None, DEPENDS, None, False, False, False, False, False,
                             None, 0, False, True, False, 0, None, False, ())
    cylinder = next((item for item in ir['userdata'] if item['name'] == CYLINDER), None)
    if (cylinder is None or _owner_metadata(cylinder, fields=True) != common_value_metadata or
            cylinder['aliases'] or cylinder['methods'] or len(cylinder['fields']) != len(expected_cylinder_fields) or
            _userdata_fields(cylinder) != expected_cylinder_fields):
        raise ValueError('unaudited EFI cylinder state')

    expected_state_fields = {
        name: common_float for name in (
            'spark_dwell_time_ms', 'atmospheric_pressure_kpa', 'intake_manifold_pressure_kpa',
            'intake_manifold_temperature', 'coolant_temperature', 'oil_pressure', 'oil_temperature',
            'fuel_pressure', 'fuel_consumption_rate_cm3pm', 'estimated_consumed_fuel_volume_cm3',
            'ignition_voltage', 'throttle_out', 'pt_compensation')
    }
    for name in ('last_updated_ms', 'engine_speed_rpm'):
        expected_state_fields[name] = (('uint32_t', None, False, False, False, True, None),
                                       True, True, None, None, None, None, None)
    for name in ('engine_load_percent', 'throttle_position_percent', 'ecu_index'):
        expected_state_fields[name] = (('uint8_t', None, False, False, False, True, None),
                                       True, True, None, None, None, None, None)
    expected_state_fields['general_error'] = (('boolean', None, False, False, False, False, None),
                                              True, True, None, None, None, None, None)
    expected_state_fields['fuel_pressure_status'] = (
        ('enum', 'Fuel_Pressure_Status', False, False, False, False,
         ('Fuel_Pressure_Status::NOT_SUPPORTED', 'Fuel_Pressure_Status::ABOVE_NOMINAL')),
        True, True, None, None, None, None, None)
    expected_state_fields['cylinder_status'] = (('userdata', CYLINDER, False, False, False, False, None),
                                                True, True, None, None, None, None, None)
    state = next((item for item in ir['userdata'] if item['name'] == STATE), None)
    if (state is None or _owner_metadata(state, fields=True) != common_value_metadata or
            state['aliases'] or state['methods'] or len(state['fields']) != len(expected_state_fields) or
            _userdata_fields(state) != expected_state_fields):
        raise ValueError('unaudited EFI state')
    return specification(first_id)


def specification(first_id):
    cylinder = dict(name='EfiCylinderState', fields=[
        ['ignition_timing_deg', 'optional-f32-le,nan-unavailable'],
        ['injection_time_ms', 'optional-f32-le,nan-unavailable'],
        ['cylinder_head_temperature', 'optional-f32-le,nan-unavailable'],
        ['cylinder_head_temperature2', 'optional-f32-le,nan-unavailable'],
        ['exhaust_gas_temperature', 'optional-f32-le,nan-unavailable'],
        ['exhaust_gas_temperature2', 'optional-f32-le,nan-unavailable'],
        ['lambda_coefficient', 'optional-f32-le,nan-unavailable'],
    ])
    state = dict(name='EfiState', fields=[
        ['last_updated_ms', 'u32-le'], ['engine_state', 'EngineState-u8'], ['general_error', 'bool-u8'],
        ['crankshaft_sensor_status', 'CrankshaftSensorStatus-u8'],
        ['temperature_status', 'TemperatureStatus-u8'], ['fuel_pressure_status', 'FuelPressureStatus-u8'],
        ['oil_pressure_status', 'OilPressureStatus-u8'], ['detonation_status', 'DetonationStatus-u8'],
        ['misfire_status', 'MisfireStatus-u8'], ['debris_status', 'DebrisStatus-u8'],
        ['engine_load_percent', 'u8'], ['engine_speed_rpm', 'u32-le'],
        ['spark_dwell_time_ms', 'optional-f32-le,nan-unavailable'],
        ['atmospheric_pressure_kpa', 'optional-f32-le,nan-unavailable'],
        ['intake_manifold_pressure_kpa', 'optional-f32-le,nan-unavailable'],
        ['intake_manifold_temperature', 'optional-f32-le,nan-unavailable'],
        ['coolant_temperature', 'optional-f32-le,nan-unavailable'],
        ['oil_pressure', 'optional-f32-le,nan-unavailable'],
        ['oil_temperature', 'optional-f32-le,nan-unavailable'],
        ['fuel_pressure', 'optional-f32-le,nan-unavailable'],
        ['fuel_consumption_rate_cm3pm', 'optional-f32-le,nan-unavailable'],
        ['estimated_consumed_fuel_volume_cm3', 'optional-f32-le,nan-unavailable'],
        ['throttle_position_percent', 'u8'], ['ecu_index', 'u8'], ['spark_plug_usage', 'SparkPlugUsage-u8'],
        ['cylinder_status', cylinder], ['ignition_voltage', 'optional-f32-le,minus-one-unavailable'],
        ['throttle_out', 'optional-f32-le,nan-unavailable'],
        ['pt_compensation', 'optional-f32-le,nan-unavailable'],
    ])
    get_backend = dict(id=first_id, name='efi.get_backend',
                       arguments=[dict(type='u8', direction='in')],
                       result=dict(presence='bool-u8', handle='EfiBackend,index-revalidated'), policy=POLICY)
    handle = dict(id=first_id + 1, name='EfiBackend.handle_scripting',
                  arguments=[dict(type='u8', direction='revalidated-backend-index'),
                             dict(type=state, direction='in')], result='bool-u8', policy=POLICY)
    get_state = dict(id=first_id + 2, name='efi.get_state', arguments=[],
                     result=dict(type=state, direction='owned-out'), policy=POLICY)
    return dict(methods=[get_backend, handle, get_state])


def client_model(spec):
    method = spec['methods'][0]
    backend = Handle('EfiBackend', 'borrowed-index')
    return Module((Call(method['name'], method['id'], (Field('index', U8),),
                        SelectedHandle(backend, 'index')),), (backend,))


def rust(spec, notice):
    get_backend, handle, get_state = spec['methods']
    return notice + render(
        'rust/efi.rs',
        state_wire_size=STATE_WIRE_SIZE,
        get_backend_id=get_backend['id'],
        get_state_id=get_state['id'],
        state_wire_size_plus_1=STATE_WIRE_SIZE + 1,
        handle_id=handle['id'],
    )
