# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned rangefinder state and revalidated rangefinder/proximity backends."""

import rust_calls

from call_model import BOOL
from call_model import F32
from call_model import I8
from call_model import I32
from call_model import U8
from call_model import U16
from call_model import U32
from call_model import Call
from call_model import Enum
from call_model import Field
from call_model import Handle
from call_model import Module
from call_model import Projection
from call_model import Record
from call_model import SelectedHandle

RANGEFINDER_OWNER = 'RangeFinder'
RANGEFINDER_BACKEND = 'AP_RangeFinder_Backend'
RANGEFINDER_STATE = 'RangeFinder::RangeFinder_State'
PROXIMITY_OWNER = 'AP_Proximity'
PROXIMITY_BACKEND = 'AP_Proximity_Backend'
RANGEFINDER_DEPENDS = 'AP_RANGEFINDER_ENABLED'
PROXIMITY_DEPENDS = 'HAL_PROXIMITY_ENABLED == 1'
STATE_WIRE_SIZE = 13
POLICY = ('v1:non-owning-backend-index,revalidate-every-call,no-retained-firmware-pointers,'
          'complete-owned-rangefinder-state,enum-and-bool-validation,all-f32-bits,'
          'native-backend-return,no-lifecycle-rollback')


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


def _shape(result, arguments=(), rename=None):
    return (result, tuple(arguments), rename, None, None)


def select(ir, first_id):
    """Fail closed unless the audited Lua owners and method shapes are unchanged."""
    rangefinder = next((item for item in ir['singletons'] if item['name'] == RANGEFINDER_OWNER), None)
    expected_rangefinder_metadata = ('rangefinder', RANGEFINDER_DEPENDS, None, False, False, False, False, False)
    rangefinder_methods = {
        'distance_cm_orient', 'distance_orient', 'get_backend', 'get_pos_offset_orient',
        'ground_clearance_cm_orient', 'ground_clearance_orient', 'has_data_orient', 'has_orientation',
        'max_distance_cm_orient', 'max_distance_orient', 'min_distance_cm_orient', 'min_distance_orient',
        'num_sensors', 'signal_quality_pct_orient', 'status_orient',
    }
    if (rangefinder is None or _owner_metadata(rangefinder) != expected_rangefinder_metadata or
            rangefinder['aliases'] or rangefinder['fields'] or
            {method['name'] for method in rangefinder['methods']} != rangefinder_methods):
        raise ValueError('unaudited RangeFinder owner metadata')
    expected_get_backend = _shape(
        ('ap_object', RANGEFINDER_BACKEND, False, False, False, False, None),
        (('uint8_t', None, False, False, False, True, None),))
    actual_get_backend = next(method for method in rangefinder['methods'] if method['name'] == 'get_backend')
    if _method_shape(actual_get_backend) != expected_get_backend:
        raise ValueError('unaudited RangeFinder get_backend')

    object_metadata = (None, RANGEFINDER_DEPENDS, None, False, False, False, False, False,
                       None, 0, False, False, False, 0, None, False, ())
    rangefinder_backend = next(
        (item for item in ir['ap_objects'] if item['name'] == RANGEFINDER_BACKEND), None)
    expected_backend_names = {'distance', 'get_state', 'orientation', 'signal_quality_pct', 'status', 'type'}
    if (rangefinder_backend is None or _owner_metadata(rangefinder_backend, fields=True) != object_metadata or
            rangefinder_backend['fields'] or
            {method['name'] for method in rangefinder_backend['methods']} != expected_backend_names):
        raise ValueError('unaudited RangeFinder backend metadata')
    expected_backend_methods = {
        'distance': _shape(('float', None, False, False, False, False, None)),
        'get_state': _shape(
            ('void', None, False, False, False, False, None),
            (('userdata', RANGEFINDER_STATE, False, True, False, False, None),)),
        'signal_quality_pct': _shape(
            ('float', None, False, False, False, False, None), rename='signal_quality'),
        'type': _shape(('uint8_t', None, False, False, False, False, None)),
        'status': _shape(('uint8_t', None, False, False, False, False, None)),
        'orientation': _shape(('enum', 'Rotation', False, False, False, False, None)),
    }
    actual_backend_methods = {
        method['name']: _method_shape(method) for method in rangefinder_backend['methods']
        if method['name'] in expected_backend_methods
    }
    if actual_backend_methods != expected_backend_methods:
        raise ValueError('unaudited RangeFinder backend methods')
    expected_aliases = [dict(name='lua_range_finder_handle_script_msg', alias='handle_script_msg',
                             depends=None, manual=True)]
    if rangefinder_backend['aliases'] != expected_aliases:
        raise ValueError('unaudited RangeFinder handle_script_msg')

    state_metadata = ('RangeFinder_State', RANGEFINDER_DEPENDS, None, False, False, False, False, False,
                      None, 0, False, True, False, 0, None, False, ())
    state = next((item for item in ir['userdata'] if item['name'] == RANGEFINDER_STATE), None)
    expected_state_fields = {
        'distance_m': (('float', None, False, False, False, True, None),
                       True, True, 'distance', None, None, None, None),
        'signal_quality_pct': (('int8_t', None, False, False, False, True, None),
                               True, True, 'signal_quality', None, None, None, None),
        'voltage_mv': (('uint16_t', None, False, False, False, True, None),
                       True, True, 'voltage', None, None, None, None),
        'status': (('enum', 'RangeFinder::Status', False, False, False, False,
                    ('RangeFinder::Status::NotConnected', 'RangeFinder::Status::PoweredDown')),
                   True, True, None, None, None, None, None),
        'range_valid_count': (('uint8_t', None, False, False, False, True, None),
                              True, True, None, None, None, None, None),
        'last_reading_ms': (('uint32_t', None, False, False, False, True, None),
                            True, True, 'last_reading', None, None, None, None),
    }
    if (state is None or _owner_metadata(state, fields=True) != state_metadata or state['aliases'] or
            state['methods'] or _userdata_fields(state) != expected_state_fields):
        raise ValueError('unaudited RangeFinder state')

    proximity = next((item for item in ir['singletons'] if item['name'] == PROXIMITY_OWNER), None)
    expected_proximity_metadata = ('proximity', PROXIMITY_DEPENDS, None, False, False, False, False, False)
    proximity_methods = {
        'get_backend', 'get_closest_object', 'get_object_angle_and_distance', 'get_object_count',
        'get_status', 'num_sensors',
    }
    if (proximity is None or _owner_metadata(proximity) != expected_proximity_metadata or
            proximity['aliases'] or proximity['fields'] or
            {method['name'] for method in proximity['methods']} != proximity_methods):
        raise ValueError('unaudited Proximity owner metadata')
    expected_get_backend = _shape(
        ('ap_object', PROXIMITY_BACKEND, False, False, False, False, None),
        (('uint8_t', None, False, False, False, True, None),))
    actual_get_backend = next(method for method in proximity['methods'] if method['name'] == 'get_backend')
    if _method_shape(actual_get_backend) != expected_get_backend:
        raise ValueError('unaudited Proximity get_backend')

    proximity_object_metadata = (None, PROXIMITY_DEPENDS, None, False, False, False, False, False,
                                 None, 0, False, False, False, 0, None, False, ())
    proximity_backend = next((item for item in ir['ap_objects'] if item['name'] == PROXIMITY_BACKEND), None)
    if (proximity_backend is None or
            _owner_metadata(proximity_backend, fields=True) != proximity_object_metadata or
            proximity_backend['aliases'] or proximity_backend['fields']):
        raise ValueError('unaudited Proximity backend metadata')
    boolean = ('boolean', None, False, False, False, False, None)
    float_skip = ('float', None, False, False, False, True, None)
    expected_proximity_methods = {
        'handle_script_distance_msg': _shape(boolean, (float_skip, float_skip, float_skip, boolean)),
        'handle_script_3d_msg': _shape(
            boolean, (('userdata', 'Vector3f', False, False, False, False, None), boolean)),
        'type': _shape(('uint8_t', None, False, False, False, False, None)),
        'set_distance_min_max': _shape(boolean, (float_skip, float_skip)),
        'update_virtual_boundary': _shape(boolean),
    }
    actual_proximity_methods = {
        method['name']: _method_shape(method) for method in proximity_backend['methods']
    }
    if actual_proximity_methods != expected_proximity_methods:
        raise ValueError('unaudited Proximity backend methods')
    return specification(first_id)


def specification(first_id):
    rangefinder_handle = dict(type='u8,revalidated-rangefinder-backend-index',
                              direction='borrowed-receiver')
    proximity_handle = dict(type='u8,revalidated-proximity-backend-index',
                            direction='borrowed-receiver')
    optional_rangefinder = dict(presence='bool-u8', handle='RangeFinderBackend,index-revalidated')
    optional_proximity = dict(presence='bool-u8', handle='ProximityBackend,index-revalidated')
    state = dict(name='RangeFinderState', fields=[
        ['distance_m', 'f32-le,all-bits'], ['signal_quality_pct', 'i8'], ['voltage_mv', 'u16-le'],
        ['status', 'RangeFinderStatus-u8,range0-5'], ['range_valid_count', 'u8'],
        ['last_reading_ms', 'u32-le'],
    ])
    methods = [
        dict(id=first_id, name='RangeFinder.get_backend',
             arguments=[dict(type='u8', direction='in', name='index')],
             result=optional_rangefinder, policy=POLICY),
        dict(id=first_id + 1, name='RangeFinderBackend.type', arguments=[rangefinder_handle],
             result='u8', policy=POLICY),
        dict(id=first_id + 2, name='RangeFinderBackend.status', arguments=[rangefinder_handle],
             result='RangeFinderStatus-u8,range0-5', policy=POLICY),
        dict(id=first_id + 3, name='RangeFinderBackend.handle_script_distance',
             arguments=[rangefinder_handle, dict(type='f32-le', direction='in', float_input='all-f32-bits')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 4, name='RangeFinderBackend.handle_script_state',
             arguments=[rangefinder_handle, dict(type=state, direction='in')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 5, name='Proximity.get_backend',
             arguments=[dict(type='u8', direction='in', name='index')],
             result=optional_proximity, policy=POLICY),
        dict(id=first_id + 6, name='ProximityBackend.type', arguments=[proximity_handle],
             result='u8', policy=POLICY),
        dict(id=first_id + 7, name='ProximityBackend.handle_script_distance_msg',
             arguments=[proximity_handle,
                        dict(type='f32-le', direction='in', name='distance_m', float_input='all-f32-bits'),
                        dict(type='f32-le', direction='in', name='yaw_deg', float_input='all-f32-bits'),
                        dict(type='f32-le', direction='in', name='pitch_deg', float_input='all-f32-bits'),
                        dict(type='bool-u8', direction='in', name='push_to_boundary')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 8, name='ProximityBackend.set_distance_min_max',
             arguments=[proximity_handle,
                        dict(type='f32-le', direction='in', name='minimum', float_input='all-f32-bits'),
                        dict(type='f32-le', direction='in', name='maximum', float_input='all-f32-bits')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 9, name='ProximityBackend.update_virtual_boundary',
             arguments=[proximity_handle], result='bool-u8', policy=POLICY),
        dict(id=first_id + 10, name='RangeFinderBackend.get_state',
             arguments=[rangefinder_handle], result=state, policy=POLICY),
        dict(id=first_id + 11, name='RangeFinderBackend.orientation',
             arguments=[rangefinder_handle], result='Rotation-i32-le', policy=POLICY),
        dict(id=first_id + 12, name='ProximityBackend.handle_script_3d_msg',
             arguments=[proximity_handle, dict(type='Vector3f,three-f32-le', name='vector'),
                        dict(type='bool-u8', direction='in', name='push_to_boundary')],
             result='bool-u8', policy=POLICY),
    ]
    return dict(methods=methods, state_wire_size=STATE_WIRE_SIZE)


def client_model(spec):
    status = Enum('RangeFinderStatus', U8, (
        ('NotConnected', 0), ('NoData', 1), ('OutOfRangeLow', 2),
        ('OutOfRangeHigh', 3), ('Good', 4), ('PoweredDown', 5)), default='NotConnected')
    state = Record('RangeFinderState', (
        Field('distance_m', F32), Field('signal_quality_pct', I8), Field('voltage_mv', U16),
        Field('status', status), Field('range_valid_count', U8), Field('last_reading_ms', U32)))
    vector = Record('Vector3f', tuple(Field(name, F32) for name in ('x', 'y', 'z')), external=True)
    rotation = Enum('Rotation', I32, external=True)
    rangefinder = Handle('RangeFinderBackend', 'borrowed-index')
    proximity = Handle('ProximityBackend', 'borrowed-index')

    def floats(*names):
        return tuple(Field(name, F32) for name in names)

    descriptions = {
        'RangeFinder.get_backend': ((Field('index', U8),), SelectedHandle(rangefinder, 'index'), None, '', 'rangefinder'),
        'RangeFinderBackend.type': ((), U8, rangefinder, 'sensor_type', ''),
        'RangeFinderBackend.status': ((), status, rangefinder, '', ''),
        'RangeFinderBackend.handle_script_distance': (floats('distance_m'), BOOL, rangefinder, '', ''),
        'RangeFinderBackend.handle_script_state': ((Field('state', state),), BOOL, rangefinder, '', ''),
        'RangeFinderBackend.get_state': ((), state, rangefinder, '', ''),
        'RangeFinderBackend.orientation': ((), rotation, rangefinder, '', ''),
        'Proximity.get_backend': ((Field('index', U8),), SelectedHandle(proximity, 'index'), None, '', 'proximity'),
        'ProximityBackend.type': ((), U8, proximity, 'sensor_type', ''),
        'ProximityBackend.handle_script_distance_msg': (
            floats('distance_m', 'yaw_deg', 'pitch_deg') + (Field('push_to_boundary', BOOL),), BOOL, proximity, '', ''),
        'ProximityBackend.handle_script_3d_msg': (
            (Field('vector', vector), Field('push_to_boundary', BOOL)), BOOL, proximity, '', ''),
        'ProximityBackend.set_distance_min_max': (floats('minimum', 'maximum'), BOOL, proximity, '', ''),
        'ProximityBackend.update_virtual_boundary': ((), BOOL, proximity, '', ''),
    }
    calls = []
    for method in spec['methods']:
        arguments, result, receiver, name, namespace = descriptions[method['name']]
        calls.append(Call(method['name'], method['id'], arguments, result, receiver=receiver,
                          method_name=name, namespace=namespace))
    projections = (Projection(rangefinder.name, 'distance', 'get_state', 'distance_m', F32),
                   Projection(rangefinder.name, 'signal_quality', 'get_state', 'signal_quality_pct', F32))
    return Module(tuple(calls), (status, state, vector, rotation, rangefinder, proximity), projections)


def rust(spec, notice):
    return rust_calls.emit(client_model(spec), notice)
