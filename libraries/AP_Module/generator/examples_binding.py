# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Small manual bindings exercised by the AP_Scripting examples."""

from native_templates import render

VEHICLE_OWNER = 'AP_Vehicle'
SRV_OWNER = 'SRV_Channels'
VELOCITY_ALIAS = 'set_target_velocity_NED'
SAFETY_ALIAS = 'get_safety_state'
MISSION_GLOBAL = 'mission_receive'
FLEXDEBUG_GLOBAL = 'DroneCAN_get_FlexDebug'
FLEXDEBUG_PAYLOAD_MAX = 255
MISSION_WIRE_SIZE = 1 + 4 + 2 + 3 * 4
FLEXDEBUG_WIRE_SIZE = 1 + 4 + 1 + FLEXDEBUG_PAYLOAD_MAX

VELOCITY_POLICY = ('v1:owned-vector3f-input,optional-align-yaw,blocking-scheduler-lock,'
                   'shared-guided-target-state,no-retained-module-pointers')
SAFETY_POLICY = 'v1:safety-switch-state-snapshot,no-retained-module-pointers'
MISSION_POLICY = ('v1:shared-scripting-mission-command-queue,destructive-oldest-pop,queue-depth5,'
                  'complete-owned-copy,no-retained-module-pointers,zero-on-none')
FLEXDEBUG_POLICY = ('v1:dronecan-driver-index-revalidated-each-call,node-and-message-keyed-latest,'
                    'caller-timestamp-suppresses-stale,payload-max255,complete-owned-copy,'
                    'no-retained-module-pointers,zero-on-none')


def _owner(ir, name):
    owner = next((item for item in ir['singletons'] if item['name'] == name), None)
    if owner is None:
        raise ValueError(f'missing {name} singleton')
    return owner


def _manual_alias(owner, alias, target):
    matches = [item for item in owner['aliases'] if item['alias'] == alias]
    if matches != [dict(name=target, alias=alias, depends=None, manual=True)]:
        raise ValueError(f'unaudited {owner["name"]}.{alias} declaration')


def select(ir, first_id):
    vehicle = _owner(ir, VEHICLE_OWNER)
    expected_vehicle = dict(rename='vehicle', depends='AP_SCRIPTING_BINDING_VEHICLE_ENABLED',
                            get=None, literal=False, reference=False, semaphore=False,
                            scheduler_semaphore=True, semaphore_pointer=False)
    if any(vehicle[key] != value for key, value in expected_vehicle.items()):
        raise ValueError('unaudited AP_Vehicle owner metadata')
    _manual_alias(vehicle, VELOCITY_ALIAS, 'lua_AP_Vehicle_set_target_velocity_NED')

    srv = _owner(ir, SRV_OWNER)
    expected_srv = dict(rename=SRV_OWNER,
                        depends='(!defined(HAL_BUILD_AP_PERIPH) || AP_PERIPH_RC_OUT_ENABLED)', get=None,
                        literal=False, reference=False, semaphore=False,
                        scheduler_semaphore=False, semaphore_pointer=False)
    if any(srv[key] != value for key, value in expected_srv.items()):
        raise ValueError('unaudited SRV_Channels owner metadata')
    _manual_alias(srv, SAFETY_ALIAS, 'SRV_Channels_get_safety_state')
    return specification(first_id)


def specification(first_id):
    return dict(methods=[
        dict(id=first_id, name='vehicle.set_target_velocity_ned',
             arguments=[dict(type='owned-vector3f', name='velocity_ned'),
                        dict(type='bool-u8', name='align_yaw_to_target')],
             result='bool-u8', policy=VELOCITY_POLICY),
        dict(id=first_id + 1, name='srv_channels.get_safety_state', arguments=[],
             result='bool-u8', policy=SAFETY_POLICY),
        dict(id=first_id + 2, name='mission.receive', arguments=[],
             result=dict(presence='bool-u8', fields=[['time_ms', 'u32-le'], ['param1', 'u16-le'],
                                                     ['param2', 'f32-le,all-bits'],
                                                     ['param3', 'f32-le,all-bits'],
                                                     ['param4', 'f32-le,all-bits']]),
             policy=MISSION_POLICY),
        dict(id=first_id + 3, name='DroneCan.get_flex_debug',
             arguments=[dict(type='u8', name='driver'), dict(type='u8', name='node_id'),
                        dict(type='u16-le', name='message_id'),
                        dict(type='u32-le', name='last_timestamp_us')],
             result=dict(presence='bool-u8', timestamp_us='u32-le', length='u8',
                         payload=f'bytes-max{FLEXDEBUG_PAYLOAD_MAX}-zero-pad{FLEXDEBUG_PAYLOAD_MAX}'),
             policy=FLEXDEBUG_POLICY),
    ])


def cpp_headers():
    return [
        '#include <AP_HAL/AP_HAL.h>',
        'extern const AP_HAL::HAL &hal;',
        '#include <AP_Scripting/AP_Scripting_config.h>',
        '#include <AP_Mission/AP_Mission_config.h>',
        '#if AP_MISSION_ENABLED && AP_MODULE_ENABLED',
        '#include <AP_Module/Runtime.h>',
        '#endif',
        '#if HAL_ENABLE_DRONECAN_DRIVERS && AP_SCRIPTING_ENABLED',
        '#include <AP_DroneCAN/AP_DroneCAN.h>',
        '#endif',
    ]


def cpp(spec):
    velocity, safety, mission, flexdebug = spec['methods']
    return [
        f'// {velocity["name"]}',
        'static __attribute__((noinline)) int32_t binding_examples_vehicle_set_target_velocity_ned(',
        '    const uint8_t *input, uint32_t, uint8_t *output, uint32_t)',
        '{',
        '    output[0] = 0;',
        '    if (input[12] > 1) {',
        '        return AP_MODULE_STATUS_INVALID;',
        '    }',
        '#if AP_SCRIPTING_BINDING_VEHICLE_ENABLED',
        '    auto *vehicle = AP_Vehicle::get_singleton();',
        '    if (vehicle == nullptr) {',
        '        return AP_MODULE_STATUS_UNAVAILABLE;',
        '    }',
        '    Vector3f velocity_ned{BindingWire::read_float(input),',
        '                          BindingWire::read_float(input + 4),',
        '                          BindingWire::read_float(input + 8)};',
        '    BindingLock lock(AP::scheduler().get_semaphore());',
        '    output[0] = vehicle->set_target_velocity_NED(velocity_ned, input[12] != 0) ? 1 : 0;',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
        f'// {safety["name"]}',
        'static __attribute__((noinline)) int32_t binding_examples_srv_channels_get_safety_state(',
        '    const uint8_t *, uint32_t, uint8_t *output, uint32_t)',
        '{',
        '    output[0] = 0;',
        '#if (!defined(HAL_BUILD_AP_PERIPH) || AP_PERIPH_RC_OUT_ENABLED)',
        '    if (hal.util == nullptr) {',
        '        return AP_MODULE_STATUS_UNAVAILABLE;',
        '    }',
        '    output[0] = hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_ARMED ? 1 : 0;',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
        f'// {mission["name"]}',
        'static __attribute__((noinline)) int32_t binding_examples_mission_receive(',
        '    const uint8_t *, uint32_t, uint8_t *output, uint32_t)',
        '{',
        f'    memset(output, 0, {MISSION_WIRE_SIZE});',
        '#if AP_MISSION_ENABLED && AP_MODULE_ENABLED',
        '    auto *runtime = AP_Module::Runtime::get_singleton();',
        '    if (runtime == nullptr) {',
        '        return AP_MODULE_STATUS_UNAVAILABLE;',
        '    }',
        '    AP_Module::Runtime::MissionCommand command;',
        '    if (!runtime->pop_mission_command(command)) {',
        '        return AP_MODULE_STATUS_OK;',
        '    }',
        '    output[0] = 1;',
        '    BindingWire::write(output + 1, command.time_ms, 4);',
        '    BindingWire::write(output + 5, command.p1, 2);',
        '    BindingWire::write_float(output + 7, command.content_p1);',
        '    BindingWire::write_float(output + 11, command.content_p2);',
        '    BindingWire::write_float(output + 15, command.content_p3);',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
        f'// {flexdebug["name"]}',
        'static __attribute__((noinline)) int32_t binding_examples_dronecan_get_flex_debug(',
        '    const uint8_t *input, uint32_t, uint8_t *output, uint32_t)',
        '{',
        '    (void)input;',
        f'    memset(output, 0, {FLEXDEBUG_WIRE_SIZE});',
        '#if HAL_ENABLE_DRONECAN_DRIVERS && AP_SCRIPTING_ENABLED',
        '    const uint8_t driver = input[0];',
        '    const uint8_t node_id = input[1];',
        '    const uint16_t message_id = uint16_t(BindingWire::read(input + 2, 2));',
        '    uint32_t timestamp_us = uint32_t(BindingWire::read(input + 4, 4));',
        '    const auto *dronecan = AP_DroneCAN::get_dronecan(driver);',
        '    if (dronecan == nullptr) {',
        '        return AP_MODULE_STATUS_OK;',
        '    }',
        '    dronecan_protocol_FlexDebug message;',
        f'    static_assert(sizeof(message.u8.data) == {FLEXDEBUG_PAYLOAD_MAX},',
        '                  "unexpected FlexDebug payload capacity");',
        '    if (!dronecan->get_FlexDebug(node_id, message_id, timestamp_us, message)) {',
        '        return AP_MODULE_STATUS_OK;',
        '    }',
        '    output[0] = 1;',
        '    BindingWire::write(output + 1, timestamp_us, 4);',
        '    output[5] = message.u8.len;',
        '    memcpy(output + 6, message.u8.data, message.u8.len);',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
    ]


def rust(spec, notice):
    velocity, safety, mission, flexdebug = spec['methods']
    return notice + render(
        'rust/examples.rs',
        flexdebug_payload_max=FLEXDEBUG_PAYLOAD_MAX,
        velocity_id=velocity['id'],
        safety_id=safety['id'],
        mission_wire_size=MISSION_WIRE_SIZE,
        mission_id=mission['id'],
        flexdebug_wire_size=FLEXDEBUG_WIRE_SIZE,
        flexdebug_id=flexdebug['id'],
    )
