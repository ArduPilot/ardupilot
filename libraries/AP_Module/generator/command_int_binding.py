# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Typed native adapter for Lua's manual GCS.run_command_int binding."""

from native_templates import render

OWNER = 'GCS'
ALIAS = 'run_command_int'
TARGET = 'lua_GCS_command_int'
WIRE_SIZE = 31
POLICY = ('v1:owned-command-int-copy,zero-default-params,all-f32-bits,x-y-i32,frame-u8,'
          'command-u16,blocking-scheduler-semaphore,channel0,dummy-message,no-retained-module-pointers')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    if owner is None:
        raise ValueError('missing GCS singleton')
    expected = dict(rename='gcs', depends='(HAL_GCS_ENABLED && !defined(HAL_BUILD_AP_PERIPH))',
                    get=None, literal=False, reference=False, semaphore=False,
                    scheduler_semaphore=False, semaphore_pointer=False)
    if any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited GCS owner metadata')
    aliases = [alias for alias in owner['aliases'] if alias['alias'] == ALIAS]
    if aliases != [dict(name=TARGET, alias=ALIAS, depends=None, manual=True)]:
        raise ValueError('unaudited GCS.run_command_int declaration')
    return specification(first_id)


def specification(first_id):
    return dict(
        id=first_id,
        name='gcs.run_command_int',
        arguments=[
            dict(type='u16-le', direction='in'),
            dict(type=dict(name='CommandIntParams', fields=[
                ['p1', 'f32-le'], ['p2', 'f32-le'], ['p3', 'f32-le'], ['p4', 'f32-le'],
                ['x', 'i32-le'], ['y', 'i32-le'], ['z', 'f32-le'], ['frame', 'u8'],
            ]), direction='in'),
        ],
        result=dict(type='MAV_RESULT', encoding='i32-le'),
        policy=POLICY,
    )


def cpp(method):
    binding = method['id']
    return [
        '// GCS::run_command_int manual adapter',
        f'static __attribute__((noinline)) int32_t binding_{binding}(',
        '    const uint8_t *input, uint32_t, uint8_t *output, uint32_t)',
        '{',
        '    (void)input;',
        '    memset(output, 0, 4);',
        '#if AP_SCRIPTING_ENABLED && HAL_GCS_ENABLED && !defined(HAL_BUILD_AP_PERIPH) && AP_SCHEDULER_ENABLED',
        '    auto *gcs = GCS::get_singleton();',
        '    auto *scheduler = AP_Scheduler::get_singleton();',
        '    if (gcs == nullptr || scheduler == nullptr) {',
        '        return AP_MODULE_STATUS_UNAVAILABLE;',
        '    }',
        '    mavlink_command_int_t packet{};',
        '    packet.command = uint16_t(BindingWire::read(input, 2));',
        '    packet.param1 = BindingWire::read_float(input + 2);',
        '    packet.param2 = BindingWire::read_float(input + 6);',
        '    packet.param3 = BindingWire::read_float(input + 10);',
        '    packet.param4 = BindingWire::read_float(input + 14);',
        '    const uint32_t x_bits = BindingWire::read(input + 18, 4);',
        '    const uint32_t y_bits = BindingWire::read(input + 22, 4);',
        '    memcpy(&packet.x, &x_bits, sizeof(packet.x));',
        '    memcpy(&packet.y, &y_bits, sizeof(packet.y));',
        '    packet.z = BindingWire::read_float(input + 26);',
        '    packet.frame = input[30];',
        '    BindingLock lock(scheduler->get_semaphore());',
        '    const MAV_RESULT result = gcs->lua_command_int_packet(packet);',
        '    if (!BindingWire::write_enum(output, result)) {',
        '        return AP_MODULE_STATUS_HOST_ERROR;',
        '    }',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
    ]


def rust(method, notice):
    return notice + render(
        'rust/command_int.rs',
        wire_size=WIRE_SIZE,
        method_id=method['id'],
    )
