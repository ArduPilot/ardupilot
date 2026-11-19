# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Snapshot of named AP_Notify flags as a fixed u32 bitmask, and of event counts."""

import rust_calls

from call_model import U32
from call_model import U64
from call_model import Call
from call_model import Module

OWNER = 'AP_Notify'
# Bit n is FLAGS[n]. Append only: the order is the wire contract.
FLAGS = ('armed', 'flying', 'have_pos_abs', 'gps_glitching', 'ekf_bad', 'failsafe_battery',
         'failsafe_gcs', 'failsafe_radio', 'compass_cal_running', 'initialising',
         'pre_arm_check', 'pre_arm_gps_check')
POLICY = 'v1:flags-' + '-'.join(FLAGS) + ',bit-per-flag-lsb-first,unlocked-snapshot'
# Byte n is the wrapping count of EVENTS[n]. Append only: the order is the wire contract.
EVENTS = ('initiated_compass_cal', 'compass_cal_saved', 'compass_cal_failed', 'compass_cal_canceled',
          'initiated_temp_cal', 'temp_cal_saved', 'temp_cal_failed', 'arming_failed')
EVENTS_POLICY = 'v1:events-' + '-'.join(EVENTS) + ',u8-wrapping-count-per-byte-lsb-first,unlocked-snapshot'


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected = dict(rename='notify', get=None, literal=False, reference=False,
                    semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited AP_Notify metadata')
    return specification(first_id, owner['depends'])


def specification(first_id, depends=None):
    return dict(depends=depends, methods=[
        dict(id=first_id, name='notify.get_flags', arguments=[], result='u32-le', policy=POLICY),
        dict(id=first_id + 1, name='notify.get_event_counts', arguments=[], result='u64-le',
             policy=EVENTS_POLICY)])


def client_model(spec):
    flags, events = spec['methods']
    return Module((Call(flags['name'], flags['id'], result=U32, docs=(
        'Named AP_Notify flags packed as a bitmask; see NotifyFlags for the bits.',
        'Flags are read without a lock, so separate flags may come from adjacent updates.',
    )), Call(events['name'], events['id'], result=U64, docs=(
        'Wrapping 8-bit counts of named AP_Notify events, one per byte; see NotifyEvents for the shifts.',
        'Compare with an earlier value to see events that happened since; Unavailable if not counted.',
    ))))


def cpp(spec):
    return flags_cpp(spec) + events_cpp(spec)


def flags_cpp(spec):
    method = spec['methods'][0]
    lines = [f'// {method["name"]}',
             'static __attribute__((noinline)) int32_t binding_notify_get_flags(',
             '    const uint8_t *, uint32_t, uint8_t *output, uint32_t)', '{']
    if spec['depends']:
        lines.append(f'#if {spec["depends"]}')
    lines += ['    uint32_t value = 0;']
    lines += [f'    value |= uint32_t(AP_Notify::flags.{name}) << {bit};' for bit, name in enumerate(FLAGS)]
    lines += ['    BindingWire::write(output, value, 4);', '    return AP_MODULE_STATUS_OK;']
    if spec['depends']:
        lines += ['#else', '    memset(output, 0, 4);', '    return AP_MODULE_STATUS_UNAVAILABLE;', '#endif']
    return lines + ['}', '']


def events_cpp(spec):
    method = spec['methods'][1]
    condition = 'AP_NOTIFY_EVENT_COUNTS_ENABLED'
    if spec['depends']:
        condition = f'({spec["depends"]}) && {condition}'
    lines = [f'// {method["name"]}',
             'static __attribute__((noinline)) int32_t binding_notify_get_event_counts(',
             '    const uint8_t *, uint32_t, uint8_t *output, uint32_t)', '{',
             f'#if {condition}']
    # counts are indexed by the event's bit position in notify_events_type
    for byte, name in enumerate(EVENTS):
        lines += ['    {', '        AP_Notify::notify_events_type event {};', f'        event.{name} = 1;',
                  '        uint32_t bits;', '        memcpy(&bits, &event, sizeof(bits));',
                  f'        output[{byte}] = AP_Notify::get_event_count(__builtin_ctz(bits));', '    }']
    lines += ['    return AP_MODULE_STATUS_OK;',
              '#else', '    memset(output, 0, 8);', '    return AP_MODULE_STATUS_UNAVAILABLE;', '#endif']
    return lines + ['}', '']


def rust(spec, notice):
    constants = ''.join(f'    pub const {name.upper()}: u32 = 1 << {bit};\n' for bit, name in enumerate(FLAGS))
    shifts = ''.join(f'    pub const {name.upper()}: u32 = {8 * byte};\n' for byte, name in enumerate(EVENTS))
    return (rust_calls.emit(client_model(spec), notice) +
            '\n/// Bits of `notify().get_flags()`.\npub struct NotifyFlags;\n\nimpl NotifyFlags {\n' +
            constants + '}\n' +
            '\n/// Shifts of the 8-bit counts in `notify().get_event_counts()`.\npub struct NotifyEvents;\n\n'
            'impl NotifyEvents {\n' + shifts + '}\n')
