# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""AP_Periph vehicle-notification state with an owned u64 wire value."""

import rust_calls

from call_model import BOOL
from call_model import U32
from call_model import U64
from call_model import Call
from call_model import Field
from call_model import Module
from call_model import Record
from call_model import String

OWNER = 'AP_Periph_FW'
POLICY = 'v1:read-singleton-state,u64-little-endian,no-retained-firmware-pointers'


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected = dict(rename='periph', depends='defined(HAL_BUILD_AP_PERIPH)', get=None,
                    literal=False, reference=False, semaphore=False,
                    scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited AP_Periph_FW metadata')
    method = next((item for item in owner['methods'] if item['name'] == 'get_vehicle_state'), None)
    if (method is None or method['return']['kind'] != 'userdata' or method['return']['name'] != 'uint64_t' or
            method['arguments'] or method['depends'] != 'AP_PERIPH_NOTIFY_ENABLED'):
        raise ValueError('unaudited AP_Periph_FW.get_vehicle_state')
    return specification(first_id)


def specification(first_id):
    return dict(methods=[
        dict(id=first_id, name='periph.get_vehicle_state', arguments=[], result='u64-le', policy=POLICY),
        dict(id=first_id + 1, name='periph.elrs_shadow', arguments=[], result='nine-u32-le',
             policy='v1:latest-packet-snapshot,crc-and-zero-state-channel-signature,no-control'),
        dict(id=first_id + 2, name='periph.elrs_codec_register',
             arguments=[dict(type='u32-le', name='version'), dict(type='u32-le', name='address')], result='bool-u8',
             policy='v1:trusted-pure-thumb-callback,55-in-37-out,owned-rwpi,pinned-until-return,revoke-before-release'),
        dict(id=first_id + 3, name='periph.elrs_codec_stats', arguments=[], result='five-u32-le',
             policy='v1:accepted,rejected,unavailable,faults,max-us,independent-relaxed-counters'),
        dict(id=first_id + 4, name='periph.debug_write',
             arguments=[dict(type='cstring256', name='text')], result='void',
             policy='v1:stdout-literal-line,max255-bytes,no-retained-pointer,worker-only'),
        dict(id=first_id + 5, name='periph.elrs_service_register',
             arguments=[dict(type='u32-le', name='version'), dict(type='u32-le', name='address')], result='bool-u8',
             policy='v1:exclusive-state1536,80-in-256-out,module-worker-init,pinned-rwpi,revoke-before-release'),
        dict(id=first_id + 6, name='periph.elrs_service_stats', arguments=[], result='five-u32-le',
             policy='v1:calls,rejected,unavailable,faults,max-us,independent-relaxed-counters'),
        dict(id=first_id + 7, name='periph.sx1280_register',
             arguments=[dict(type='u32-le', name='version'), dict(type='u32-le', name='address')], result='bool-u8',
             policy='v1:opaque4096,events128-actions256,serialized-native-worker,pinned-until-return'),
        dict(id=first_id + 8, name='periph.sx1280_stats', arguments=[], result='five-u32-le',
             policy='v1:calls,rejected,unavailable,faults,max-us'),
        dict(id=first_id + 9, name='periph.sx1280_configure',
             arguments=[dict(type='u32-le', name=n) for n in ('word0', 'word1', 'word2')], result='bool-u8',
             policy='v1:opaque12-byte-init-settings,no-active-owner,worker-only'),
        dict(id=first_id + 10, name='periph.sx1280_snapshot',
             arguments=[dict(type='u32-le', name='offset')], result='four-u32-le',
             policy='v1:copy16-bytes-from-256-byte-report,offset0-240,owner-only'),
    ])


def client_model(spec):
    method = spec['methods'][0]
    shadow = spec['methods'][1]
    record = Record('ElrsShadow', tuple(Field(name, U32) for name in (
        'sequence', 'packet0', 'packet1', 'packet2', 'packet3', 'control', 'mode', 'flags', 'signature')))
    stats = Record('ElrsCodecStats', tuple(Field(name, U32) for name in (
        'accepted', 'rejected', 'unavailable', 'faults', 'max_us')))
    register, counters, debug, service, service_stats = spec['methods'][2:7]
    words = Record('RadioWords', tuple(Field(name, U32) for name in ('word0', 'word1', 'word2', 'word3')))
    return Module((Call(method['name'], method['id'], result=U64),
                   Call(shadow['name'], shadow['id'], result=record),
                   Call(register['name'], register['id'], arguments=(Field('version', U32), Field('address', U32)),
                        result=BOOL),
                   Call(counters['name'], counters['id'], result=stats),
                   Call(debug['name'], debug['id'], arguments=(Field('text', String(256)),)),
                   Call(service['name'], service['id'], arguments=(Field('version', U32), Field('address', U32)),
                        result=BOOL),
                   Call(service_stats['name'], service_stats['id'], result=stats),
                   Call(spec['methods'][7]['name'], spec['methods'][7]['id'],
                        arguments=(Field('version', U32), Field('address', U32)), result=BOOL),
                   Call(spec['methods'][8]['name'], spec['methods'][8]['id'], result=stats),
                   Call(spec['methods'][9]['name'], spec['methods'][9]['id'],
                        arguments=tuple(Field(n, U32) for n in ('word0', 'word1', 'word2')), result=BOOL),
                   Call(spec['methods'][10]['name'], spec['methods'][10]['id'],
                        arguments=(Field('offset', U32),), result=words)), types=(record, stats, words))


def rust(spec, notice):
    source = rust_calls.emit(client_model(spec), notice)
    source = source.replace('    pub fn sx1280_register(',
                            '    /// # Safety\n'
                            '    /// Entry must implement the bounded radio event/action ABI v1 without SDK calls.\n'
                            '    pub unsafe fn sx1280_register(')
    source = source.replace('    pub fn elrs_service_register(',
                            '    /// # Safety\n'
                            '    /// Entry must implement exclusive bounded service ABI v1 without SDK calls or panics.\n'
                            '    pub unsafe fn elrs_service_register(')
    return source.replace('    pub fn elrs_codec_register(',
                          '    /// # Safety\n'
                          '    /// Entry must implement the pure bounded ABI v1 and never panic, allocate,\n'
                          '    /// call SDK methods, or access mutable globals from the RF thread.\n'
                          '    pub unsafe fn elrs_codec_register(')
