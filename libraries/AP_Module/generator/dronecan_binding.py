# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Lifecycle-owned DroneCAN broadcast subscriptions for native modules."""

from native_templates import render

PAYLOAD_MAX = 256
MESSAGE_WIRE_SIZE = 1 + 2 + PAYLOAD_MAX + 1 + 8 + 1
POLICY = ('v1:module-context-owned-subscription,max16-firmware-wide,cleanup-after-worker-join,'
          'subscription-transfer-type-broadcast,queue-depth8,payload-max256,oversize-and-full-queue-drop,'
          'owned-broadcast-handle,priority0,timeout10ms,per-handle-transfer-id,can-fd-option,'
          'no-retained-module-pointers,token-revalidated-by-context,'
          'zero-skipped-live-token-collision-scan')
REQUEST_POLICY = (POLICY + ',request-node1-to127,response-node-and-5bit-transfer-id-filter,'
                  'failed-request-removes-response-subscription')


def select(ir, first_id):
    owner = next((item for item in ir['userdata'] if item['name'] == 'DroneCAN_Handle'), None)
    expected = dict(rename=None, depends='HAL_ENABLE_DRONECAN_DRIVERS', get=None,
                    creation='DroneCAN_Handle::new_handle', creation_manual=True, creation_args=4,
                    has_fields=False, has_enums=False, operator_getter=None, operators_manual=True,
                    operator_names=[], literal=False, reference=False, semaphore=False,
                    scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited DroneCAN_Handle metadata')
    subscribe = [method for method in owner['methods'] if method['name'] == 'subscribe']
    if len(subscribe) != 1:
        raise ValueError('unaudited DroneCAN_Handle.subscribe method')
    method = subscribe[0]
    result = method['return']
    result_shape = (result['kind'], result['name'], result['nullable'], result['reference'],
                    result['pointer'], result['skip_check'], result['range'])
    if (result_shape != ('boolean', None, False, False, False, False, None) or method['arguments'] or
            method['rename'] is not None or method['depends'] is not None or method['deprecate'] is not None):
        raise ValueError('unaudited DroneCAN_Handle.subscribe method')
    aliases = sorted((alias['name'], alias['alias'], alias['depends'], alias['manual'])
                     for alias in owner['aliases'])
    expected_aliases = sorted([
        ('DroneCAN_Handle::__gc', '__gc', None, True),
        ('DroneCAN_Handle::broadcast', 'broadcast', None, True),
        ('DroneCAN_Handle::check_message', 'check_message', None, True),
        ('DroneCAN_Handle::request', 'request', None, True),
    ])
    if aliases != expected_aliases:
        raise ValueError('unaudited DroneCAN_Handle manual operations')
    return specification(first_id)


def specification(first_id):
    subscribe = dict(id=first_id, name='DroneCan.subscribe',
                     arguments=[dict(type='u8', direction='in'), dict(type='u64-le', direction='in'),
                                dict(type='u16-le', direction='in')],
                     result=dict(presence='u32-le,zero-is-none', handle='DroneCanSubscription'), policy=POLICY)
    check = dict(id=first_id + 1, name='DroneCanSubscription.check_message',
                 contract_aliases=['DroneCanHandle.check_message'],
                 arguments=[dict(type='u32-le,nonzero', direction='module-owned-receiver')],
                 result=dict(presence='bool-u8', length='u16-le',
                             payload=f'bytes-max{PAYLOAD_MAX}-zero-pad{PAYLOAD_MAX}', node_id='u8',
                             timestamp_us='u64-le', can_fd='bool-u8'), policy=POLICY)
    close = dict(id=first_id + 2, name='DroneCanSubscription.close',
                 arguments=[dict(type='u32-le,nonzero', direction='module-owned-receiver')],
                 result='void', policy=POLICY)
    open_handle = dict(id=first_id + 3, name='DroneCan.open',
                       arguments=[dict(type='u8', direction='in'), dict(type='u64-le', direction='in'),
                                  dict(type='u16-le', direction='in'), dict(type='bool-u8', direction='in')],
                       result=dict(presence='u32-le,zero-is-none', handle='DroneCanHandle'), policy=POLICY)
    broadcast = dict(id=first_id + 4, name='DroneCanHandle.broadcast',
                     arguments=[dict(type='u32-le,nonzero', direction='module-owned-receiver'),
                                dict(type=f'bytes-max{PAYLOAD_MAX}', direction='in')],
                     result='bool-u8', policy=POLICY)
    handle_close = dict(id=first_id + 5, name='DroneCanHandle.close',
                        arguments=[dict(type='u32-le,nonzero', direction='module-owned-receiver')],
                        result='void', policy=POLICY)
    request = dict(id=first_id + 6, name='DroneCanHandle.request',
                   arguments=[dict(type='u32-le,nonzero', direction='module-owned-receiver'),
                              dict(type='u8', range=[1, 127], direction='target-node'),
                              dict(type=f'bytes-max{PAYLOAD_MAX}', direction='in')],
                   result='bool-u8', policy=REQUEST_POLICY)
    return dict(methods=[subscribe, check, close, open_handle, broadcast, handle_close, request])


def rust(spec, notice):
    subscribe, check, close, open_handle, broadcast, handle_close, request = spec['methods']
    return notice + render(
        'rust/dronecan.rs',
        payload_max=PAYLOAD_MAX,
        message_wire_size=MESSAGE_WIRE_SIZE,
        check_id=check['id'],
        message_wire_size_minus_1=MESSAGE_WIRE_SIZE - 1,
        n3_plus_payload_max=3 + PAYLOAD_MAX,
        n4_plus_payload_max=4 + PAYLOAD_MAX,
        n12_plus_payload_max=12 + PAYLOAD_MAX,
        subscribe_id=subscribe['id'],
        open_handle_id=open_handle['id'],
        close_id=close['id'],
        broadcast_id=broadcast['id'],
        request_id=request['id'],
        handle_close_id=handle_close['id'],
    )
