# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Lifecycle-owned raw MAVLink service for native modules."""

from native_templates import render

OWNER = 'mavlink'
MESSAGE_SIZE = 298
PAYLOAD_MAX = 255
RECEIVE_WIRE_SIZE = 1 + MESSAGE_SIZE + 1 + 4
SEND_WIRE_SIZE = 1 + 4 + 2 + 1 + 4 + PAYLOAD_MAX
ALIASES = {
    'init': 'lua_mavlink_init',
    'register_rx_msgid': 'lua_mavlink_register_rx_msgid',
    'receive_chan': 'lua_mavlink_receive_chan',
    'send_chan': 'lua_mavlink_send_chan',
    'block_command': 'lua_mavlink_block_command',
}
POLICY = ('v2:module-context-owned-rx,cleanup-after-worker-join,dynamic-owner-registry,queue-and-id-max25,'
          'full-mavlink-message298,channel-u8,timestamp-ms-u32,queue-full-drops-newest,'
          'payload-max255,target-system-u32,target32-compatible,no-retained-module-pointers')
BLOCK_POLICY = ('v1:module-context-owned-command-blocks,cleanup-after-worker-join,u16-command-id,'
                'duplicate-owner-idempotent,shared-with-lua-owner-isolation,no-retained-module-pointers')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    if owner is None:
        raise ValueError('missing mavlink singleton')
    expected = dict(rename=None, depends='(HAL_GCS_ENABLED && !defined(HAL_BUILD_AP_PERIPH))',
                    get=None, literal=False, reference=False, semaphore=False,
                    scheduler_semaphore=False, semaphore_pointer=False)
    if any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited mavlink owner metadata')
    for alias, target in ALIASES.items():
        matches = [item for item in owner['aliases'] if item['alias'] == alias]
        if matches != [dict(name=target, alias=alias, depends=None, manual=True)]:
            raise ValueError(f'unaudited mavlink.{alias} declaration')
    return specification(first_id)


def specification(first_id):
    methods = [
        dict(id=first_id, name='mavlink.init',
             arguments=[dict(type='u8', range='0..25', name='queue_depth'),
                        dict(type='u8', range='0..25', name='message_ids')], result='void', policy=POLICY),
        dict(id=first_id + 1, name='mavlink.register_rx_msgid',
             arguments=[dict(type='u24-in-u32-le', name='msgid')], result='bool-u8', policy=POLICY),
        dict(id=first_id + 2, name='mavlink.receive_chan', arguments=[],
             result=dict(presence='bool-u8', message=f'bytes{MESSAGE_SIZE}', channel='u8',
                         timestamp_ms='u32-le'), policy=POLICY),
        dict(id=first_id + 3, name='mavlink.send_chan',
             arguments=[dict(type='u8', name='channel'), dict(type='u24-in-u32-le', name='msgid'),
                        dict(type=f'bytes-max{PAYLOAD_MAX}-zero-pad{PAYLOAD_MAX}', name='payload'),
                        dict(type='optional-u32-le', name='target_system')],
             result='optional-bool-u8', policy=POLICY),
        dict(id=first_id + 4, name='mavlink.block_command',
             arguments=[dict(type='u16-le', name='command')], result='bool-u8', policy=BLOCK_POLICY),
    ]
    return dict(methods=methods)


def rust(spec, notice):
    init, register, receive, send, block = spec['methods']
    return notice + render(
        'rust/mavlink.rs',
        message_size=MESSAGE_SIZE,
        payload_max=PAYLOAD_MAX,
        init_id=init['id'],
        register_id=register['id'],
        receive_wire_size=RECEIVE_WIRE_SIZE,
        receive_id=receive['id'],
        send_wire_size=SEND_WIRE_SIZE,
        send_id=send['id'],
        block_id=block['id'],
    )
