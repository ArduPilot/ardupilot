# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Module-owned bounded TCP/UDP sockets for native modules."""

from native_templates import render

OWNER = 'SocketAPM'
NETWORK_OWNER = 'AP_Networking'
TRANSFER_MAX = 1024
PATH_SIZE = 256
POLL_TIMEOUT_MAX_MS = 100
POLICY = ('v2:module-context-owned,dynamic-owner-registry,max50-sockets-per-module,generation-checked-handles,'
          'global-monotonic-generation-seeds-persist-across-context-restart,token-exhaustion-fails-closed,'
          'cleanup-after-worker-join,'
          'blocking-registry-lock,descriptor-always-nonblocking,logical-blocking-connect-and-io-poll20ms,'
          'poll-timeout-max100ms,transfer-max1024,fixed-zero-padded-wire,accept-bounded,'
          'sendfile-transfers-fd-and-socket-to-network-worker,no-retained-module-pointers')


def select(ir, first_id):
    owner = next((item for item in ir['ap_objects'] if item['name'] == OWNER), None)
    expected_owner = dict(rename=None, depends='(AP_NETWORKING_ENABLED==1)', get=None, creation=None,
                          creation_manual=False, creation_args=0, has_fields=False, has_enums=False,
                          operations=0, operator_getter=None, operators_manual=False, operator_names=[],
                          literal=False, reference=False, semaphore=False, scheduler_semaphore=False,
                          semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected_owner.items()):
        raise ValueError('unaudited SocketAPM metadata')

    def field_shape(field):
        return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
                field['skip_check'], tuple(field['range']) if field['range'] else None)

    expected_methods = {
        'bind': (field_shape(dict(kind='boolean', name=None, nullable=False, reference=False,
                                  pointer=False, skip_check=False, range=None)),
                 [('string', None, False, False, False, False, None),
                  ('uint16_t', None, False, False, False, True, None)]),
        'connect': (field_shape(dict(kind='boolean', name=None, nullable=False, reference=False,
                                     pointer=False, skip_check=False, range=None)),
                    [('string', None, False, False, False, False, None),
                     ('uint16_t', None, False, False, False, True, None)]),
        'is_connected': (('boolean', None, False, False, False, False, None), []),
        'is_pending': (('boolean', None, False, False, False, False, None), []),
        'listen': (('boolean', None, False, False, False, False, None),
                   [('uint8_t', None, False, False, False, True, None)]),
        'pollin': (('boolean', None, False, False, False, False, None),
                   [('uint32_t', None, False, False, False, True, None)]),
        'pollout': (('boolean', None, False, False, False, False, None),
                    [('uint32_t', None, False, False, False, True, None)]),
        'send': (('int32_t', None, False, False, False, False, None),
                 [('string', None, False, False, False, False, None),
                  ('uint32_t', None, False, False, False, True, None)]),
        'sendto': (('int32_t', None, False, False, False, False, None),
                   [('string', None, False, False, False, False, None),
                    ('uint32_t', None, False, False, False, True, None),
                    ('uint32_t', None, False, False, False, True, None),
                    ('uint16_t', None, False, False, False, True, None)]),
        'reuseaddress': (('boolean', None, False, False, False, False, None), []),
        'set_blocking': (('boolean', None, False, False, False, False, None),
                         [('boolean', None, False, False, False, False, None)]),
    }
    methods = {method['name']: (field_shape(method['return']), [field_shape(arg) for arg in method['arguments']])
               for method in owner['methods'] if method['name'] in expected_methods}
    if methods != expected_methods:
        raise ValueError('unaudited SocketAPM methods')
    aliases = {item['alias']: item['name'] for item in owner['aliases'] if item['manual'] and item['depends'] is None}
    if aliases != {'accept': 'SocketAPM_accept', 'recv': 'SocketAPM_recv',
                   'close': 'SocketAPM_close', 'sendfile': 'SocketAPM_sendfile'}:
        raise ValueError('unaudited SocketAPM manual aliases')

    networking = next((item for item in ir['singletons'] if item['name'] == NETWORK_OWNER), None)
    expected_networking = dict(rename='networking', depends='AP_NETWORKING_ENABLED', get=None,
                               literal=False, reference=False, semaphore=False,
                               scheduler_semaphore=False, semaphore_pointer=False)
    if networking is None or any(networking[key] != value for key, value in expected_networking.items()):
        raise ValueError('unaudited AP_Networking metadata')
    address = next((method for method in networking['methods'] if method['name'] == 'address_to_str'), None)
    if (address is None or field_shape(address['return']) !=
            ('string', None, False, False, False, False, None) or
            [field_shape(arg) for arg in address['arguments']] !=
            [('uint32_t', None, False, False, False, True, None)]):
        raise ValueError('unaudited AP_Networking.address_to_str method')
    return specification(first_id)


def specification(first_id):
    handle = dict(type='u32-le,nonzero,module-owned-generation-handle', direction='in')
    methods = [
        dict(id=first_id, name='network.socket_open',
             arguments=[dict(type='bool-u8', name='datagram')], result='nonzero-u32-le-handle', policy=POLICY),
        dict(id=first_id + 1, name='network.socket_close', arguments=[handle], result='void', policy=POLICY),
        dict(id=first_id + 2, name='network.socket_bind',
             arguments=[handle, dict(type='nonempty-nul-free-bytes-max255-zero-pad256', name='address'),
                        dict(type='nonzero-u16-le', name='port')], result='bool-u8', policy=POLICY),
        dict(id=first_id + 3, name='network.socket_listen',
             arguments=[handle, dict(type='nonzero-u8', name='backlog')], result='bool-u8', policy=POLICY),
        dict(id=first_id + 4, name='network.socket_set_blocking',
             arguments=[handle, dict(type='bool-u8', name='blocking')], result='void', policy=POLICY),
        dict(id=first_id + 5, name='network.socket_is_connected', arguments=[handle], result='bool-u8', policy=POLICY),
        dict(id=first_id + 6, name='network.socket_pollin',
             arguments=[handle, dict(type='u16-le', range=[0, POLL_TIMEOUT_MAX_MS], name='timeout_ms')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 7, name='network.socket_pollout',
             arguments=[handle, dict(type='u16-le', range=[0, POLL_TIMEOUT_MAX_MS], name='timeout_ms')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 8, name='network.socket_accept', arguments=[handle],
             result='optional-nonzero-u32-le-handle', policy=POLICY),
        dict(id=first_id + 9, name='network.socket_recv',
             arguments=[handle, dict(type='u16-le', range=[1, TRANSFER_MAX], name='length')],
             result=dict(presence='bool-u8', length='u16-le',
                         data=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}'), policy=POLICY),
        dict(id=first_id + 10, name='network.socket_send',
             arguments=[handle, dict(type=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}', name='data')],
             result=dict(presence='bool-u8', length='u16-le'), policy=POLICY),
        dict(id=first_id + 11, name='network.socket_sendfile',
             arguments=[handle, dict(type='nonempty-nul-free-bytes-max255-zero-pad256', name='path')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 12, name='network.socket_connect',
             arguments=[handle, dict(type='nonempty-nul-free-bytes-max255-zero-pad256', name='address'),
                        dict(type='nonzero-u16-le', name='port')], result='bool-u8', policy=POLICY),
        dict(id=first_id + 13, name='network.socket_recv_from',
             arguments=[handle, dict(type='u16-le', range=[1, TRANSFER_MAX], name='length')],
             result=dict(presence='bool-u8', length='u16-le', peer_presence='bool-u8',
                         peer_address='host-order-u32-le-ipv4', peer_port='host-order-u16-le',
                         data=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}'), policy=POLICY),
        dict(id=first_id + 14, name='network.socket_is_pending', arguments=[handle],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 15, name='network.socket_reuse_address', arguments=[handle],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 16, name='network.socket_send_to',
             arguments=[handle,
                        dict(type=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}', name='data'),
                        dict(type='host-order-u32-le-ipv4', name='address'),
                        dict(type='host-order-u16-le', name='port')],
             result=dict(presence='bool-u8', length='u16-le'), policy=POLICY),
    ]
    return dict(methods=methods, transfer_max=TRANSFER_MAX, poll_timeout_max_ms=POLL_TIMEOUT_MAX_MS)


def rust(spec, notice):
    (open_, close, bind, listen, blocking, connected, pollin, pollout,
     accept, recv, send, sendfile, connect, recv_from, pending, reuse_address, send_to) = spec['methods']
    return notice + render(
        'rust/network.rs',
        transfer_max=TRANSFER_MAX,
        poll_timeout_max_ms=POLL_TIMEOUT_MAX_MS,
        close_id=close['id'],
        path_size=PATH_SIZE,
        n4_plus_path_size_plus_2=4 + PATH_SIZE + 2,
        n4_plus_path_size=4 + PATH_SIZE,
        bind_id=bind['id'],
        connect_id=connect['id'],
        listen_id=listen['id'],
        blocking_id=blocking['id'],
        connected_id=connected['id'],
        pending_id=pending['id'],
        reuse_address_id=reuse_address['id'],
        pollin_id=pollin['id'],
        pollout_id=pollout['id'],
        accept_id=accept['id'],
        transfer_max_plus_3=TRANSFER_MAX + 3,
        recv_id=recv['id'],
        transfer_max_plus_10=TRANSFER_MAX + 10,
        recv_from_id=recv_from['id'],
        transfer_max_plus_6=TRANSFER_MAX + 6,
        send_id=send['id'],
        transfer_max_plus_12=TRANSFER_MAX + 12,
        send_to_id=send_to['id'],
        sendfile_id=sendfile['id'],
        open_id=open_['id'],
    )
