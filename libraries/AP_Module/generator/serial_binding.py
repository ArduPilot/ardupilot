# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Revalidated, non-owning serial-port selectors and bounded transfers."""

import call_model as model
import rust_calls

TRANSFER_MAX = 256
OWNER = 'serial'
USERDATA = 'AP_Scripting_SerialAccess'
POLICY = ('v1:non-owning-selector,revalidate-every-call,simulated-generation-checked,'
          'shared-stream-with-lua-and-firmware,no-exclusive-ownership,no-config-rollback-on-exit,'
          'blocking-binding-lock,transfer-max256,fixed-zero-padded-wire,no-retained-module-pointers')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    if owner is None:
        raise ValueError('missing serial singleton')
    expected_owner = dict(rename=None, depends='AP_SERIALMANAGER_ENABLED', get=None, literal=False,
                          reference=False, semaphore=False, scheduler_semaphore=False,
                          semaphore_pointer=False)
    if any(owner[key] != value for key, value in expected_owner.items()) or owner['methods']:
        raise ValueError('unaudited serial owner metadata')
    expected_aliases = {
        'find_serial': dict(name='lua_serial_find_serial', alias='find_serial', depends=None, manual=True),
        'find_simulated_device': dict(name='lua_serial_find_simulated_device', alias='find_simulated_device',
                                      depends='AP_SCRIPTING_SERIALDEVICE_ENABLED', manual=True),
    }
    if {item['alias']: item for item in owner['aliases']} != expected_aliases:
        raise ValueError('unaudited serial aliases')

    userdata = next((item for item in ir['userdata'] if item['name'] == USERDATA), None)
    expected_userdata = dict(rename=None, depends=None, get=None, creation='null', creation_manual=True,
                             creation_args=-1, has_fields=False, has_enums=False, operations=0,
                             operator_getter=None, operators_manual=False, operator_names=[], literal=False,
                             reference=False, semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if userdata is None or any(userdata[key] != value for key, value in expected_userdata.items()):
        raise ValueError('unaudited serial userdata metadata')

    def field_shape(field):
        return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
                field['skip_check'], tuple(field['range']) if field['range'] else None)

    def shape(result, arguments):
        return (result, None, False, False, False, False, None), arguments

    expected_methods = {
        'available': shape('uint32_t', []),
        'configure_parity': shape('void', [('uint8_t', None, False, False, False, False, ('0', '2'))]),
        'read': shape('int16_t', []),
        'set_flow_control': shape('void', [('enum', 'AP_HAL::UARTDriver::flow_control', False, False, False,
                                            False, ('AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE',
                                                    'AP_HAL::UARTDriver::FLOW_CONTROL_RTS_DE'))]),
        'set_stop_bits': shape('void', [('uint8_t', None, False, False, False, False, ('1', '2'))]),
        'set_unbuffered_writes': shape('void', [('boolean', None, False, False, False, False, None)]),
        'write': shape('uint32_t', [('uint8_t', None, False, False, False, True, None)]),
    }
    actual_methods = {
        method['name']: (field_shape(method['return']), [field_shape(arg) for arg in method['arguments']])
        for method in userdata['methods']
        if method['rename'] is None and method['depends'] is None and method['deprecate'] is None
    }
    if actual_methods != expected_methods:
        raise ValueError('unaudited serial userdata methods')
    expected_userdata_aliases = {
        'begin': 'lua_serial_begin', 'readstring': 'lua_serial_readstring',
        'writestring': 'lua_serial_writestring',
    }
    actual_aliases = {item['alias']: item['name'] for item in userdata['aliases']
                      if item['manual'] and item['depends'] is None}
    if len(actual_aliases) != len(userdata['aliases']) or actual_aliases != expected_userdata_aliases:
        raise ValueError('unaudited serial userdata aliases')
    return specification(first_id)


def specification(first_id):
    handle = dict(type='u32-le,nonzero,revalidated-serial-selector', direction='borrowed-receiver')
    optional_handle = dict(presence='u32-le,zero-is-none', handle='SerialPort')
    methods = [
        dict(id=first_id, name='Serial.find_serial',
             arguments=[dict(type='u8', direction='in', name='instance')],
             result=optional_handle, policy=POLICY),
        dict(id=first_id + 1, name='Serial.find_simulated_device',
             arguments=[dict(type='u7', direction='in', name='protocol'),
                        dict(type='u16-le', direction='in', name='instance')],
             result=optional_handle, policy=POLICY),
        dict(id=first_id + 2, name='SerialPort.begin',
             arguments=[handle, dict(type='optional-nonzero-u32-le', direction='in', name='baud')],
             result='void', policy=POLICY),
        dict(id=first_id + 3, name='SerialPort.configure_parity',
             arguments=[handle, dict(type='u8', direction='in', range=[0, 2])],
             result='void', policy=POLICY),
        dict(id=first_id + 4, name='SerialPort.set_stop_bits',
             arguments=[handle, dict(type='u8', direction='in', range=[1, 2])],
             result='void', policy=POLICY),
        dict(id=first_id + 5, name='SerialPort.set_unbuffered_writes',
             arguments=[handle, dict(type='bool-u8', direction='in')], result='void', policy=POLICY),
        dict(id=first_id + 6, name='SerialPort.set_flow_control',
             arguments=[handle, dict(type='u8', direction='in', range=[0, 3])],
             result='void', policy=POLICY),
        dict(id=first_id + 7, name='SerialPort.available', arguments=[handle],
             result='u32-le', policy=POLICY),
        dict(id=first_id + 8, name='SerialPort.read',
             arguments=[handle, dict(type='u16-le', direction='in', range=[1, TRANSFER_MAX])],
             result=dict(length='u16-le', data=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}'),
             policy=POLICY),
        dict(id=first_id + 9, name='SerialPort.write',
             arguments=[handle, dict(type=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}', direction='in')],
             result='u16-le', policy=POLICY),
    ]
    return dict(methods=methods, transfer_max=TRANSFER_MAX)


def client_model(spec):
    (find, simulated, begin, parity, stop_bits, unbuffered, flow_control,
     available, read, write) = spec['methods']
    port = model.Handle('SerialPort', 'borrowed-token', docs=(
        'A non-owning serial selector, revalidated by firmware on every call.',
        'Clones share the byte stream with firmware and Lua. Configuration is not rolled back on exit.'))
    parity_type = model.Enum('SerialParity', model.U8, (('None', 0), ('Odd', 1), ('Even', 2)))
    stop_type = model.Enum('SerialStopBits', model.U8, (('One', 1), ('Two', 2)))
    flow_type = model.Enum('SerialFlowControl', model.U8, (('Disabled', 0), ('Enabled', 1), ('Auto', 2), ('RtsDe', 3)))
    field = model.Field
    calls = [
        model.Call(find['name'], find['id'], (field('instance', model.U8),), model.OptionalValue(port),
                   docs=('Find the nth physical UART configured for SERIAL_PROTOCOL=28.',)),
        model.Call(simulated['name'], simulated['id'], (field('protocol', model.U8, 0, 127), field('instance', model.U16)),
                   model.OptionalValue(port), docs=('Find the nth shared SCR_SDEV port configured for the protocol.',)),
        model.Call(begin['name'], begin['id'], (field('baud', model.OptionalValue(model.U32, nonzero=True)),),
                   receiver=port, docs=('Start a physical UART; None reapplies its configured baud rate.',
                                        'Beginning a simulated-device view is a validated no-op, matching Lua.')),
        model.Call(parity['name'], parity['id'], (field('parity', parity_type),), receiver=port),
        model.Call(stop_bits['name'], stop_bits['id'], (field('stop_bits', stop_type),), receiver=port),
        model.Call(unbuffered['name'], unbuffered['id'], (field('enabled', model.BOOL),), receiver=port),
        model.Call(flow_control['name'], flow_control['id'], (field('mode', flow_type),), receiver=port),
        model.Call(available['name'], available['id'], result=model.U32, receiver=port),
        model.Call(read['name'], read['id'],
                   (field('buffer', model.Buffer(TRANSFER_MAX, 'out', empty_returns_zero=True)),),
                   model.BufferResult('buffer', TRANSFER_MAX, model.U16), receiver=port,
                   request=(field('length', model.Length('buffer', model.U16)),),
                   docs=('Nonblocking read. Only returned bytes are changed; an empty buffer makes no host call.',)),
        model.Call(write['name'], write['id'],
                   (field('buffer', model.Buffer(TRANSFER_MAX, empty_returns_zero=True)),),
                   model.BoundedCount(model.U16, 'buffer'), receiver=port,
                   request=(field('length', model.Length('buffer', model.U16)), field('buffer', model.Buffer(
                       TRANSFER_MAX, empty_returns_zero=True))),
                   docs=('Nonblocking write. Partial writes are reported; an empty buffer makes no host call.',)),
    ]
    return model.Module(tuple(calls), (port, parity_type, stop_type, flow_type),
                        constants=(('SERIAL_TRANSFER_MAX', TRANSFER_MAX),),
                        byte_methods=((port.name, 'read_byte', 'read', 'out'), (port.name, 'write_byte', 'write', 'in')))


def rust(spec, notice):
    return rust_calls.emit(client_model(spec), notice)
