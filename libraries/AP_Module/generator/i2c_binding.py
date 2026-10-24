# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Lifecycle-owned I2C devices for native modules."""

import call_model as model
import rust_calls

OWNER = 'i2c'
DEVICE = 'AP_HAL::I2CDevice'
TRANSFER_MAX = 255
POLICY = ('v1:module-context-owned-devices,cleanup-after-worker-join,max4-devices-per-module,'
          'bus-0-to4,address-0-to128,transfer-max255,blocking-device-lock,'
          'copy-output-on-success,no-retained-module-pointers')


def _shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = dict(rename=None, depends=None, get=None, literal=False, reference=False,
                          semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected_owner.items()):
        raise ValueError('unaudited i2c owner metadata')
    aliases = [item for item in owner['aliases'] if item['alias'] == 'get_device']
    if aliases != [dict(name='lua_get_i2c_device', alias='get_device', depends=None, manual=True)]:
        raise ValueError('unaudited i2c.get_device declaration')

    device = next((item for item in ir['ap_objects'] if item['name'] == DEVICE), None)
    expected_device = dict(rename=None, depends=None, get=None, creation=None, creation_manual=False,
                           creation_args=0, has_fields=False, has_enums=False, operations=0,
                           operator_getter=None, operators_manual=False, operator_names=[], literal=False,
                           reference=False, semaphore=False, scheduler_semaphore=False,
                           semaphore_pointer=True)
    if device is None or any(device[key] != value for key, value in expected_device.items()):
        raise ValueError('unaudited I2CDevice metadata')
    expected = {
        'set_retries': (('void', None, False, False, False, False, None),
                        [('uint8_t', None, False, False, False, False, ('0', '20'))]),
        'set_address': (('void', None, False, False, False, False, None),
                        [('uint8_t', None, False, False, False, True, None)]),
        'write_register': (('boolean', None, False, False, False, False, None),
                           [('uint8_t', None, False, False, False, True, None),
                            ('uint8_t', None, False, False, False, True, None)]),
    }
    if {method['name'] for method in device['methods']} != set(expected):
        raise ValueError('unaudited I2CDevice methods')
    for method in device['methods']:
        actual = (_shape(method['return']), [_shape(argument) for argument in method['arguments']])
        if (method['rename'] is not None or method['depends'] is not None or
                method['deprecate'] is not None or actual != expected[method['name']]):
            raise ValueError(f'unaudited I2CDevice.{method["name"]} declaration')
    expected_aliases = {
        ('AP_HAL__I2CDevice_read_registers', 'read_registers', None, True),
        ('AP_HAL__I2CDevice_transfer', 'transfer', None, True),
    }
    if {(item['name'], item['alias'], item['depends'], item['manual']) for item in device['aliases']} != expected_aliases:
        raise ValueError('unaudited I2CDevice manual aliases')
    return specification(first_id)


def specification(first_id):
    names = ['open', 'close', 'set_retries', 'set_address', 'write_register', 'read_registers', 'transfer']
    arguments = [
        [dict(type='u8', range=[0, 4], name='bus'), dict(type='u8', range=[0, 128], name='address'),
         dict(type='u32-le', name='bus_clock'), dict(type='bool-u8', name='use_smbus')],
        [dict(type='nonzero-u32-le', name='handle')],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', range=[0, 20], name='retries')],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', name='address')],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', name='register'),
         dict(type='u8', name='value')],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', name='first_register'),
         dict(type='u8', range=[1, TRANSFER_MAX], name='receive_length')],
        [dict(type='nonzero-u32-le', name='handle'),
         dict(type=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}', name='send'),
         dict(type='u8', name='receive_length')],
    ]
    results = ['optional-nonzero-u32-le', 'void', 'void', 'void', 'bool-u8',
               f'optional-bytes-max{TRANSFER_MAX}', f'optional-bytes-max{TRANSFER_MAX}']
    return dict(methods=[dict(id=first_id + i, name=f'i2c.{name}', arguments=arguments[i],
                              result=results[i], policy=POLICY) for i, name in enumerate(names)],
                transfer_max=TRANSFER_MAX)


def client_model(spec):
    ids = {m['name'].rsplit('.', 1)[1]: m['id'] for m in spec['methods']}
    device = model.Handle('I2cDevice', 'closeable-token')
    field = model.Field
    send = model.Buffer(TRANSFER_MAX)
    receive = model.Buffer(TRANSFER_MAX, 'out')
    calls = [model.Call('i2c.open', ids['open'], (
        field('bus', model.U8, 0, 4), field('address', model.U8, 0, 128),
        field('bus_clock', model.U32), field('use_smbus', model.BOOL)),
        model.OptionalValue(device), method_name='get_device')]
    for name, arguments in (
            ('close', ()), ('set_retries', (field('retries', model.U8, 0, 20),)),
            ('set_address', (field('address', model.U8),)),
            ('write_register', (field('register', model.U8), field('value', model.U8)))):
        calls.append(model.Call('i2c.' + name, ids[name], arguments,
                                model.BOOL if name == 'write_register' else None,
                                receiver=device, consume=name == 'close'))
    calls += [
        model.Call('i2c.read_registers', ids['read_registers'],
                   (field('first_register', model.U8), field('receive', model.Buffer(TRANSFER_MAX, 'out', minimum=1))),
                   model.BufferResult('receive', TRANSFER_MAX), receiver=device,
                   request=(field('first_register', model.U8), field('receive_length', model.Length('receive', model.U8)))),
        model.Call('i2c.transfer', ids['transfer'], (field('send', send), field('receive', receive)),
                   model.BufferResult('receive', TRANSFER_MAX), receiver=device,
                   request=(field('send_length', model.Length('send', model.U8)),
                            field('receive_length', model.Length('receive', model.U8)), field('send', send))),
    ]
    return model.Module(tuple(calls), (device,), constants=(('I2C_TRANSFER_MAX', TRANSFER_MAX),))


def rust(spec, notice):
    return rust_calls.emit(client_model(spec), notice)
