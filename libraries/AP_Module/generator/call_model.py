# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Language-independent layouts and public call shapes for audited adapters.

This supplements the compatibility catalogue: it describes how a public result
maps to wire bytes, including selectors returned by presence-only replies.
It contains no Rust/C++ source, compiler layouts or runtime codec machinery.
"""

import re

from dataclasses import dataclass


def identifier(name):
    if not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', name):
        raise ValueError('invalid call-model identifier: ' + name)
    return name


@dataclass(frozen=True)
class Scalar:
    name: str
    size: int

    def __post_init__(self):
        sizes = {'u8': 1, 'i8': 1, 'u16': 2, 'i16': 2, 'u32': 4, 'i32': 4, 'u64': 8, 'f32': 4, 'bool': 1}
        if sizes.get(self.name) != self.size:
            raise ValueError('unsupported scalar layout: ' + self.name)

    @property
    def bounds(self):
        if self.name == 'f32':
            return -3.4028234663852886e38, 3.4028234663852886e38
        bits = self.size * 8
        signed = self.name.startswith('i')
        return (-(1 << (bits - 1)) if signed else 0), (1 << (bits - int(signed))) - 1


U8 = Scalar('u8', 1)
I8 = Scalar('i8', 1)
U16 = Scalar('u16', 2)
I16 = Scalar('i16', 2)
U32 = Scalar('u32', 4)
I32 = Scalar('i32', 4)
U64 = Scalar('u64', 8)
F32 = Scalar('f32', 4)
BOOL = Scalar('bool', 1)


@dataclass(frozen=True)
class Enum:
    name: str
    scalar: Scalar
    values: tuple = ()
    default: object = None
    external: bool = False

    def __post_init__(self):
        identifier(self.name)
        if self.scalar in (F32, BOOL):
            raise ValueError('enum needs an integer encoding')
        if not self.external and not self.values:
            raise ValueError('closed enum needs values')
        names = [identifier(name) for name, _ in self.values]
        values = [value for _, value in self.values]
        bits = self.scalar.size * 8
        minimum = -(1 << (bits - 1)) if self.scalar.name.startswith('i') else 0
        maximum = (1 << (bits - int(minimum != 0))) - 1
        if (len(set(names)) != len(names) or len(set(values)) != len(values) or
                any(type(value) is not int or not minimum <= value <= maximum for value in values)):
            raise ValueError('invalid enum members: ' + self.name)
        if self.default is not None and self.default not in names:
            raise ValueError('unknown default enum member')

    @property
    def size(self):
        return self.scalar.size


@dataclass(frozen=True)
class Field:
    name: str
    type: object
    minimum: object = None
    maximum: object = None

    def __post_init__(self):
        identifier(self.name)
        if (self.minimum is None) != (self.maximum is None):
            raise ValueError('call-model ranges need both bounds')
        if self.minimum is not None and self.minimum > self.maximum:
            raise ValueError('inverted call-model range')
        if self.minimum is not None:
            if not isinstance(self.type, Scalar) or self.type == BOOL:
                raise ValueError('call-model range needs a numeric scalar')
            bound_type = float if self.type == F32 else int
            low, high = self.type.bounds
            if any(type(value) is not bound_type or not low <= value <= high
                   for value in (self.minimum, self.maximum)):
                raise ValueError('call-model range does not fit ' + self.type.name)


@dataclass(frozen=True)
class Record:
    name: str
    fields: tuple
    external: bool = False

    def __post_init__(self):
        identifier(self.name)
        if len({f.name for f in self.fields}) != len(self.fields):
            raise ValueError('duplicate record field: ' + self.name)
        if any(not isinstance(f.type, (Scalar, Enum, Record)) for f in self.fields):
            raise ValueError('unsupported record field: ' + self.name)

    @property
    def size(self):
        return sum(f.type.size for f in self.fields)


@dataclass(frozen=True)
class Handle:
    name: str
    ownership: str
    docs: tuple = ()

    def __post_init__(self):
        identifier(self.name)
        if self.ownership not in ('borrowed-index', 'borrowed-token', 'context-token', 'closeable-token'):
            raise ValueError('unsupported handle policy: ' + self.ownership)

    @property
    def scalar(self):
        return U8 if self.ownership == 'borrowed-index' else U32

    @property
    def size(self):
        return self.scalar.size


@dataclass(frozen=True)
class OptionalValue:
    type: object
    nonzero: bool = False

    def __post_init__(self):
        if not isinstance(self.type, (Scalar, Enum, Record, Handle)):
            raise ValueError('unsupported optional value type')
        if isinstance(self.type, Handle) and self.type.ownership == 'borrowed-index':
            raise ValueError('borrowed-index optionals need an explicit selected input')
        if self.nonzero and (not isinstance(self.type, Scalar) or self.type in (F32, BOOL)):
            raise ValueError('nonzero optional needs an integer scalar')

    @property
    def size(self):
        # Owned tokens reserve zero; other values have a separate presence byte.
        return self.type.size + (0 if isinstance(self.type, Handle) else 1)


@dataclass(frozen=True)
class SelectedHandle:
    """A bool-u8 reply selects an index already carried by the request."""
    type: Handle
    argument: str

    def __post_init__(self):
        if self.type.ownership != 'borrowed-index':
            raise ValueError('presence-only result needs a borrowed index')

    @property
    def size(self):
        return 1


@dataclass(frozen=True)
class Buffer:
    capacity: int
    direction: str = 'in'
    minimum: int = 0
    empty_returns_zero: bool = False

    def __post_init__(self):
        if (type(self.capacity) is not int or self.capacity <= 0 or
                type(self.minimum) is not int or not 0 <= self.minimum <= self.capacity or
                self.direction not in ('in', 'out') or
                self.empty_returns_zero and self.minimum != 0):
            raise ValueError('invalid bounded buffer')

    @property
    def size(self):
        return self.capacity


@dataclass(frozen=True)
class String:
    capacity: int
    nonempty: bool = False

    def __post_init__(self):
        if type(self.capacity) is not int or self.capacity < 1:
            raise ValueError('invalid bounded string')

    @property
    def size(self):
        return self.capacity


@dataclass(frozen=True)
class Length:
    buffer: str
    scalar: Scalar

    def __post_init__(self):
        identifier(self.buffer)
        if self.scalar not in (U8, U16, U32):
            raise ValueError('buffer length needs an unsigned scalar')

    @property
    def size(self):
        return self.scalar.size


@dataclass(frozen=True)
class BufferResult:
    """Count replies require a canonical zero tail before partial copyback.

    Presence replies copy the requested extent only when present; ignored
    payload/tail bytes are unspecified, matching the existing I2C contract.
    """
    buffer: str
    capacity: int
    length: object = None

    def __post_init__(self):
        identifier(self.buffer)
        if type(self.capacity) is not int or self.capacity <= 0 or self.length not in (None, U8, U16, U32):
            raise ValueError('invalid buffer reply')
        if self.length is not None and self.capacity > self.length.bounds[1]:
            raise ValueError('buffer reply does not fit its length')

    @property
    def size(self):
        return self.capacity + (self.length.size if self.length else 1)


@dataclass(frozen=True)
class BoundedCount:
    scalar: Scalar
    buffer: str

    @property
    def size(self):
        return self.scalar.size


def layout(fields):
    offset = 0
    result = []
    for field in fields:
        result.append((field, offset))
        offset += field.type.size
    return result


@dataclass(frozen=True)
class Call:
    name: str
    binding_id: int
    arguments: tuple = ()
    result: object = None
    receiver: object = None
    method_name: str = ''
    namespace: str = ''
    consume: bool = False
    docs: tuple = ()
    request: object = None

    def __post_init__(self):
        owner, dot, method = self.name.partition('.')
        identifier(owner)
        identifier(method)
        if not dot or type(self.binding_id) is not int or not 0 <= self.binding_id <= 0xffffffff:
            raise ValueError('invalid binding call')
        if self.method_name:
            identifier(self.method_name)
        if self.namespace:
            identifier(self.namespace)
        if self.result is not None and not isinstance(
                self.result, (Scalar, Enum, Record, OptionalValue, SelectedHandle, BufferResult, BoundedCount)):
            raise ValueError('unsupported call result: ' + self.name)
        if (len({a.name for a in self.arguments}) != len(self.arguments) or
                any(a.name in ('receiver', 'context', 'self') for a in self.arguments)):
            raise ValueError('duplicate or reserved call argument: ' + self.name)
        if any(not isinstance(a.type, (Scalar, Enum, Record, Buffer, String, OptionalValue)) for a in self.arguments):
            raise ValueError('unsupported call argument: ' + self.name)
        if any(isinstance(a.type, OptionalValue) and not isinstance(a.type.type, (Scalar, Enum, Record))
               for a in self.arguments):
            raise ValueError('unsupported optional call argument: ' + self.name)
        arguments = {a.name: a.type for a in self.arguments}
        used = set()
        for field in self.inputs:
            if field.name == 'receiver':
                continue
            if isinstance(field.type, Length):
                buffer = arguments.get(field.type.buffer)
                if not isinstance(buffer, Buffer) or buffer.capacity > field.type.scalar.bounds[1]:
                    raise ValueError('invalid buffer length input: ' + self.name)
                used.add(field.type.buffer)
            elif field.name not in arguments or arguments[field.name] != field.type:
                raise ValueError('request field does not match argument: ' + self.name)
            else:
                if isinstance(field.type, Buffer) and field.type.direction != 'in':
                    raise ValueError('output buffer cannot be encoded as input')
                used.add(field.name)
        if used != set(arguments) or len({f.name for f in self.inputs}) != len(self.inputs):
            raise ValueError('incomplete or duplicate request layout: ' + self.name)
        if isinstance(self.result, BufferResult):
            buffer = arguments.get(self.result.buffer)
            if not isinstance(buffer, Buffer) or buffer.direction != 'out' or buffer.capacity != self.result.capacity:
                raise ValueError('reply needs a matching output buffer')
        if isinstance(self.result, BoundedCount):
            buffer = arguments.get(self.result.buffer)
            if (self.result.scalar not in (U8, U16, U32) or not isinstance(buffer, Buffer) or
                    buffer.direction != 'in' or buffer.capacity > self.result.scalar.bounds[1]):
                raise ValueError('count needs a matching input buffer')
        if (isinstance(self.result, BoundedCount) or
                isinstance(self.result, BufferResult) and self.result.length):
            lengths = [f for f in self.inputs if isinstance(f.type, Length) and f.type.buffer == self.result.buffer]
            if len(lengths) != 1:
                raise ValueError('count reply needs one matching request length')
        if any(isinstance(value, Buffer) and value.direction == 'out' and
               (not isinstance(self.result, BufferResult) or self.result.buffer != name)
               for name, value in arguments.items()):
            raise ValueError('output buffer needs a matching reply')
        for name, buffer in arguments.items():
            if isinstance(buffer, Buffer) and buffer.empty_returns_zero:
                if (not isinstance(self.result, (BoundedCount, BufferResult)) or self.result.buffer != name or
                        isinstance(self.result, BufferResult) and not self.result.length):
                    raise ValueError('empty zero return needs a matching count result')
        if isinstance(self.result, SelectedHandle):
            if not any(a.name == self.result.argument and a.type == U8 for a in self.arguments):
                raise ValueError('presence-only selector must name a u8 input')
        if self.consume and (self.receiver is None or self.receiver.ownership != 'closeable-token' or self.result is not None):
            raise ValueError('close needs a closeable receiver and void result')

    @property
    def inputs(self):
        receiver = (Field('receiver', self.receiver.scalar),) if self.receiver else ()
        return receiver + (self.arguments if self.request is None else self.request)

    @property
    def input_size(self):
        return sum(f.type.size for f in self.inputs)

    @property
    def output_size(self):
        return self.result.size if self.result else 0


@dataclass(frozen=True)
class Projection:
    owner: str
    name: str
    method: str
    field: str
    type: object


@dataclass(frozen=True)
class Module:
    calls: tuple
    types: tuple = ()
    projections: tuple = ()
    constants: tuple = ()
    byte_methods: tuple = ()

    def __post_init__(self):
        names = [call.name for call in self.calls]
        if len(set(names)) != len(names):
            raise ValueError('duplicate modeled binding')
        if len({t.name for t in self.types}) != len(self.types):
            raise ValueError('duplicate modeled type')


def calls(spec):
    """Collect migrated adapter models without changing the contract serializer."""
    # Local imports avoid cycles while selectors and their clients migrate.
    import airspeed_binding
    import analog_pwm_binding
    import efi_binding
    import i2c_binding
    import old_parameter_binding
    import onvif_binding
    import periph_binding
    import rangefinder_proximity_binding
    import serial_binding
    import serialled_binding

    result = {}
    for name, adapter in (
            ('airspeed', airspeed_binding), ('analog_pwm', analog_pwm_binding), ('efi', efi_binding),
            ('old_parameter', old_parameter_binding), ('onvif', onvif_binding), ('periph', periph_binding),
            ('rangefinder_proximity', rangefinder_proximity_binding), ('serialled', serialled_binding),
            ('serial', serial_binding), ('i2c', i2c_binding)):
        if spec.get(name) is not None:
            for call in adapter.client_model(spec[name]).calls:
                if call.name in result:
                    raise ValueError('duplicate modeled binding: ' + call.name)
                result[call.name] = call
    return result
