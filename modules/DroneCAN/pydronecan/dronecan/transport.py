#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

from __future__ import division, absolute_import, print_function, unicode_literals
import sys
import time
import math
import copy
import struct
import functools

try:
    import collections.abc  # Python 3
    MutableSequence = collections.abc.MutableSequence
except ImportError:
    import collections  # Python 2
    MutableSequence = collections.MutableSequence

import dronecan
import dronecan.dsdl as dsdl
import dronecan.dsdl.common as common


try:
    long        # Python 2
except NameError:
    long = int  # Python 3

if sys.version_info[0] < 3:
    bchr = chr
else:
    def bchr(x):
        return bytes([x])


def get_dronecan_data_type(obj):
    # noinspection PyProtectedMember
    return obj._type


def is_union(obj):
    if not isinstance(obj, CompoundValue):
        raise ValueError('Only CompoundValue can be union')
    # noinspection PyProtectedMember
    return obj._is_union


def get_active_union_field(obj):
    if not is_union(obj):
        raise ValueError('Object is not a union')
    # noinspection PyProtectedMember
    return obj._union_field


def switch_union_field(obj, value):
    if not is_union(obj):
        raise ValueError('Object is not a union')
    # noinspection PyProtectedMember
    obj._union_field = value


def get_fields(obj):
    if not isinstance(obj, CompoundValue):
        raise ValueError('Only CompoundValue can have fields')
    # noinspection PyProtectedMember
    return obj._fields


def get_constants(obj):
    if not isinstance(obj, CompoundValue):
        raise ValueError('Only CompoundValue can have constants')
    # noinspection PyProtectedMember
    return obj._constants


def is_request(obj):
    # noinspection PyProtectedMember
    return obj._mode == 'request'


def is_response(obj):
    # noinspection PyProtectedMember
    return obj._mode == 'response'


def bits_from_bytes(s):
    return "".join(format(c, "08b") for c in s)


def bytes_from_bits(s):
    #pad bytes if not a multiple of 8
    if len(s) % 8 != 0:
        s += '0' * (8 - len(s) % 8)
    return bytearray(int(s[i:i + 8], 2) for i in range(0, len(s), 8))


def be_from_le_bits(s, bitlen):
    if len(s) < bitlen:
        raise ValueError("Not enough bits; need {0} but got {1}".format(bitlen, len(s)))
    elif len(s) > bitlen:
        s = s[0:bitlen]

    return "".join([s[i:i + 8] for i in range(0, len(s), 8)][::-1])


def le_from_be_bits(s, bitlen):
    if len(s) < bitlen:
        raise ValueError("Not enough bits; need {0} but got {1}".format(bitlen, len(s)))
    elif len(s) > bitlen:
        s = s[len(s) - bitlen:]

    return "".join([s[max(0, i - 8):i] for i in range(len(s), 0, -8)])


def format_bits(s):
    return " ".join(s[i:i + 8] for i in range(0, len(s), 8))


def union_tag_bits_from_num_elements(num_elements):
    return int(math.ceil(math.log(num_elements, 2)))

def array_len_bits_from_max_size(max_size):
    return int(math.ceil(math.log(max_size+1, 2)))

def enum_mark_last(iterable, start=0):
    """
    Returns a generator over iterable that tells whether the current item is the last one.
    Usage:
        >>> iterable = range(10)
        >>> for index, is_last, item in enum_mark_last(iterable):
        >>>     print(index, item, end='\n' if is_last else ', ')
    """
    it = iter(iterable)
    count = start
    try:
        last = next(it)
    except StopIteration:
        return
    for val in it:
        yield count, False, last
        last = val
        count += 1
    yield count, True, last


class Float32IntegerUnion(object):
    """
    Yes we've got ourselves a tiny little union here:
        union FloatIntegerUnion
        {
            std::uint32_t u;
            float f;
        };
    This is madness.
    """

    def __init__(self, integer=None, floating_point=None):
        self._bytes = struct.pack("=L", 0)
        if integer is not None:
            assert floating_point is None
            self.u = int(integer)
        if floating_point is not None:
            self.f = float(floating_point)

    @property
    def f(self):
        return struct.unpack("=f", self._bytes)[0]

    @f.setter
    def f(self, value):
        assert isinstance(value, float)
        self._bytes = struct.pack("=f", value)

    @property
    def u(self):
        return struct.unpack("=I", self._bytes)[0]

    @u.setter
    def u(self, value):
        assert isinstance(value, (int, long))
        self._bytes = struct.pack("=I", value)


def f16_from_f32(float32):
    # Directly translated from libuavcan's implementation in C++
    f32infty = Float32IntegerUnion(integer=255 << 23)
    f16infty = Float32IntegerUnion(integer=31 << 23)
    magic = Float32IntegerUnion(integer=15 << 23)
    inval = Float32IntegerUnion(floating_point=float32)
    sign_mask = 0x80000000
    round_mask = ~0xFFF

    sign = inval.u & sign_mask
    inval.u ^= sign

    if inval.u >= f32infty.u:                           # Inf or NaN (all exponent bits set)
        out = 0x7FFF if inval.u > f32infty.u else 0x7C00
    else:
        inval.u &= round_mask
        inval.f *= magic.f
        inval.u -= round_mask
        if inval.u > f16infty.u:
            inval.u = f16infty.u                        # Clamp to signed infinity if overflowed
        out = (inval.u >> 13) & 0xFFFF                  # Take the bits!

    return out | (sign >> 16) & 0xFFFF


def f32_from_f16(float16):
    # Directly translated from libuavcan's implementation in C++
    magic = Float32IntegerUnion(integer=(254 - 15) << 23)
    was_inf_nan = Float32IntegerUnion(integer=(127 + 16) << 23)

    out = Float32IntegerUnion(integer=(float16 & 0x7FFF) << 13)     # exponent/mantissa bits
    out.f *= magic.f                                                # exponent adjust
    if out.f >= was_inf_nan.f:                                      # make sure Inf/NaN survive
        out.u |= 255 << 23
    out.u |= (float16 & 0x8000) << 16                               # sign bit

    return out.f


def cast(value, dtype):
    if dtype.cast_mode == dsdl.PrimitiveType.CAST_MODE_SATURATED:
        if value > dtype.value_range[1]:
            value = dtype.value_range[1]
        elif value < dtype.value_range[0]:
            value = dtype.value_range[0]
        return value
    elif dtype.cast_mode == dsdl.PrimitiveType.CAST_MODE_TRUNCATED and dtype.kind == dsdl.PrimitiveType.KIND_FLOAT:
        if not math.isnan(value) and value > dtype.value_range[1]:
            value = float("+inf")
        elif not math.isnan(value) and value < dtype.value_range[0]:
            value = float("-inf")
        return value
    elif dtype.cast_mode == dsdl.PrimitiveType.CAST_MODE_TRUNCATED:
        return value & ((1 << dtype.bitlen) - 1)
    else:
        raise ValueError("Invalid cast_mode: " + repr(dtype))


class BaseValue(object):
    # noinspection PyUnusedLocal
    def __init__(self, _dronecan_type, *_args, **_kwargs):
        self._type = _dronecan_type
        self._bits = None

    def _unpack(self, stream, tao):
        if self._type.bitlen:
            self._bits = be_from_le_bits(stream, self._type.bitlen)
            return stream[self._type.bitlen:]
        else:
            return stream

    def _pack(self, tao):
        if self._bits:
            return le_from_be_bits(self._bits, self._type.bitlen)
        else:
            return "0" * self._type.bitlen


class VoidValue(BaseValue):
    def _unpack(self, stream, tao):
        return stream[self._type.bitlen:]

    def _pack(self, tao):
        return "0" * self._type.bitlen


class PrimitiveValue(BaseValue):
    def __init__(self, _dronecan_type, *args, **kwargs):
        super(PrimitiveValue, self).__init__(_dronecan_type, *args, **kwargs)
        # Default initialization
        self.value = 0

    def __repr__(self):
        return repr(self.value)

    @property
    def value(self):
        if not self._bits:
            return None

        int_value = int(self._bits, 2)
        if self._type.kind == dsdl.PrimitiveType.KIND_BOOLEAN:
            return bool(int_value)
        elif self._type.kind == dsdl.PrimitiveType.KIND_UNSIGNED_INT:
            return int_value
        elif self._type.kind == dsdl.PrimitiveType.KIND_SIGNED_INT:
            if int_value >= (1 << (self._type.bitlen - 1)):
                int_value = -((1 << self._type.bitlen) - int_value)
            return int_value
        elif self._type.kind == dsdl.PrimitiveType.KIND_FLOAT:
            if self._type.bitlen == 16:
                return f32_from_f16(int_value)
            elif self._type.bitlen == 32:
                return struct.unpack("<f", struct.pack("<L", int_value))[0]
            elif self._type.bitlen == 64:
                return struct.unpack("<d", struct.pack("<Q", int_value))[0]
            else:
                raise ValueError('Bad float')

    @value.setter
    def value(self, new_value):
        if new_value is None:
            raise ValueError("Can't serialize a None value")
        elif self._type.kind == dsdl.PrimitiveType.KIND_BOOLEAN:
            self._bits = "1" if new_value else "0"
        elif self._type.kind == dsdl.PrimitiveType.KIND_UNSIGNED_INT:
            new_value = cast(new_value, self._type)
            self._bits = format(new_value, "0" + str(self._type.bitlen) + "b")
        elif self._type.kind == dsdl.PrimitiveType.KIND_SIGNED_INT:
            new_value = cast(new_value, self._type)
            if new_value < 0:                       # Computing two's complement for negatives
                new_value += 2 ** self._type.bitlen
            self._bits = format(new_value, "0" + str(self._type.bitlen) + "b")
        elif self._type.kind == dsdl.PrimitiveType.KIND_FLOAT:
            new_value = cast(new_value, self._type)
            if self._type.bitlen == 16:
                int_value = f16_from_f32(new_value)
            elif self._type.bitlen == 32:
                int_value = struct.unpack("<L", struct.pack("<f", new_value))[0]
            elif self._type.bitlen == 64:
                int_value = struct.unpack("<Q", struct.pack("<d", new_value))[0]
            else:
                raise ValueError('Bad float, no donut')
            self._bits = format(int_value, "0" + str(self._type.bitlen) + "b")


# noinspection PyProtectedMember
class ArrayValue(BaseValue, MutableSequence):
    def __init__(self, _dronecan_type, *args, **kwargs):
        super(ArrayValue, self).__init__(_dronecan_type, *args, **kwargs)

        if isinstance(self._type.value_type, dsdl.PrimitiveType):
            self.__item_ctor = functools.partial(PrimitiveValue, self._type.value_type)
        elif isinstance(self._type.value_type, dsdl.ArrayType):
            self.__item_ctor = functools.partial(ArrayValue, self._type.value_type)
        elif isinstance(self._type.value_type, dsdl.CompoundType):
            self.__item_ctor = functools.partial(CompoundValue, self._type.value_type)

        if self._type.mode == dsdl.ArrayType.MODE_STATIC:
            self.__items = list(self.__item_ctor() for _ in range(self._type.max_size))
        else:
            self.__items = []

    def __repr__(self):
        return "ArrayValue(type={0!r}, items={1!r})".format(self._type, self.__items)

    def __str__(self):
        if self._type.is_string_like:
            # noinspection PyBroadException
            try:
                return self.decode()
            except Exception:
                pass
        return self.__repr__()

    def __getitem__(self, idx):
        if isinstance(self.__items[idx], PrimitiveValue):
            return self.__items[idx].value if self.__items[idx]._bits else 0
        else:
            return self.__items[idx]

    def __setitem__(self, idx, value):
        if idx >= self._type.max_size:
            raise IndexError("Index {0} too large (max size {1})".format(idx, self._type.max_size))
        if isinstance(self._type.value_type, dsdl.PrimitiveType):
            self.__items[idx].value = value
        else:
            self.__items[idx] = value

    def __delitem__(self, idx):
        del self.__items[idx]

    def __len__(self):
        return len(self.__items)

    def __eq__(self, other):
        if isinstance(other, str):
            return self.decode() == other
        else:
            return list(self) == other

    def clear(self):
        try:
            while True:
                self.pop()
        except IndexError:
            pass

    def new_item(self):
        return self.__item_ctor()

    def insert(self, idx, value):
        if idx >= self._type.max_size:
            raise IndexError("Index {0} too large (max size {1})".format(idx, self._type.max_size))
        elif len(self) == self._type.max_size:
            raise IndexError("Array already full (max size {0})".format(self._type.max_size))
        if isinstance(self._type.value_type, dsdl.PrimitiveType):
            new_item = self.__item_ctor()
            new_item.value = value
            self.__items.insert(idx, new_item)
        else:
            self.__items.insert(idx, value)

    def _unpack(self, stream, tao):
        if self._type.mode == dsdl.ArrayType.MODE_STATIC:
            for _, last, i in enum_mark_last(range(self._type.max_size)):
                stream = self.__items[i]._unpack(stream, tao and last)

        elif tao and self._type.value_type.get_min_bitlen() >= 8:
            del self[:]
            while len(stream) >= 8:
                new_item = self.__item_ctor()
                stream = new_item._unpack(stream, False)
                self.__items.append(new_item)
            stream = ''

        else:
            del self[:]
            count_width = array_len_bits_from_max_size(self._type.max_size)
            count = int(be_from_le_bits(stream[0:count_width], count_width), 2)
            stream = stream[count_width:]
            for _, last, i in enum_mark_last(range(count)):
                new_item = self.__item_ctor()
                stream = new_item._unpack(stream, tao and last)
                self.__items.append(new_item)

        return stream

    def _pack(self, tao):
        self.__items = self.__items[:self._type.max_size]   # Constrain max len

        if self._type.mode == dsdl.ArrayType.MODE_STATIC:
            while len(self) < self._type.max_size:              # Constrain min len
                self.__items.append(self.new_item())
            return ''.join(i._pack(tao and last) for _, last, i in enum_mark_last(self.__items))

        elif tao and self._type.value_type.get_min_bitlen() >= 8:
            return ''.join(i._pack(False) for i in self.__items)

        else:
            count_width = array_len_bits_from_max_size(self._type.max_size)
            count = le_from_be_bits(format(len(self), '0{0:1d}b'.format(count_width)), count_width)
            return count + ''.join(i._pack(tao and last) for _, last, i in enum_mark_last(self.__items))

    def from_bytes(self, value):
        del self[:]
        for byte in bytearray(value):
            self.append(byte)

    def to_bytes(self):
        return bytes(bytearray(item.value for item in self.__items if item._bits))

    def encode(self, value, errors='strict'):
        if not self._type.is_string_like:
            raise ValueError('encode() can be used only with string-like arrays')
        del self[:]
        value = bytearray(value, encoding="utf-8", errors=errors)
        for byte in value:
            self.append(byte)

    def decode(self, encoding="utf-8"):
        if not self._type.is_string_like:
            raise ValueError('decode() can be used only with string-like arrays')
        return bytearray(item.value for item in self.__items if item._bits).decode(encoding)


# noinspection PyProtectedMember
class CompoundValue(BaseValue):
    def __init__(self, _dronecan_type, _mode=None, *args, **kwargs):
        self.__dict__["_fields"] = collections.OrderedDict()
        self.__dict__["_constants"] = {}
        super(CompoundValue, self).__init__(_dronecan_type, *args, **kwargs)

        if self._type.kind == dsdl.CompoundType.KIND_SERVICE:
            if _mode == "request":
                source_fields = self._type.request_fields
                source_constants = self._type.request_constants
                self._is_union = self._type.request_union
            elif _mode == "response":
                source_fields = self._type.response_fields
                source_constants = self._type.response_constants
                self._is_union = self._type.response_union
            else:
                raise ValueError("mode must be either 'request' or 'response' for service types")
        else:
            if _mode is not None:
                raise ValueError("mode is not applicable for message types")
            source_fields = self._type.fields
            source_constants = self._type.constants
            self._is_union = self._type.union

        self._mode = _mode
        self._union_field = None

        for constant in source_constants:
            self._constants[constant.name] = constant.value

        for idx, field in enumerate(source_fields):
            if isinstance(field.type, dsdl.VoidType):
                self._fields["_void_{0}".format(idx)] = VoidValue(field.type)
            elif isinstance(field.type, dsdl.PrimitiveType):
                self._fields[field.name] = PrimitiveValue(field.type)
            elif isinstance(field.type, dsdl.ArrayType):
                self._fields[field.name] = ArrayValue(field.type)
            elif isinstance(field.type, dsdl.CompoundType):
                self._fields[field.name] = CompoundValue(field.type)

        for name, value in kwargs.items():
            if name.startswith('_'):
                raise NameError('%r is not a valid field name' % name)
            setattr(self, name, value)

    def __repr__(self):
        if self._is_union:
            field = self._union_field or list(self._fields.keys())[0]
            fields = "{0}={1!r}".format(field, self._fields[field])
        else:
            fields = ", ".join("{0}={1!r}".format(f, v) for f, v in self._fields.items() if not f.startswith("_void_"))
        return "{0}({1})".format(self._type.full_name, fields)

    def __copy__(self):
        # http://stackoverflow.com/a/15774013/1007777
        cls = self.__class__
        result = cls.__new__(cls)
        result.__dict__.update(self.__dict__)
        return result

    def __deepcopy__(self, memo):
        # http://stackoverflow.com/a/15774013/1007777
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            # noinspection PyArgumentList
            result.__dict__[k] = copy.deepcopy(v, memo)
        return result

    def __getattr__(self, attr):
        if attr in self._constants:
            return self._constants[attr]
        elif attr in self._fields:
            if self._is_union:
                if self._union_field and self._union_field != attr:
                    raise AttributeError(attr)
                else:
                    self._union_field = attr

            if isinstance(self._fields[attr], PrimitiveValue):
                return self._fields[attr].value
            else:
                return self._fields[attr]
        else:
            raise AttributeError(attr)

    def __setattr__(self, attr, value):
        if attr in self._constants:
            raise AttributeError(attr + " is read-only")
        elif attr in self._fields:
            if self._is_union:
                if self._union_field and self._union_field != attr:
                    raise AttributeError(attr)
                else:
                    self._union_field = attr

            # noinspection PyProtectedMember
            attr_type = self._fields[attr]._type

            if isinstance(attr_type, dsdl.PrimitiveType):
                self._fields[attr].value = value

            elif isinstance(attr_type, dsdl.CompoundType):
                if not isinstance(value, CompoundValue):
                    raise AttributeError('Invalid type of the value, expected CompoundValue, got %r' % type(value))
                if attr_type.full_name != get_dronecan_data_type(value).full_name:
                    raise AttributeError('Incompatible type of the value, expected %r, got %r' %
                                         (attr_type.full_name, get_dronecan_data_type(value).full_name))
                self._fields[attr] = copy.copy(value)

            elif isinstance(attr_type, dsdl.ArrayType):
                self._fields[attr].clear()
                try:
                    if isinstance(value, str):
                        self._fields[attr].encode(value)
                    else:
                        for item in value:
                            self._fields[attr].append(item)
                except Exception as ex:
                    # We should be using 'raise from' here, but unfortunately we have to be compatible with 2.7
                    raise AttributeError('Array field could not be constructed from the provided value', ex)

            else:
                raise AttributeError(attr + " cannot be set directly")
        else:
            super(CompoundValue, self).__setattr__(attr, value)

    def _unpack(self, stream, tao=True):
        if self._is_union:
            tag_len = union_tag_bits_from_num_elements(len(self._fields))
            self._union_field = list(self._fields.keys())[int(stream[0:tag_len], 2)]
            stream = self._fields[self._union_field]._unpack(stream[tag_len:], tao)
        else:
            for _, last, field in enum_mark_last(self._fields.values()):
                stream = field._unpack(stream, tao and last)
        return stream

    def _pack(self, tao=True):
        if self._is_union:
            keys = list(self._fields.keys())
            field = self._union_field or keys[0]
            tag = keys.index(field)
            tag_len = union_tag_bits_from_num_elements(len(self._fields))
            return format(tag, '0' + str(tag_len) + 'b') + self._fields[field]._pack(tao)
        else:
            return ''.join(field._pack(tao and last) for _, last, field in enum_mark_last(self._fields.values()))


class Frame(object):
    def __init__(self, message_id, data, ts_monotonic=None, ts_real=None, canfd=False):  # @ReservedAssignment
        self.message_id = message_id
        self.bytes = bytearray(data)
        self.ts_monotonic = ts_monotonic
        self.ts_real = ts_real
        self.canfd = canfd

    @property
    def transfer_key(self):
        # The transfer is uniquely identified by the message ID and the 5-bit
        # Transfer ID contained in the last byte of the frame payload.
        return self.message_id, (self.bytes[-1] & 0x1F) if self.bytes else None

    @property
    def toggle(self):
        return bool(self.bytes[-1] & 0x20) if self.bytes else False

    @property
    def end_of_transfer(self):
        return bool(self.bytes[-1] & 0x40) if self.bytes else False

    @property
    def start_of_transfer(self):
        return bool(self.bytes[-1] & 0x80) if self.bytes else False


class TransferError(dronecan.UAVCANException):
    pass


def datalength_roundup(data_length):
    # Number of data bytes 12  16  20  24  32  48  64
    if (data_length <= 8):
        return data_length
    elif (data_length <= 12):
        return 12
    elif (data_length <= 16):
        return 16
    elif (data_length <= 20):
        return 20
    elif (data_length <= 24):
        return 24
    elif (data_length <= 32):
        return 32
    elif (data_length <= 48):
        return 48
    else:
        return 64


class Transfer(object):
    DEFAULT_TRANSFER_PRIORITY = 31

    def __init__(self,
                 transfer_id=0,
                 source_node_id=0,
                 dest_node_id=None,
                 payload=None,
                 transfer_priority=None,
                 request_not_response=False,
                 service_not_message=False,
                 discriminator=None,
                 canfd=False):
        self.transfer_priority = transfer_priority if transfer_priority is not None else self.DEFAULT_TRANSFER_PRIORITY
        self.transfer_id = transfer_id
        self.source_node_id = source_node_id
        self.dest_node_id = dest_node_id
        self.data_type_signature = 0
        self.request_not_response = request_not_response
        self.service_not_message = service_not_message
        self.discriminator = discriminator
        self.ts_monotonic = None
        self.ts_real = None
        self.canfd = canfd

        if payload:
            # noinspection PyProtectedMember
            payload_bits = payload._pack(tao=not canfd)
            if len(payload_bits) & 7:
                payload_bits += "0" * (8 - (len(payload_bits) & 7))
            self.payload = bytes_from_bits(payload_bits)
            self.data_type_id = get_dronecan_data_type(payload).default_dtid
            self.data_type_signature = get_dronecan_data_type(payload).get_data_type_signature()
            self.data_type_crc = get_dronecan_data_type(payload).base_crc
        else:
            self.payload = None
            self.data_type_id = None
            self.data_type_signature = None
            self.data_type_crc = None

        self.is_complete = True if self.payload else False

    def __repr__(self):
        return "Transfer(id={0}, source_node_id={1}, dest_node_id={2}, transfer_priority={3}, payload={4!r})"\
            .format(self.transfer_id, self.source_node_id, self.dest_node_id, self.transfer_priority, self.payload)

    @property
    def message_id(self):
        # Common fields
        id_ = (((self.transfer_priority & 0x1F) << 24) |
               (int(self.service_not_message) << 7) |
               (self.source_node_id or 0))

        if self.service_not_message:
            assert 0 <= self.data_type_id <= 0xFF
            assert 1 <= self.dest_node_id <= 0x7F
            # Service frame format
            id_ |= self.data_type_id << 16
            id_ |= int(self.request_not_response) << 15
            id_ |= self.dest_node_id << 8
        elif self.source_node_id == 0:
            assert self.dest_node_id is None
            assert self.discriminator is not None
            # Anonymous message frame format
            id_ |= self.discriminator << 10
            id_ |= (self.data_type_id & 0x3) << 8
        else:
            assert 0 <= self.data_type_id <= 0xFFFF
            # Message frame format
            id_ |= self.data_type_id << 8

        return id_

    @message_id.setter
    def message_id(self, value):
        self.transfer_priority = (value >> 24) & 0x1F
        self.service_not_message = bool(value & 0x80)
        self.source_node_id = value & 0x7F

        if self.service_not_message:
            self.data_type_id = (value >> 16) & 0xFF
            self.request_not_response = bool(value & 0x8000)
            self.dest_node_id = (value >> 8) & 0x7F
        elif self.source_node_id == 0:
            self.discriminator = (value >> 10) & 0x3FFF
            self.data_type_id = (value >> 8) & 0x3
        else:
            self.data_type_id = (value >> 8) & 0xFFFF

    def to_frames(self):
        out_frames = []
        payload = self.payload
        frame_max = 64 if self.canfd else 8

        if self.canfd and len(payload) > 7:
            # we are sending a CANFD message that needs padding to account for DLC granularity
            total_len = len(payload) + 1
            if total_len > frame_max:
                # we will need more than one frame, so add crc length
                total_len += 2
            mod_len = total_len % (frame_max-1)
            rounded_length = datalength_roundup(mod_len)
            padlen = rounded_length - mod_len
            payload += bytearray([0]*padlen)

        remaining_payload = payload

        # Prepend the transfer CRC to the payload if the transfer requires
        # multiple frames
        if len(remaining_payload) > frame_max-1:
            crc = common.crc16_from_bytes(payload,
                                          initial=self.data_type_crc)
            remaining_payload = bytearray([crc & 0xFF, crc >> 8]) + remaining_payload

        # Generate the frame sequence
        tail = 0x20  # set toggle bit high so the first frame is emitted with it cleared
        while True:
            # Tail byte contains start-of-transfer, end-of-transfer, toggle, and Transfer ID
            tail = ((0x80 if len(out_frames) == 0 else 0) |
                    (0x40 if len(remaining_payload) <= (frame_max-1) else 0) |
                    ((tail ^ 0x20) & 0x20) |
                    (self.transfer_id & 0x1F))
            this_data = remaining_payload[0:frame_max-1] + bchr(tail)
            out_frames.append(Frame(message_id=self.message_id,data=this_data,canfd=self.canfd))
            remaining_payload = remaining_payload[frame_max-1:]
            if not remaining_payload:
                break

        return out_frames

    def from_frames(self, frames):
        # Initialize transfer timestamps from the first frame
        self.ts_monotonic = frames[0].ts_monotonic
        self.ts_real = frames[0].ts_real

        # Validate the flags in the tail byte
        expected_toggle = 0
        expected_transfer_id = frames[0].bytes[-1] & 0x1F
        for idx, f in enumerate(frames):
            tail = f.bytes[-1]
            if (tail & 0x1F) != expected_transfer_id:
                raise TransferError("Transfer ID {0} incorrect, expected {1}".format(tail & 0x1F, expected_transfer_id))
            elif idx == 0 and not (tail & 0x80):
                raise TransferError("Start of transmission not set on frame 0")
            elif idx > 0 and tail & 0x80:
                raise TransferError("Start of transmission set unexpectedly on frame {0}".format(idx))
            elif idx == len(frames) - 1 and not (tail & 0x40):
                raise TransferError("End of transmission not set on last frame")
            elif idx < len(frames) - 1 and (tail & 0x40):
                raise TransferError("End of transmission set unexpectedly on frame {0}".format(idx))
            elif (tail & 0x20) != expected_toggle:
                raise TransferError("Toggle bit value {0} incorrect on frame {1}".format(tail & 0x20, idx))

            expected_toggle ^= 0x20

        self.transfer_id = expected_transfer_id
        self.message_id = frames[0].message_id
        payload_bytes = bytearray(b''.join(bytes(f.bytes[0:-1]) for f in frames))

        # Find the data type
        if self.service_not_message:
            kind = dsdl.CompoundType.KIND_SERVICE
        else:
            kind = dsdl.CompoundType.KIND_MESSAGE
        datatype = dronecan.DATATYPES.get((self.data_type_id, kind))
        if not datatype:
            raise TransferError("Unrecognised {0} type ID {1}"
                                .format("service" if self.service_not_message else "message", self.data_type_id))

        # For a multi-frame transfer, validate the CRC and frame indexes
        if len(frames) > 1:
            transfer_crc = payload_bytes[0] + (payload_bytes[1] << 8)
            payload_bytes = payload_bytes[2:]
            crc = common.crc16_from_bytes(payload_bytes, initial=datatype.base_crc)
            if crc != transfer_crc:
                raise TransferError("CRC mismatch: expected {0:x}, got {1:x} for payload {2!r} (DTID {3:d})"
                                    .format(crc, transfer_crc, payload_bytes, self.data_type_id))

        self.data_type_id = datatype.default_dtid
        self.data_type_signature = datatype.get_data_type_signature()
        self.data_type_crc = datatype.base_crc

        if self.service_not_message:
            self.payload = datatype(_mode="request" if self.request_not_response else "response")
        else:
            self.payload = datatype()
        tao = True
        for frame in frames:
            if frame.canfd:    # we are in CANFD world can't use tao
                tao = False
                self.canfd = True
        # noinspection PyProtectedMember
        self.payload._unpack(bits_from_bytes(payload_bytes), tao)

    @property
    def key(self):
        return self.message_id, self.transfer_id

    def is_response_to(self, transfer):
        if (transfer.service_not_message and self.service_not_message and
                transfer.request_not_response and
                not self.request_not_response and
                transfer.dest_node_id == self.source_node_id and
                transfer.source_node_id == self.dest_node_id and
                transfer.data_type_id == self.data_type_id and
                transfer.transfer_id == self.transfer_id):
            return True
        else:
            return False


class TransferManager(object):
    def __init__(self):
        self.active_transfers = {}
        self.active_transfer_timestamps = {}

    def receive_frame(self, frame):
        result = None
        key = frame.transfer_key
        if key in self.active_transfers or frame.start_of_transfer:
            # If the first frame was received, restart this transfer from scratch
            if frame.start_of_transfer:
                self.active_transfers[key] = []

            self.active_transfers[key].append(frame)
            self.active_transfer_timestamps[key] = time.monotonic()

            # If the last frame of a transfer was received, return its frames
            if frame.end_of_transfer:
                result = self.active_transfers[key]
                del self.active_transfers[key]
                del self.active_transfer_timestamps[key]

        return result

    def remove_inactive_transfers(self, timeout=1.0):
        t = time.monotonic()
        transfer_keys = self.active_transfers.keys()
        for key in transfer_keys:
            if t - self.active_transfer_timestamps[key] > timeout:
                del self.active_transfers[key]
                del self.active_transfer_timestamps[key]
