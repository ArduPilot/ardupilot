# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Typed C++ clients for the same contract catalogue and wire policies as Rust.

No firmware dispatch IDs or C++ object layouts enter this SDK. New wire types
must be described explicitly; an unknown contract is a generation error.
"""

import re

import borrowed_binding
import call_model
import contracts
import math_binding
import singleton_binding
import value_binding

from native_templates import render_lines


def identifier(name):
    name = re.sub(r'[^a-zA-Z0-9_]', '_', name)
    if name in ('register', 'delete', 'new', 'default', 'typeid', 'namespace', 'class', 'template', 'operator'):
        name += '_'
    return name


class Types:
    def __init__(self):
        self.definitions = []
        self.records = {}
        self.enum_values = {}

    def record(self, name, fields):
        name = identifier(name)
        fields = [(identifier(n), t) for n, t in fields]
        if name in self.records:
            if self.records[name] != fields:
                raise ValueError('conflicting C++ wire record: ' + name)
            return name
        self.records[name] = fields
        self.definitions += ['struct ' + name + ' {'] + [f'    {t} {n};' for n, t in fields] + ['};']
        total = ' + '.join(f'Codec<{t}>::size' for _, t in fields) or '0'
        self.definitions += [f'namespace wire {{ template<> struct Codec<{name}> {{',
                             f'    static constexpr size_t size = {total};']
        for operation, qualifier, const in [('put', '', 'const '), ('get', 'const ', '')]:
            self.definitions += [f'    static bool {operation}({qualifier}uint8_t *p, {const}{name} &v) {{']
            offset = '0'
            for field, kind in fields:
                self.definitions += [f'        if (!Codec<{kind}>::{operation}(p + ({offset}), v.{field})) '
                                     '{ return false; }']
                offset += f' + Codec<{kind}>::size'
            self.definitions += ['        return true;', '    }']
        self.definitions += ['}; }']
        return name

    def enum(self, name, scalar='int32_t', values=None):
        name = identifier(singleton_binding.enum_name(name))
        key = 'enum:' + name
        if key in self.records and self.records[key] != scalar:
            raise ValueError('conflicting C++ enum width: ' + name)
        if key not in self.records:
            self.records[key] = scalar
            constants = (values if values is not None else
                         singleton_binding.mavlink_constants(singleton_binding.MAVLINK_COMMON, 'MAV_SEVERITY')
                         if name == 'MavSeverity' else {})
            self.enum_values[name] = constants
            self.definitions += [f'enum class {name} : {scalar} {{' +
                                 ', '.join(f'{n} = {v}' for n, v in constants.items()) + '};']
        return name

    def size(self, kind):
        scalars = {'bool': 1, 'uint8_t': 1, 'int8_t': 1, 'uint16_t': 2, 'int16_t': 2,
                   'uint32_t': 4, 'int32_t': 4, 'float': 4, 'uint64_t': 8, 'void': 0}
        if kind in scalars:
            return scalars[kind]
        if kind.startswith('Optional<'):
            return 1 + self.size(kind[9:-1])
        if kind.startswith('Array<'):
            element, count = kind[6:-1].rsplit(', ', 1)
            return self.size(element) * int(count)
        if 'enum:' + kind in self.records:
            return self.size(self.records['enum:' + kind])
        return sum(self.size(t) for _, t in self.records[kind])

    def type(self, value, hint='Value', input=False):
        if isinstance(value, dict):
            if value.get('name') == 'EscTelemetryUpdate':
                return self.record('EscTelemetryUpdate', [('present', 'uint8_t'), ('temperature_cdeg', 'int16_t'),
                                                          ('voltage', 'float'), ('current', 'float')])
            if value.get('name') == 'ServoTelemetryData':
                return self.record('ServoTelemetryData', [('present', 'bool'), ('measured_position', 'float')])
            if 'fields' in value:
                result = self.record(value.get('name', hint),
                                     [(n, self.type(t, hint + '_' + n)) for n, t in value['fields']])
                return f'Optional<{result}>' if 'presence' in value else result
            if 'enum' in value:
                return self.enum(value['enum'])
            if 'wire' in value:
                return self.record(value.get('handle', hint) + 'Token', [('value', 'uint32_t')])
            if 'type' in value:
                if value.get('encoding') == 'i32-le':
                    return self.enum(value['type'])
                return self.type(value['type'], hint, input)
            if value.get('presence', '').startswith('u32'):
                return self.record(value['handle'] + 'Token', [('value', 'uint32_t')])
            if 'handle' in value:
                data = self.record(value['handle'].split(',')[0] + 'Token', [('value', 'uint8_t')])
                return f'Optional<{data}>'
            fields = [(n, self.type(t, hint + '_' + n)) for n, t in value.items() if n != 'presence']
            result = self.record(hint, fields)
            return f'Optional<{result}>' if 'presence' in value else result
        if value.startswith('format:'):
            return self.record('PrintfPacket', [('format', 'Array<uint8_t, 256>'), ('count', 'uint8_t'),
                                                ('tags', 'Array<uint8_t, 8>'), ('values', 'Array<uint64_t, 8>'),
                                                ('strings', 'Array<uint8_t, 256>')])
        if value.startswith('length:u16-le,text:'):
            return self.record('PrintfText', [('length', 'uint16_t'), ('text', 'Array<uint8_t, 256>')])
        if value.startswith('optional-f32-le,'):
            return 'float'
        if value == 'Vector3f,three-f32-le':
            return self.type('Vector3f')
        explicit = {
            'owned-video-stream-information': ('VideoStreamInformation', [
                ('framerate', 'float'), ('bitrate', 'uint32_t'), ('flags', 'uint16_t'),
                ('resolution_h', 'uint16_t'), ('resolution_v', 'uint16_t'), ('rotation', 'uint16_t'),
                ('hfov', 'uint16_t'), ('stream_id', 'uint8_t'), ('count', 'uint8_t'),
                ('stream_type', 'uint8_t'),
                ('name', 'Array<uint8_t, 32>'), ('uri', 'Array<uint8_t, 160>'), ('encoding', 'uint8_t')]),
            'owned-camera-information230': ('CameraInformation', [
                ('firmware_version', 'uint32_t'), ('focal_length', 'float'), ('sensor_size_h', 'float'),
                ('sensor_size_v', 'float'), ('flags', 'uint32_t'), ('resolution_h', 'uint16_t'),
                ('resolution_v', 'uint16_t'), ('definition_version', 'uint16_t'),
                ('vendor_name', 'Array<uint8_t, 32>'), ('model_name', 'Array<uint8_t, 32>'),
                ('definition_uri', 'Array<uint8_t, 140>')]),
            'optional-camera-state31': ('CameraState', [
                ('present', 'bool'), ('take_picture_increment', 'uint16_t'), ('recording_video', 'bool'),
                ('zoom_type', 'uint8_t'), ('zoom_value', 'float'), ('focus_type', 'uint8_t'),
                ('focus_value', 'float'), ('tracking_type', 'uint8_t'),
                ('tracking_x1', 'float'), ('tracking_y1', 'float'), ('tracking_x2', 'float'), ('tracking_y2', 'float')]),
            'ServoTelemetryUpdate,optional-fields-mask-u16': ('ServoTelemetryUpdate', [
                ('present', 'uint16_t'), ('command_position', 'float'), ('measured_position', 'float'),
                ('force', 'float'), ('speed', 'float'), ('voltage', 'float'), ('current', 'float'),
                ('duty_cycle', 'uint8_t'), ('motor_temperature_cdeg', 'int16_t'),
                ('pcb_temperature_cdeg', 'int16_t'), ('status_flags', 'uint8_t')]),
            'optional-owned-event-count-param-events-payload57': ('CrsfPeekEvent', [
                ('present', 'bool'), ('queue_depth', 'uint8_t'), ('param_id', 'uint8_t'),
                ('events', 'uint8_t'), ('length', 'uint8_t'), ('payload', 'Array<uint8_t, 57>')]),
            'optional-owned-event-param-events-payload57': ('CrsfEvent', [
                ('present', 'bool'), ('param_id', 'uint8_t'), ('events', 'uint8_t'),
                ('length', 'uint8_t'), ('payload', 'Array<uint8_t, 57>')]),
        }
        if value in explicit:
            return self.record(*explicit[value])
        if value == 'optional-backend-index-bool-u8':
            return 'Optional<uint8_t>'
        if value == 'utf8-bytes-max32-length-u8-zero-pad':
            return self.record('MotorFrameName', [('length', 'uint8_t'), ('data', 'Array<uint8_t, 32>')])
        if value == 'optional-bool-u8':
            return 'Optional<bool>'
        if value == 'bytes-len-u8-max255-zero-pad255':
            return self.record('CrsfData', [('length', 'uint8_t'), ('data', 'Array<uint8_t, 255>')])
        if value in ('menu-name', 'utf8-len-u8-max15-zero-pad15'):
            return self.record('CrsfName', [('length', 'uint8_t'), ('data', 'Array<uint8_t, 15>')])
        value = value.split(',')[0]
        scalars = {'bool-u8': 'bool', 'u8': 'uint8_t', 'u7': 'uint8_t', 'i8': 'int8_t',
                   'u16': 'uint16_t', 'u16-le': 'uint16_t', 'i16': 'int16_t', 'i16-le': 'int16_t',
                   'u32': 'uint32_t', 'u32-le': 'uint32_t', 'i32': 'int32_t', 'i32-le': 'int32_t',
                   'u64': 'uint64_t', 'u64-le': 'uint64_t', 'f32': 'float', 'f32-le': 'float', 'void': 'void',
                   'host-order-u32-le-ipv4': 'uint32_t', 'host-order-u16-le': 'uint16_t',
                   'u24-in-u32-le': 'uint32_t', 'param-id': 'uint8_t', 'events': 'uint8_t',
                   'event-mask-u8': 'uint8_t', 'type-u8-range1-4': 'uint8_t', 'u8-index': 'uint8_t',
                   'u16-le-key': 'uint16_t', 'u32-le-group-element': 'uint32_t'}
        if value in scalars:
            return scalars[value]
        if value in ('Parameter',):
            return 'uint64_t'
        if value == 'optional-nonzero-u32-le' and input:
            return 'Optional<uint32_t>' if hint.startswith('SerialPort_begin') else 'uint32_t'
        if value.startswith(('nonzero-u32', 'optional-nonzero-u32')) or value in (
                'menu-token', 'parameter-token', 'optional-menu-token', 'optional-parameter-token'):
            return 'uint32_t'
        if value.startswith('nonzero-'):
            return self.type(value[len('nonzero-'):], hint, input)
        if value in ('owned-vector3f', 'Vector3f'):
            return self.type(dict(name='Vector3f', fields=[[n, 'f32-le'] for n in ('x', 'y', 'z')]))
        if value == 'optional-u32-le':
            return 'Optional<uint32_t>'
        if value == 'optional-f32-le':
            return 'Optional<float>'
        if value.startswith('optional-f32-le'):
            return 'float'
        array = re.fullmatch(r'(u16-le|f32-le)\[(\d+)\]', value)
        if array:
            return f'Array<{self.type(array[1])}, {array[2]}>'
        if value.endswith(('-u8', '-i32-le')):
            scalar = 'uint8_t' if value.endswith('-u8') else 'int32_t'
            return self.enum(value.rsplit('-', 1)[0] if scalar == 'uint8_t' else value[:-7], scalar)
        string = re.search(r'(?:zero-pad|zero-terminated|cstring)(\d+)', value)
        if string:
            n = int(string[1])
            if input and 'bytes-max' in value and 'nul-free' not in value and not value.startswith('utf8'):
                return self.record(hint + 'Bytes', [('length', 'uint16_t'), ('data', f'Array<uint8_t, {n}>')])
            return f'Array<uint8_t, {n}>'
        fixed = re.fullmatch(r'bytes(\d+)', value)
        if fixed:
            return f'Array<uint8_t, {fixed[1]}>'
        if value == 'bytes-max256':
            return self.record('Payload256', [('length', 'uint16_t'), ('data', 'Array<uint8_t, 256>')])
        if value == 'optional-bytes-max255':
            return 'Optional<Array<uint8_t, 255>>'
        if value == 'bytes-len-u8-max255-zero-pad255':
            return self.record('CrsfData', [('length', 'uint8_t'), ('data', 'Array<uint8_t, 255>')])
        if value in ('menu-name', 'utf8-len-u8-max15-zero-pad15'):
            return self.record('CrsfName', [('length', 'uint8_t'), ('data', 'Array<uint8_t, 15>')])
        raise ValueError(f'undescribed C++ wire type {value!r} ({hint})')

    def field(self, arg):
        kind = arg['type']
        if kind == 'enum':
            return self.enum(arg['enum'])
        if kind in value_binding.TYPES or kind in math_binding.TYPES:
            return self.type(value_binding.wire_type(kind))
        if kind in borrowed_binding.TYPES:
            return self.type(borrowed_binding.wire_type(kind))
        if kind == 'fwstring':
            return self.record('FirmwareString', [('length', 'uint8_t'), ('data', 'Array<uint8_t, 64>')])
        return {'boolean': 'bool', 'uint8_t': 'uint8_t', 'uint16_t': 'uint16_t', 'uint32_t': 'uint32_t',
                'int8_t': 'int8_t', 'int16_t': 'int16_t', 'int32_t': 'int32_t', 'float': 'float',
                'string': 'StringView', 'cstring': 'StringView', 'void': 'void'}[kind]

    def from_model(self, value, hint='Value'):
        if value is None:
            return 'void'
        if isinstance(value, call_model.Scalar):
            return {'bool': 'bool', 'f32': 'float', 'u8': 'uint8_t', 'i8': 'int8_t', 'u16': 'uint16_t',
                    'i16': 'int16_t', 'u32': 'uint32_t', 'i32': 'int32_t', 'u64': 'uint64_t'}[value.name]
        if isinstance(value, call_model.Enum):
            return self.enum(value.name, self.from_model(value.scalar), None if value.external else dict(value.values))
        if isinstance(value, call_model.Record):
            return self.record(value.name, [(f.name, self.from_model(f.type)) for f in value.fields])
        if isinstance(value, call_model.Handle):
            return self.from_model(value.scalar)
        if isinstance(value, (call_model.Length, call_model.BoundedCount)):
            return self.from_model(value.scalar)
        if isinstance(value, call_model.Buffer):
            return f'Array<uint8_t, {value.capacity}>'
        if isinstance(value, call_model.String):
            return 'StringView'
        if isinstance(value, call_model.OptionalValue):
            inner = self.from_model(value.type)
            return inner if isinstance(value.type, call_model.Handle) else f'Optional<{inner}>'
        if isinstance(value, call_model.BufferResult):
            data = f'Array<uint8_t, {value.capacity}>'
            return (self.record(hint, [('length', self.from_model(value.length)), ('data', data)])
                    if value.length else f'Optional<{data}>')
        raise ValueError('unsupported C++ call-model type')


def describe(spec):
    types = Types()
    entries = contracts.describe(spec)
    models = call_model.calls(spec)
    plans = []
    handled = set()
    for method in spec['methods']:
        args, outputs, optional = singleton_binding.io(method)
        inputs, copybacks = [], []
        if method.get('value_receiver'):
            kind = types.field(dict(type=method['value_receiver']))
            inputs.append(('receiver', kind, None))
            copybacks.append(('receiver', kind))
        elif method.get('receiver'):
            inputs.append(('receiver', types.field(dict(type=method['receiver'])), None))
        for i, arg in args:
            kind = types.field(arg)
            inputs.append((f'arg{i}', kind, arg))
            if arg.get('inout'):
                copybacks.append((f'arg{i}', kind))
        result_kind = types.field(method['result'])
        if outputs:
            fields = [(f'out{i}', types.field(a)) for i, a in outputs]
            kind = fields[0][1] if len(fields) == 1 else types.record(
                identifier(method['api_name']) + 'Output', fields)
            result_kind = f'Optional<{kind}>' if optional else kind
        results = [] if result_kind == 'void' else [('value', result_kind)]
        plans.append((method['api_name'], inputs, results, copybacks, singleton_binding.sizes(method), method))
        handled.add(method['api_name'])
        for alias in method['api_aliases']:
            handled.add(alias)
            plans.append((alias, inputs, results, copybacks, singleton_binding.sizes(method), method))
    # Remaining audited categories are added below; no raw-buffer escape hatch.
    for owner in spec['math']['types']:
        for method in owner['methods']:
            name = owner['name'] + '.' + method['name']
            ins, outs = math_binding.wire_fields(owner['name'], method)

            def kind(k):
                return types.field(dict(type=k))
            inputs = [(n, kind(k), None) for n, k in ins]
            copybacks = [(n, kind(k)) for n, k in outs if n != 'value']
            for n, k in copybacks:
                if n not in {n for n, _, _ in inputs}:
                    inputs.append((n, k, {'out_only': True}))
            results = [(n, kind(k)) for n, k in outs if n == 'value']
            sizes = tuple(sum(math_binding.size(k) for _, k in fields) for fields in (ins, outs))
            plans.append((name, inputs, results, copybacks, sizes, None))
            handled.add(name)
    for method in spec['parameter']['methods']:
        name = 'Parameter.' + method['name']
        inputs = [('receiver', 'uint64_t', None)]
        for i, arg in enumerate(method['arguments']):
            if not arg['nullable']:
                inputs.append((f'arg{i}', types.field(dict(type=arg['kind'])),
                               {'type': arg['kind']}))
        result = 'Optional<float>' if method['name'] == 'get' else 'bool'
        sizes = (8 + (17 if method['name'] == 'init' else 4 if len(inputs) > 1 else 0),
                 5 if method['name'] == 'get' else 1)
        plans.append((name, inputs, [('value', result)], [], sizes, None))
        handled.add(name)
    for entry in entries:
        name = entry['contract']['name']
        if name in handled:
            continue
        call = models.get(name)
        if call is not None:
            inputs = [(identifier(field.name), types.from_model(field.type),
                       {'type': 'cstring', 'size': field.type.capacity, 'nonempty': field.type.nonempty}
                       if isinstance(field.type, call_model.String) else None) for field in call.inputs]
            # Keep the existing C++ presentation of index selectors (a scalar
            # or named token record); the shared model defines their wire shape.
            result = (types.type(entry['contract']['result'], identifier(name) + 'Output')
                      if isinstance(call.result, call_model.SelectedHandle) else
                      types.from_model(call.result, identifier(name) + 'Output'))
            results = [] if result == 'void' else [('value', result)]
            plans.append((name, inputs, results, [], (call.input_size, call.output_size), call))
            continue
        contract = entry['contract']
        inputs = []
        for i, arg in enumerate(contract['arguments']):
            arg = arg if isinstance(arg, dict) else {'type': arg}
            if arg.get('direction') in ('optional-out', 'out'):
                continue
            inputs.append((identifier(arg.get('name', f'arg{i}')),
                           types.type(arg['type'], identifier(name) + f'Arg{i}', True), arg))
        result = types.type(contract['result'], identifier(name) + 'Output')
        # These audited packets place length/presence words before the data,
        # rather than following the public argument order.
        if name == 'mavlink.send_chan':
            inputs = [inputs[0], inputs[1], ('payload_length', 'uint16_t', None), inputs[3],
                      ('payload', 'Array<uint8_t, 255>', None)]
        elif name == 'network.socket_send_to':
            inputs = [inputs[0], ('length', 'uint16_t', None), inputs[2], inputs[3],
                      ('data', 'Array<uint8_t, 1024>', None)]
        elif name == 'vehicle.register_custom_mode':
            inputs = [inputs[0], ('full_length', 'uint8_t', None), inputs[1],
                      ('short_length', 'uint8_t', None), inputs[2]]
        elif name == 'arming.set_aux_auth_failed':
            inputs = [inputs[0], ('length', 'uint8_t', None), inputs[1]]
        results = [] if result == 'void' else [('value', result)]
        plans.append((name, inputs, results, [], None, None))
    return types, entries, plans


def emit(spec, notice):
    types, entries, plans = describe(spec)
    by_name = {e['contract']['name']: e for e in entries}
    lines = [notice, '#pragma once', '#include "runtime.hpp"', 'namespace ap_module {']
    lines += types.definitions
    groups = {}
    for name, inputs, results, copybacks, sizes, method in plans:
        group, _, fn = name.partition('.')
        group = 'parameter_cache' if group == 'Parameter' else group.lower()
        groups.setdefault(group, []).append((name, fn, inputs, results, copybacks, sizes, method))
    for group, methods in groups.items():
        cls = identifier(group) + 'Api'
        method_lines = []
        for name, fn, inputs, results, copybacks, sizes, method in methods:
            entry = by_name[name]
            slot = 'import_' + entry['hash']
            kind = results[0][1] if results else 'void'
            args = ', '.join(f'{t} {"&" if any(n == a for a, _ in copybacks) else ""}{n}'
                             for n, t, _ in inputs)
            ni = str(sizes[0]) if sizes else ' + '.join(f'wire::Codec<{t}>::size' for _, t, _ in inputs) or '0'
            no = str(sizes[1]) if sizes else ' + '.join(f'wire::Codec<{t}>::size' for _, t in results) or '0'
            encoded = []
            offset = '0'
            for arg, t, policy in inputs:
                if policy and policy.get('out_only'):
                    continue
                if t == 'StringView':
                    size = policy.get('size', 17 if policy['type'] == 'string' else 256)
                    nonempty = policy.get('nonempty', size == 17)
                    encode = f'wire::string<{size}, {str(nonempty).lower()}>(input + ({offset}), {arg})'
                    step = str(size)
                else:
                    encode = f'wire::Codec<{t}>::put(input + ({offset}), {arg})'
                    step = f'wire::Codec<{t}>::size'
                encoded += [f'        if (!{encode}) {{ return {{Error::Invalid}}; }}']
                offset += ' + ' + step
            copied = []
            offset = f'wire::Codec<{kind}>::size' if results else '0'
            for arg, t in copybacks:
                copied += [f'        if (!wire::Codec<{t}>::get(output + ({offset}), {arg})) '
                           '{ return {Error::Host}; }']
                offset += f' + wire::Codec<{t}>::size'
            decoded = []
            if results:
                decoded = [f'        {kind} _result_value{{}};',
                           f'        if (!wire::Codec<{kind}>::get(output, _result_value)) {{ return {{Error::Host}}; }}']
                if isinstance(method, call_model.Call) and isinstance(method.result, call_model.SelectedHandle):
                    offset = next(pos for field, pos in call_model.layout(method.inputs)
                                  if field.name == method.result.argument)
                    decoded = render_lines('cpp/decode_selected_handle.hpp', result=kind, input_offset=offset)
                if (isinstance(method, call_model.Call) and
                        (isinstance(method.result, call_model.BoundedCount) or
                         isinstance(method.result, call_model.BufferResult) and method.result.length)):
                    field, position = next((f, p) for f, p in call_model.layout(method.inputs)
                                           if isinstance(f.type, call_model.Length) and
                                           f.type.buffer == method.result.buffer)
                    buffer = next(a.type for a in method.arguments if a.name == method.result.buffer)
                    count = '_result_value.length' if isinstance(method.result, call_model.BufferResult) else '_result_value'
                    padding = ''
                    if isinstance(method.result, call_model.BufferResult):
                        padding = ('        for (size_t i = ' + str(method.result.length.size) +
                                   ' + size_t(' + count + '); i < ' + str(method.output_size) + '; i++) {\n'
                                   '            if (output[i] != 0) { return {Error::Host}; }\n        }\n')
                    decoded += render_lines(
                        'cpp/check_count.hpp', scalar=types.from_model(field.type.scalar), offset=position,
                        value=count, capacity=buffer.capacity, padding=padding,
                    )
            method_lines += render_lines(
                'cpp/method.hpp', result=kind, name=identifier(fn), arguments=args, slot=slot,
                hash=entry['hash'], api_name=name, input_size=ni, output_size=no,
                encode=''.join(line + '\n' for line in encoded),
                copybacks=''.join(line + '\n' for line in copied),
                decode=''.join(line + '\n' for line in decoded),
                result_value=', _result_value' if results else '',
            )
        lines += render_lines(
            'cpp/namespace.hpp', class_name=cls, methods=''.join(line + '\n' for line in method_lines),
        )
    proxies = [f'    {identifier(group)}Api {identifier(group.lower())}() const '
               f'{{ return {identifier(group)}Api(*this); }}\n' for group in groups]
    lines += render_lines('cpp/context.hpp', proxies=''.join(proxies))
    lines += ['} // namespace ap_module']
    imports = [notice, '#include <stdint.h>', 'namespace ap_module {']
    for entry in entries:
        name, h = entry['contract']['name'], entry['hash']
        imports += render_lines('cpp/import.cpp', hash=h, name=name)
    imports += ['}']
    return {'sdk/cpp/bindings.hpp': '\n'.join(lines) + '\n',
            'sdk/cpp/imports.cpp': '\n'.join(imports) + '\n'}
