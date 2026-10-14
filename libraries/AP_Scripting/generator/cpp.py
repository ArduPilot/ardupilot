# AP_FLAKE8_CLEAN
"""Generate Lua/C++ bindings from the parsed description and C++ templates."""

from model import NUMERIC_TYPES
from model import OPERATORS
from model import OUTPUT_ARGUMENT_BASE
from model import BindingError
from model import ErrorCode
from templates import guarded

# Lua extraction functions and the limits imposed by the destination C++ type.
NUMBER_CHECKS = {
    'float': ('float', 'luaL_checknumber', 'get_number', '-INFINITY', 'INFINITY'),
    'int8_t': ('int8_t', 'get_int8_t', 'get_integer', 'INT8_MIN', 'INT8_MAX'),
    'int16_t': ('int16_t', 'get_int16_t', 'get_integer', 'INT16_MIN', 'INT16_MAX'),
    'int32_t': ('lua_Integer', 'luaL_checkinteger', 'get_integer', 'INT32_MIN', 'INT32_MAX'),
    'uint8_t': ('uint8_t', 'get_uint8_t', 'get_integer', '0', 'UINT8_MAX'),
    'uint16_t': ('uint16_t', 'get_uint16_t', 'get_integer', '0', 'UINT16_MAX'),
    'uint32_t': ('uint32_t', 'coerce_to_uint32_t', 'get_uint32', '0U', 'UINT32_MAX'),
    'enum': ('lua_Integer', 'luaL_checkinteger', 'get_integer', None, None),
}


class CppGenerator:
    def __init__(self, bindings, templates):
        self.bindings = bindings
        self.render = templates.render

    def includes(self):
        return ''.join(guarded(f'#include <{header.name}>\n', header.dependency) for header in self.bindings.headers)

    def object_template(self, template, obj, **values):
        return self.render(template, type=obj.name, identifier=obj.identifier, lua_name=obj.lua_name, **values)

    def allocators(self, objects):
        result = []
        for obj in objects:
            text = self.object_template(f'{obj.kind}_allocator.cpp', obj)
            if obj.kind == 'userdata' and obj.creation is None and obj.emit_creation:
                text += self.object_template('constructor.cpp', obj)
            result.append(guarded(text, obj.dependency) + '\n')
        return ''.join(result)

    def checkers(self, objects):
        result = []
        for obj in objects:
            if obj.kind == 'singleton':
                if not obj.needs_checker:
                    continue
                text = self.object_template('singleton_checker.cpp', obj, get=obj.get or f'{obj.name}::get_singleton()')
            else:
                text = self.object_template(f'{obj.kind}_checker.cpp', obj)
            result.append(guarded(text, obj.dependency) + '\n')
        return ''.join(result)

    def receiver(self, obj, field=False):
        """Extract the C++ receiver before checking arguments that may use it."""
        if obj.kind == 'singleton':
            if obj.literal:
                return ''
            expression = f'check_{obj.identifier}(L)'
        else:
            prefix = '*' if obj.kind == 'ap_object' else ''
            expression = f'{prefix}check_{obj.identifier}(L, 1)'
        spacing = '*ud' if field and obj.kind != 'singleton' else '* ud'
        return f'    {obj.name} {spacing} = {expression};\n'

    def argument_check(self, arg, number, skipped=0, indent='    '):
        """Declare an output argument or read and range-check a Lua input."""
        if number > OUTPUT_ARGUMENT_BASE:
            raise BindingError(f"Can't handle more than {OUTPUT_ARGUMENT_BASE} arguments", code=ErrorCode.INTERNAL)
        if arg.output:
            cpp_type = 'uint32_t' if arg.kind == 'enum' else arg.cpp_name
            return f'{indent}{cpp_type} data_{number + OUTPUT_ARGUMENT_BASE} {{}};\n'
        lua_index = number - skipped
        data = f'data_{number}'
        raw = f'raw_data_{number}'
        if arg.kind in NUMBER_CHECKS:
            raw_type, get, checked_get, minimum, maximum = NUMBER_CHECKS[arg.kind]
            if arg.range:
                if arg.kind in ('int8_t', 'int16_t', 'uint8_t', 'uint16_t'):
                    raw_type = 'lua_Integer'
                low, high = arg.range
                if minimum is None:
                    limits = f'static_cast<int32_t>({low}), static_cast<int32_t>({high})'
                else:
                    limits = f'MAX({low}, {minimum}), MIN({high}, {maximum})'
                expression = f'{checked_get}(L, {lua_index}, {limits})'
            else:
                expression = f'{get}(L, {lua_index})'
            text = f'{indent}const {raw_type} {raw} = {expression};\n'
            converted = raw if arg.kind in ('float', 'int32_t') else f'static_cast<{arg.cpp_name}>({raw})'
            return text + f'{indent}const {arg.cpp_name} {data} = {converted};\n'
        if arg.kind == 'boolean':
            return f'{indent}const bool {data} = static_cast<bool>(lua_toboolean(L, {lua_index}));\n'
        if arg.kind == 'string':
            return f'{indent}const char * {data} = luaL_checkstring(L, {lua_index});\n'
        if arg.kind in ('userdata', 'ap_object'):
            pointer = '&' if arg.kind == 'userdata' else '*'
            return f'{indent}{arg.name} {pointer} {data} = *check_{arg.identifier}(L, {lua_index});\n'
        return ''

    def push(self, arg, expression, indent='    ', field=False):
        """Push one result, allocating boxed integers and objects as needed."""
        if arg.kind in ('void', 'literal'):
            return ''
        if arg.kind in ('userdata', 'ap_object', 'uint32_t'):
            return f'{indent}*new_{arg.identifier}(L) = {expression};\n'
        if arg.kind == 'boolean':
            function = 'lua_pushinteger' if field else 'lua_pushboolean'
        elif arg.kind == 'float':
            function = 'lua_pushnumber'
        elif arg.kind == 'string':
            function = 'lua_pushstring'
        else:
            function = 'lua_pushinteger'
        if field and arg.kind == 'enum':
            expression = f'static_cast<int32_t>({expression})'
        return f'{indent}{function}(L, {expression});\n'

    def field(self, obj, member):
        both = member.read and member.write
        indent = '            ' if both else '    '
        write_index = 3 if member.array_len else 2
        expression = f'{obj.receiver}{obj.access}{member.name}'
        if member.array_len:
            expression += '[index]'
        body = self.receiver(obj, field=True)
        if not both:
            count = (2 if member.write else 1) + bool(member.array_len)
            body += f'    binding_argcheck(L, {count});\n'
        if member.array_len:
            body += self.render('array_index.cpp', length=member.array_len) + '\n'
        read = ''
        write = ''
        mask = member.type.valid_mask
        if mask:
            mask_field, mask_value = mask
            mask_expression = f'{obj.receiver}{obj.access}{mask_field}'
        if member.read:
            if both:
                read += '        case 1:\n'
            if mask:
                read += (f'{indent}if (({mask_expression} & {mask_value}) == 0) {{\n'
                         f'{indent}    return 0;\n{indent}}}\n')
            read += self.push(member.type, expression, indent, field=True)
            read += f'{indent}return 1;\n'
        if member.write:
            if both:
                write += '        case 2: {\n'
            write += self.argument_check(member.type, write_index, indent=indent)
            if mask:
                write += f'{indent}{mask_expression} |= {mask_value};\n'
            write += f'{indent}{expression} = data_{write_index};\n{indent}return 0;\n'
            if both:
                write += '         }\n'
        if both:
            count = 'lua_gettop(L)-1' if member.array_len else 'lua_gettop(L)'
            body += self.render('field_switch.cpp', argument_count=count, cases=read + write)
        else:
            body += read + write
        text = self.render('function.cpp', name=f'{obj.identifier}_{member.name}', body=body) + '\n'
        return guarded(text, member.dependency)

    def fields(self, objects):
        return ''.join(guarded(''.join(self.field(obj, member) for member in obj.fields), obj.dependency)
                       for obj in objects if obj.fields)

    def semaphore(self, obj, take):
        if obj.semaphore is None:
            return ''
        action = 'take_blocking' if take else 'give'
        if obj.semaphore == 'scheduler-semaphore':
            return (f'#if AP_SCHEDULER_ENABLED\n'
                    f'    AP::scheduler().get_semaphore().{action}();\n#endif\n')
        access = '->' if obj.semaphore == 'semaphore-pointer' else '.'
        return f'    {obj.receiver}{obj.access}get_semaphore(){access}{action}();\n'

    def call(self, obj, method):
        """Build the native call, keeping output argument numbers stable."""
        result = method.return_type
        call = f'{obj.receiver}{obj.access}{method.name}('
        static_cast = result.kind in NUMERIC_TYPES + ('boolean',)
        if static_cast:
            prefix = f'    const {result.cpp_name} data = static_cast<{result.cpp_name}>({call}'
        elif result.kind == 'string':
            prefix = f'    const char * data = {call}'
        elif result.kind in ('enum', 'userdata'):
            if result.kind == 'userdata' and method.name == 'copy':
                prefix = f'    const {result.name} data = (*{obj.receiver}'
            else:
                prefix = f'    const {result.name} &data = {call}'
        elif result.kind == 'ap_object':
            prefix = f'    {result.name} *data = {call}'
        elif result.kind == 'void':
            prefix = f'    {call}'
        else:
            raise BindingError("Can't return a literal from a method", method.line)
        arguments = []
        number = 2
        for arg in method.arguments:
            if arg.kind == 'literal':
                expression = arg.name
            else:
                expression = f'data_{number + (OUTPUT_ARGUMENT_BASE if arg.output else 0)}'
                if arg.address:
                    expression = '&' + expression
                number += 1
            arguments.append('            ' + expression)
        if number != 2:
            prefix += '\n'
        suffix = '));\n\n' if static_cast else ');\n\n'
        return prefix + ',\n'.join(arguments) + suffix

    def references(self, method, indent):
        count = sum(arg.output for arg in method.arguments)
        text = self.render('reference_stack.cpp', slots=count + 1, indent=indent) + '\n'
        # Preserve the original numbering, including literal positions here.
        for number, arg in enumerate(method.arguments, OUTPUT_ARGUMENT_BASE + 2):
            if arg.output:
                text += self.push(arg, f'data_{number}', indent)
        return text, count

    def method(self, obj, method):
        body = ''
        if method.deprecate is not None:
            body += self.render('deprecation.cpp', object_name=obj.rename or '(null)',
                                method_name=method.lua_name, message=method.deprecate) + '\n'
        body += f'    binding_argcheck(L, {len(method.inputs) + 1});\n'
        body += self.receiver(obj)
        number = 2
        skipped = 0
        for arg in method.arguments:
            if arg.kind != 'literal':
                body += self.argument_check(arg, number, skipped)
                number += 1
            if arg.output:
                skipped += 1
        body += self.semaphore(obj, take=True)
        body += self.call(obj, method)
        body += self.semaphore(obj, take=False)
        returns = 1
        if method.references:
            text, count = self.references(method, '    ')
            body += text
            returns += count
        result = method.return_type
        if result.kind == 'boolean' and method.nullable:
            text, count = self.references(method, '        ')
            body += f'    if (data) {{\n{text}        return {count};\n    }}\n    return 0;\n'
        else:
            if result.kind == 'ap_object':
                body += '    if (data == NULL) {\n        return 0;\n    }\n'
            indent = '        ' if result.kind == 'uint32_t' else '    '
            body += self.push(result, 'data', indent)
            if result.kind == 'void':
                returns -= 1
            body += f'    return {returns};\n'
        text = self.render('function.cpp', name=f'{obj.identifier}_{method.identifier}', body=body)
        return guarded(guarded(text, method.dependency), obj.dependency) + '\n'

    def operators(self, obj):
        functions = []
        for symbol, name in OPERATORS.items():
            if symbol not in obj.operators:
                continue
            access = '' if obj.operator_getter else '*'
            getter = obj.operator_getter or f'check_{obj.identifier}'
            body = f'    binding_argcheck(L, 2);\n    {obj.name} {access}ud = {getter}(L, 1);\n'
            if symbol == '~':
                expression = f'{symbol} ({access}ud)'
            else:
                body += f'    {obj.name} {access}ud2 = {getter}(L, 2);\n'
                expression = f'({access}ud) {symbol} ({access}ud2)'
            if symbol in ('==', '<', '<='):
                body += f'    lua_pushboolean(L, {expression});\n'
            else:
                body += f'    *new_{obj.identifier}(L) = {expression};\n'
            body += '    return 1;\n'
            functions.append(self.render('function.cpp', name=f'{obj.identifier}_{name}', body=body) + '\n')
        return guarded(''.join(functions), obj.dependency)

    def methods(self, objects):
        text = ''
        for obj in objects:
            text += ''.join(self.method(obj, method) for method in obj.methods)
            if obj.has_operators:
                text += self.operators(obj)
        return text

    def table(self, cpp_type, name, entries):
        return self.render('table.cpp', type=cpp_type, name=name, entries=''.join(entries)) + '\n'

    def entry(self, name, function, dependency=None):
        return guarded(f'    {{"{name}", {function}}},\n', dependency)

    def index(self, obj):
        entries = [self.entry(method.lua_name, f'{obj.identifier}_{method.name}', method.dependency)
                   for method in obj.methods]
        entries += [self.entry(member.lua_name, f'{obj.identifier}_{member.name}', member.dependency)
                    for member in obj.fields]
        for alias in obj.aliases:
            if alias.kind == 'manual_operator':
                entries.append(guarded('', alias.dependency))
                continue
            function = alias.name if alias.kind == 'manual' else f'{obj.identifier}_{alias.name}'
            entries.append(self.entry(alias.lua_name, function, alias.dependency))
        text = self.table('luaL_Reg', f'{obj.identifier}_meta', entries)
        if obj.has_operators:
            entries = [self.entry(name, f'{obj.identifier}_{name}')
                       for symbol, name in OPERATORS.items() if symbol in obj.operators]
            entries += [self.entry(alias.lua_name, alias.name) for alias in obj.aliases if alias.kind == 'manual_operator']
            entries.append('    {NULL, NULL},\n')
            text += self.table('luaL_Reg', f'{obj.identifier}_operators', entries)
        if obj.enums:
            text += self.table('struct userdata_enum', f'{obj.identifier}_enums',
                               [self.entry(name, f'{obj.name}::{name}') for name in obj.enums])
        enum_loader = ''
        if obj.enums:
            enum_loader = f' || load_enum(L,{obj.identifier}_enums,ARRAY_SIZE({obj.identifier}_enums))'
        text += self.render('index.cpp', identifier=obj.identifier, enum_loader=enum_loader)
        return guarded(text, obj.dependency) + '\n'

    def constructors(self):
        entries = []
        for obj in self.bindings.userdata:
            if obj.emit_creation:
                entries.append(self.entry(obj.lua_name, obj.creation or f'lua_new_{obj.identifier}', obj.dependency))
        entries += [self.entry(alias.lua_name, alias.name, alias.dependency) for alias in self.bindings.globals]
        return self.render('constructors.cpp', entries=''.join(entries)) + '\n'

    def loaders(self):
        text = ''
        for name, objects in (('userdata', self.bindings.userdata), ('singleton', self.bindings.singletons),
                              ('ap_object', self.bindings.ap_objects)):
            entries = []
            for obj in objects:
                function = f'{obj.identifier}_index'
                if name == 'userdata':
                    function += f', {obj.identifier}_operators' if obj.has_operators else ', nullptr'
                entries.append(self.entry(obj.lua_name, function, obj.dependency))
            cpp_type = 'struct userdata_meta' if name == 'userdata' else 'struct luaL_Reg'
            text += self.table(cpp_type, name + '_fun', entries)
        return text

    def source(self):
        bindings = self.bindings
        # Keep section order stable for comparisons with the original C generator.
        allocators = self.allocators(bindings.userdata)
        checkers = self.checkers(bindings.userdata)
        checkers += self.allocators(bindings.ap_objects) + self.checkers(bindings.ap_objects)
        checkers += self.checkers(bindings.singletons)
        sections = ''
        for objects in (bindings.singletons, bindings.userdata, bindings.ap_objects):
            sections += self.fields(objects) + self.methods(objects)
            sections += ''.join(self.index(obj) for obj in objects)
        return self.render('source.cpp', includes=self.includes(), allocators=allocators, checkers=checkers,
                           bindings=sections, constructors=self.constructors(), loaders=self.loaders())

    def header(self):
        declarations = ''
        for obj in self.bindings.userdata:
            text = f'{obj.name} * new_{obj.identifier}(lua_State *L);\n'
            if obj.creation is None and obj.emit_creation:
                text += f'int lua_new_{obj.identifier}(lua_State *L);\n'
            text += f'{obj.name} * check_{obj.identifier}(lua_State *L, int arg);\n'
            declarations += guarded(text, obj.dependency)
        for obj in self.bindings.ap_objects:
            text = (f'{obj.name} ** new_{obj.identifier}(lua_State *L);\n'
                    f'{obj.name} ** check_{obj.identifier}(lua_State *L, int arg);\n')
            declarations += guarded(text, obj.dependency)
        for obj in self.bindings.singletons:
            if obj.needs_checker:
                declarations += guarded(f'{obj.name} * check_{obj.identifier}(lua_State *L);\n', obj.dependency)
        return self.render('header.h', includes=self.includes(), declarations=declarations)
