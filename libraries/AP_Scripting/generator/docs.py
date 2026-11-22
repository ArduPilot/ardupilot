# AP_FLAKE8_CLEAN
"""Generate the same editable EmmyLua documentation as the binding compiler."""

from model import BindingError
from model import ErrorCode


class DocsGenerator:
    def __init__(self, bindings, templates):
        self.bindings = bindings
        self.render = templates.render

    def type_name(self, arg, parameter=False):
        if arg.kind in ('void', 'literal'):
            return None
        names = {'boolean': 'boolean', 'float': 'number', 'string': 'string', 'uint32_t': 'uint32_t_ud'}
        if arg.kind == 'userdata':
            obj = next((obj for obj in self.bindings.userdata if obj.identifier == arg.identifier), None)
            if obj is None:
                raise BindingError(f'Could not find userdata {arg.identifier}', code=ErrorCode.GENERAL)
            return obj.docs_name
        if arg.kind == 'ap_object':
            return arg.identifier + '_ud'
        if arg.kind == 'uint32_t' and parameter:
            return 'uint32_t_ud|integer|number'
        return names.get(arg.kind, 'integer')

    def annotation(self, prefix, arg, parameter=False, nullable=False):
        name = self.type_name(arg, parameter)
        if name is None:
            return ''
        return f'{prefix} {name}{"|nil" if nullable else ""}\n'

    def return_annotation(self, arg, nullable=False):
        return self.annotation('---@return', arg, nullable=nullable or arg.kind == 'ap_object')

    def function(self, name, annotations, count=None, parameters=None):
        if parameters is None:
            parameters = ', '.join(f'param{i + 1}' for i in range(count))
        return self.render('docs_function.lua', name=name, annotations=annotations, parameters=parameters) + '\n'

    def method(self, obj, method, name=None):
        annotations = '-- desc\n'
        if method.deprecate is not None:
            annotations += f'---@deprecated {method.deprecate}\n'
        for number, arg in enumerate(method.inputs, 1):
            annotations += self.annotation(f'---@param param{number}', arg, parameter=True)
        if not method.nullable:
            annotations += self.return_annotation(method.return_type)
        for arg in method.arguments:
            if arg.output and arg.kind != 'literal':
                annotations += self.return_annotation(arg, nullable=arg.nullable)
        return self.function(f'{obj.docs_name}:{name or method.lua_name}', annotations, len(method.inputs))

    def manual(self, alias, obj=None):
        annotations = '-- desc\n'
        annotations += ''.join(f'---@param param{i + 1} UNKNOWN\n' for i in range(alias.num_args))
        annotations += '---@return UNKNOWN\n' * max(0, alias.num_returns)
        name = f'{obj.docs_name}:{alias.lua_name}' if obj else alias.lua_name
        return self.function(name, annotations, alias.num_args)

    def field(self, obj, member):
        text = ''
        array = member.array_len is not None
        for write in (False, True):
            if not (member.write if write else member.read):
                continue
            action = 'set' if write else 'get'
            annotations = f'-- {action} {"array " if array else ""}field\n'
            parameters = ['index'] if array else []
            if array:
                annotations += '---@param index integer\n'
            if write:
                annotations += self.annotation('---@param value', member.type, parameter=True)
                parameters.append('value')
            else:
                annotations += self.annotation('---@return', member.type, nullable=member.type.valid_mask is not None)
            text += self.function(f'{obj.docs_name}:{member.lua_name}', annotations, parameters=', '.join(parameters))
        return text

    def object(self, obj):
        text = '-- desc\n'
        if obj.kind != 'singleton':
            text += f'---@class {obj.docs_name}\n'
        text += ''.join(f'---@field {name} number\n' for name in obj.enums)
        local = 'local ' if obj.kind != 'singleton' else ''
        text += f'{local}{obj.docs_name} = {{}}\n\n'
        if obj.kind == 'userdata' and obj.emit_creation:
            count = obj.creation_args if obj.creation else 0
            annotations = ''.join(f'---@param param{i + 1} UNKNOWN\n' for i in range(count))
            annotations += f'---@return {obj.docs_name}\n'
            text += self.function(obj.rename or obj.identifier, annotations, count)
        text += ''.join(self.field(obj, member) for member in obj.fields)
        text += ''.join(self.method(obj, method) for method in obj.methods)
        for alias in obj.aliases:
            if alias.kind == 'alias':
                method = next((method for method in obj.methods if method.name == alias.name), None)
                if method is None:
                    raise BindingError(f'Could not find Method {alias.name} to alias to {alias.lua_name}', code=ErrorCode.DOCS)
                text += self.method(obj, method, alias.lua_name)
            elif alias.kind == 'manual':
                text += self.manual(alias, obj)
        return text + '\n'

    def generate(self):
        objects = self.bindings.userdata + self.bindings.ap_objects + self.bindings.singletons
        return (self.render('docs.lua') + '\n' + ''.join(self.object(obj) for obj in objects)
                + ''.join(self.manual(alias) for alias in self.bindings.globals))
