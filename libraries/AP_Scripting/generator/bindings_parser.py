# AP_FLAKE8_CLEAN
"""Parse the line-oriented bindings.desc language without generating code."""

import re

from model import NUMERIC_TYPES
from model import OPERATORS
from model import PRIMITIVE_TYPES
from model import Alias
from model import BindingError
from model import Bindings
from model import ErrorCode
from model import Field
from model import Header
from model import Method
from model import Object
from model import Type


class Tokens:
    """A line cursor. Raw remainders preserve C++ expressions and warning text."""

    def __init__(self, text, line):
        self.text = text
        self.line = line
        self.position = 0

    def error(self, message, code=ErrorCode.USERDATA):
        raise BindingError(message, self.line, code)

    def pop(self):
        match = re.search(r'\S+', self.text[self.position:])
        if match is None:
            return None
        self.position += match.end()
        token = match.group()
        if token == '--':
            self.position = len(self.text)
            return None
        return token

    def require(self, description, code=ErrorCode.USERDATA):
        token = self.pop()
        if token is None:
            self.error(f'Missing {description}', code)
        return token

    def remainder(self, description, code=ErrorCode.USERDATA):
        text = self.text[self.position + 1:]
        self.position = len(self.text)
        if not text.strip():
            self.error(f'Missing {description}', code)
        return text

    def finish(self, code=ErrorCode.UNKNOWN_KEYWORD):
        token = self.pop()
        if token is not None:
            self.error(f'Extra token provided: {token}', code)

    def integer(self, description, code=ErrorCode.USERDATA):
        token = self.require(description, code)
        try:
            return int(token)
        except ValueError:
            self.error(f'Expected an integer for {description}, got: {token}', code)


def parse_range(tokens):
    return tokens.require('low value for a range check'), tokens.require('high value for a range check')


def parse_type(tokens, bindings, argument=False, field=False):
    token = tokens.require('data type')
    address = token.startswith('&')
    name, _, attribute = token.lstrip('&').partition("'")
    if attribute not in ('', 'enum', 'literal', 'Null', 'Ref', 'skip_check'):
        tokens.error(f"Unknown attribute: '{attribute}")
    if field and attribute == 'Null':
        tokens.error(f'{token} is not nullable in this context')
    if address and attribute not in ('Null', 'Ref'):
        tokens.error("Only support references access with 'Null or 'Ref keyword")

    if name in PRIMITIVE_TYPES:
        kind = name
    elif attribute == 'enum':
        kind = 'enum'
    elif attribute == 'literal':
        kind = 'literal'
    elif name == 'rename':
        tokens.error('Cant add rename to unknown function')
    elif any(obj.name == name for obj in bindings.ap_objects):
        kind = 'ap_object'
    else:
        kind = 'userdata'
    result = Type(kind, name, attribute, address)
    if result.skip_check and kind not in NUMERIC_TYPES:
        tokens.error(f'{name} types cannot skip range check')
    if result.output and kind in ('ap_object', 'literal', 'void'):
        tokens.error(f'{name} types cannot be nullable or passed as reference')
    if argument and kind in NUMERIC_TYPES + ('enum',) and not result.output and not result.skip_check:
        result.range = parse_range(tokens)
    return result


def find_member(members, name):
    return next((member for member in members if member.name == name), None)


def parse_method(tokens, bindings, obj):
    name = tokens.require(f'method name for {obj.name}')
    existing = find_member(obj.methods, name)
    if existing:
        action = tokens.require('method modifier')
        if action == 'rename':
            existing.rename = tokens.require('renamed method name')
        elif action in ('depends', 'deprecate'):
            value = tokens.remainder(f'{action} string')
            setattr(existing, {'depends': 'dependency', 'deprecate': 'deprecate'}[action], value)
        elif action == 'alias':
            alias_name = tokens.require('alias name')
            if find_member(obj.methods, alias_name):
                tokens.error(f'Method {alias_name} already exists for {obj.name}')
            obj.aliases.insert(0, Alias(name, alias_name, tokens.line))
        else:
            tokens.error(f'Method {name} already exists for {obj.name} (declared on {existing.line})')
        return
    if any(alias.lua_name == name for alias in obj.aliases):
        tokens.error(f'alias {name} already exists for {obj.name}')

    method = Method(name, tokens.line, parse_type(tokens, bindings))
    while True:
        saved_position = tokens.position
        if tokens.pop() is None:
            break
        tokens.position = saved_position
        arg = parse_type(tokens, bindings, argument=True)
        if arg.kind == 'void':
            tokens.error("Can't pass an empty argument to a method")
        if arg.nullable and method.return_type.kind != 'boolean':
            tokens.error('Nullable arguments are only available on a boolean method')
        if arg.attribute == 'Ref' and method.return_type.kind == 'boolean':
            tokens.error("Use Nullable arguments on a boolean method, not 'Ref")
        method.arguments.append(arg)
    obj.methods.insert(0, method)


def parse_field(tokens, bindings, obj):
    name, _, attribute = tokens.require(f'field name for {obj.name}').partition("'")
    existing = find_member(obj.fields, name)
    if existing:
        action = tokens.require('field modifier')
        if action == 'rename':
            existing.rename = tokens.require('renamed field name')
        elif action == 'depends':
            existing.dependency = tokens.remainder('dependency string')
        else:
            tokens.error(f'Field {name} already exists in {obj.name} (declared on {existing.line})')
        return
    if attribute not in ('', 'array'):
        tokens.error(f"Unknown field attribute '{attribute}")
    array_len = tokens.require('array length') if attribute else None
    member = Field(name, tokens.line, parse_type(tokens, bindings, field=True), array_len=array_len)
    while True:
        action = tokens.pop()
        if action is None:
            break
        if action == 'read':
            member.read = True
        elif action == 'write':
            member.write = True
            if member.type.kind in NUMERIC_TYPES + ('enum',) and not member.type.skip_check:
                member.type.range = parse_range(tokens)
        elif action == 'valid_mask':
            member.type.valid_mask = (tokens.require('valid mask field'), tokens.require('valid mask value'))
        else:
            tokens.error(f'Unknown access provided: {action}', code=ErrorCode.UNKNOWN_KEYWORD)
    if not member.read and not member.write:
        tokens.error('Expected to find an access specifier')
    if member.type.kind in ('void', 'literal'):
        tokens.error(f"Can't access a {member.type.kind} field", code=ErrorCode.INTERNAL)
    if member.read and member.type.kind == 'ap_object':
        tokens.error(f"Can't access a {member.type.kind} field")
    obj.fields.insert(0, member)


def parse_manual(tokens, kind='manual'):
    lua_name = tokens.require('Lua name for manual method', code=ErrorCode.SINGLETON)
    cpp_name = tokens.require('C++ name for manual method', code=ErrorCode.SINGLETON)
    alias = Alias(cpp_name, lua_name, tokens.line, kind)
    if kind != 'manual_operator':
        alias.num_args = tokens.integer('number of arguments', code=ErrorCode.SINGLETON)
        alias.num_returns = tokens.integer('number of returns', code=ErrorCode.SINGLETON)
    action = tokens.pop()
    if action is not None:
        if action != 'depends':
            tokens.error(f'Expected depends keyword for manual method, got: {action}', code=ErrorCode.SINGLETON)
        alias.dependency = tokens.remainder('dependency string')
    return alias


# Explicit language capabilities keep an object's supported directives visible.
DIRECTIVES = {
    'userdata': {'field', 'operator', 'method', 'enum', 'rename', 'depends', 'manual',
                 'creation', 'manual_operator', 'operator_getter'},
    'singleton': {'rename', 'semaphore', 'scheduler-semaphore', 'semaphore-pointer', 'method',
                  'enum', 'depends', 'get', 'literal', 'field', 'reference', 'manual'},
    'ap_object': {'rename', 'semaphore', 'scheduler-semaphore', 'semaphore-pointer',
                  'method', 'depends', 'manual', 'field'},
}


def parse_object(tokens, bindings, kind):
    name = tokens.require(f'name for the {kind}')
    objects = getattr(bindings, {'userdata': 'userdata', 'singleton': 'singletons', 'ap_object': 'ap_objects'}[kind])
    obj = next((obj for obj in objects if obj.name == name), None)
    if obj is None:
        obj = Object(name, kind)
        objects.insert(0, obj)
    object_code = ErrorCode.USERDATA if kind == 'userdata' else ErrorCode.SINGLETON
    action = tokens.require(f'access type for {kind} {name}', code=object_code)
    if action not in DIRECTIVES[kind]:
        tokens.error(f'Unknown or unsupported directive for {kind}: {action}', code=object_code)
    if action == 'method':
        parse_method(tokens, bindings, obj)
    elif action == 'field':
        parse_field(tokens, bindings, obj)
    elif action in ('manual', 'manual_operator'):
        obj.aliases.insert(0, parse_manual(tokens, action))
    elif action == 'enum':
        while True:
            enum = tokens.pop()
            if enum is None:
                break
            obj.enums.insert(0, enum)
    elif action == 'operator':
        symbol = tokens.require('operator symbol')
        if symbol not in OPERATORS:
            tokens.error(f'Unknown operation type: {symbol}')
        if symbol in obj.operators:
            tokens.error(f'Operation {symbol} was already defined for {name}')
        obj.operators.append(symbol)
        tokens.finish(code=ErrorCode.USERDATA)
    elif action == 'rename':
        if obj.rename is not None and kind != 'userdata':
            tokens.error(f'Rename was already declared for {name}', code=ErrorCode.SINGLETON)
        obj.rename = tokens.require('renamed object name', code=ErrorCode.SINGLETON)
    elif action in ('depends', 'get'):
        attr = 'dependency' if action == 'depends' else 'get'
        if getattr(obj, attr) is not None:
            tokens.error(f'{kind} only supports a single {action}', code=ErrorCode.SINGLETON)
        setattr(obj, attr, tokens.remainder(f'{action} string', code=ErrorCode.DEPENDS))
    elif action in ('creation', 'operator_getter'):
        if getattr(obj, action) is not None:
            code = ErrorCode.SINGLETON if action == 'creation' else ErrorCode.USERDATA
            tokens.error(f'Userdata only supports a single {action}', code=code)
        setattr(obj, action, tokens.require(f'{action} function'))
        if action == 'creation':
            obj.creation_args = tokens.integer('number of creation arguments', code=ErrorCode.SINGLETON)
    elif action in ('literal', 'reference'):
        setattr(obj, action, True)
    else:
        # Preserve the historical singleton semaphore-pointer spelling's behavior.
        semaphore = 'scheduler-semaphore' if kind == 'singleton' and action == 'semaphore-pointer' else action
        if kind == 'ap_object' and obj.semaphore and obj.semaphore != semaphore:
            tokens.error('Taking multiple types of semaphore is prohibited', code=ErrorCode.SINGLETON)
        # C tests these flags in this priority order, even if several are set.
        priority = ('semaphore', 'semaphore-pointer', 'scheduler-semaphore')
        if obj.semaphore is None or priority.index(semaphore) < priority.index(obj.semaphore):
            obj.semaphore = semaphore
    if kind != 'userdata':
        tokens.finish(code=ErrorCode.HEADER)


def parse(text):
    bindings = Bindings()
    for line, text_line in enumerate(text.splitlines(), 1):
        tokens = Tokens(text_line, line)
        kind = tokens.pop()
        if kind is None:
            continue
        if kind == 'include':
            name = tokens.require('header name', code=ErrorCode.HEADER)
            existing = find_member(bindings.headers, name)
            if existing:
                tokens.error(f'Header {name} was already included on line {existing.line}', code=ErrorCode.HEADER)
            header = Header(name, line)
            action = tokens.pop()
            if action is not None:
                if action != 'depends':
                    tokens.error(f'Unsupported keyword on a header: {action}', code=ErrorCode.HEADER)
                header.dependency = tokens.remainder('dependency string', code=ErrorCode.HEADER)
            bindings.headers.insert(0, header)
        elif kind in DIRECTIVES:
            parse_object(tokens, bindings, kind)
        elif kind == 'global':
            if tokens.require('global directive', code=ErrorCode.GLOBALS) != 'manual':
                tokens.error('globals only support manual keyword', code=ErrorCode.GLOBALS)
            bindings.globals.insert(0, parse_manual(tokens))
            tokens.finish(code=ErrorCode.GLOBALS)
        else:
            tokens.error(f'Expected a keyword, got: {kind}', code=ErrorCode.UNKNOWN_KEYWORD)
        tokens.finish()

    for obj in bindings.userdata:
        if not (obj.fields or obj.methods or obj.aliases):
            raise BindingError(f'Userdata {obj.name} has no fields or methods')
    return bindings
