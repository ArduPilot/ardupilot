# AP_FLAKE8_CLEAN
"""Parsed bindings shared by the C++ and EmmyLua generators."""

import re

from dataclasses import dataclass
from dataclasses import field
from enum import IntEnum
from typing import List
from typing import Optional
from typing import Tuple

# Order matches the C generator's operator bitset, and therefore its output.
OPERATORS = dict(zip(
    ('+', '-', '*', '/', '%', '&', '|', '^', '<<', '>>', '==', '<', '<=', '~'),
    ('__add', '__sub', '__mul', '__div', '__mod', '__band', '__bor', '__bxor',
     '__shl', '__shr', '__eq', '__lt', '__le', '__bnot'),
))
NUMERIC_TYPES = ('float', 'int8_t', 'int16_t', 'int32_t', 'uint8_t', 'uint16_t', 'uint32_t')
PRIMITIVE_TYPES = ('boolean', 'string', 'void') + NUMERIC_TYPES
OUTPUT_ARGUMENT_BASE = 5000


class ErrorCode(IntEnum):
    """Exit statuses defined by the original C binding generator."""

    OUT_OF_MEMORY = 1
    HEADER = 2
    UNKNOWN_KEYWORD = 3
    USERDATA = 4
    INTERNAL = 5
    GENERAL = 6
    SINGLETON = 7
    DEPENDS = 8
    DOCS = 9
    GLOBALS = 10


def sanitize(name):
    """Use the same C identifier spelling as the original generator."""
    return re.sub(r'[:.<>()\-]', '_', name)


class BindingError(Exception):
    """A description error, with the original generator's exit status."""

    def __init__(self, message, line=None, code=ErrorCode.USERDATA):
        super().__init__(message)
        self.line = line
        self.code = code

    def __str__(self):
        location = f' (line {self.line})' if self.line is not None else ''
        return f'Error{location}: {super().__str__()}'


@dataclass
class Type:
    kind: str
    name: str
    attribute: str = ''
    address: bool = False
    range: Optional[Tuple[str, str]] = None
    valid_mask: Optional[Tuple[str, str]] = None

    @property
    def output(self):
        return self.attribute in ('Null', 'Ref')

    @property
    def nullable(self):
        return self.attribute == 'Null'

    @property
    def skip_check(self):
        return self.attribute == 'skip_check'

    @property
    def cpp_name(self):
        return {'boolean': 'bool', 'string': 'char *'}.get(self.kind, self.name)

    @property
    def identifier(self):
        return sanitize(self.name)


@dataclass
class Header:
    name: str
    line: int
    dependency: Optional[str] = None


@dataclass
class Method:
    name: str
    line: int
    return_type: Type
    arguments: List[Type] = field(default_factory=list)
    rename: Optional[str] = None
    deprecate: Optional[str] = None
    dependency: Optional[str] = None

    @property
    def lua_name(self):
        return self.rename or self.name

    @property
    def identifier(self):
        return sanitize(self.name)

    @property
    def nullable(self):
        return any(arg.nullable for arg in self.arguments)

    @property
    def references(self):
        return any(arg.attribute == 'Ref' for arg in self.arguments)

    @property
    def inputs(self):
        return [arg for arg in self.arguments if not arg.output and arg.kind != 'literal']


@dataclass
class Field:
    name: str
    line: int
    type: Type
    read: bool = False
    write: bool = False
    array_len: Optional[str] = None
    rename: Optional[str] = None
    dependency: Optional[str] = None

    @property
    def lua_name(self):
        return self.rename or self.name


@dataclass
class Alias:
    name: str
    lua_name: str
    line: int
    kind: str = 'alias'
    num_args: int = 0
    num_returns: int = 0
    dependency: Optional[str] = None


@dataclass
class Object:
    name: str
    kind: str
    fields: List[Field] = field(default_factory=list)
    methods: List[Method] = field(default_factory=list)
    aliases: List[Alias] = field(default_factory=list)
    enums: List[str] = field(default_factory=list)
    operators: List[str] = field(default_factory=list)
    rename: Optional[str] = None
    dependency: Optional[str] = None
    semaphore: Optional[str] = None
    literal: bool = False
    reference: bool = False
    creation: Optional[str] = None
    creation_args: int = 0
    operator_getter: Optional[str] = None
    get: Optional[str] = None

    @property
    def identifier(self):
        return sanitize(self.name)

    @property
    def lua_name(self):
        return self.rename or self.name

    @property
    def docs_name(self):
        name = self.rename or self.identifier
        return name if self.kind == 'singleton' else name + '_ud'

    @property
    def emit_creation(self):
        if self.creation or self.methods:
            return not (self.creation and self.creation_args == -1)
        return any(member.write for member in self.fields)

    @property
    def has_operators(self):
        return bool(self.operators or any(alias.kind == 'manual_operator' for alias in self.aliases))

    @property
    def needs_checker(self):
        return not self.literal and bool(self.methods)

    @property
    def receiver(self):
        return self.name if self.literal else 'ud'

    @property
    def access(self):
        return '.' if self.reference else '->'


@dataclass
class Bindings:
    # Lists are in emission order: newest declaration first, as in the C generator.
    headers: List[Header] = field(default_factory=list)
    userdata: List[Object] = field(default_factory=list)
    singletons: List[Object] = field(default_factory=list)
    ap_objects: List[Object] = field(default_factory=list)
    globals: List[Alias] = field(default_factory=list)
