# AP_FLAKE8_CLEAN
"""Export parsed binding contracts for native binding generators."""

import json

from model import OPERATORS


def export_type(value):
    """Return the stable schema-1 representation of a parsed type."""
    named_kinds = ('enum', 'literal', 'userdata', 'ap_object')
    return {
        'kind': value.kind,
        'name': value.name if value.kind in named_kinds else None,
        'nullable': value.nullable,
        'reference': value.attribute == 'Ref',
        'pointer': value.address,
        'skip_check': value.skip_check,
        'range': list(value.range) if value.range is not None else None,
    }


def export_field(field):
    """Return the stable schema-1 representation of a userdata field."""
    valid_mask_name = None
    valid_mask_value = None
    if field.type.valid_mask is not None:
        valid_mask_name, valid_mask_value = field.type.valid_mask
    return {
        'name': field.name,
        'rename': field.rename,
        'depends': field.dependency,
        'array_len': field.array_len,
        'valid_mask_name': valid_mask_name,
        'valid_mask_value': valid_mask_value,
        'line': field.line,
        'read': field.read,
        'write': field.write,
        'type': export_type(field.type),
    }


def export_method(method):
    """Return the stable schema-1 representation of a method."""
    return {
        'name': method.name,
        'rename': method.rename,
        'depends': method.dependency,
        'deprecate': method.deprecate,
        'line': method.line,
        'return': export_type(method.return_type),
        'arguments': [export_type(argument) for argument in method.arguments],
    }


def export_alias(alias):
    """Return the stable schema-1 representation of an alias."""
    return {
        'name': alias.name,
        'alias': alias.lua_name,
        'depends': alias.dependency,
        'manual': alias.kind != 'alias',
    }


def operation_mask(owner):
    """Preserve the old C generator's presence-marker bit layout."""
    mask = 0
    for index, symbol in enumerate(OPERATORS):
        if symbol in owner.operators:
            mask |= 1 << index
    if any(alias.kind == 'manual_operator' for alias in owner.aliases):
        mask |= 1 << len(OPERATORS)
    return mask


def export_owner(owner, userdata):
    """Return the common singleton/userdata schema, with userdata metadata."""
    result = {
        'name': owner.name,
        'rename': owner.rename,
        'depends': owner.dependency,
        'get': owner.get,
    }
    if userdata:
        manual_operator = any(alias.kind == 'manual_operator' for alias in owner.aliases)
        result.update({
            'creation': owner.creation,
            'creation_manual': owner.creation is not None,
            'creation_args': owner.creation_args,
            'has_fields': bool(owner.fields),
            'has_enums': bool(owner.enums),
            'operations': operation_mask(owner),
            'operator_getter': owner.operator_getter,
            'operators_manual': manual_operator,
            'operator_names': [symbol for symbol in OPERATORS if symbol in owner.operators],
        })
    result['fields'] = [export_field(field) for field in owner.fields]
    result.update({
        'literal': owner.literal,
        'reference': owner.reference,
        'semaphore': owner.semaphore == 'semaphore',
        'scheduler_semaphore': owner.semaphore == 'scheduler-semaphore',
        'semaphore_pointer': owner.semaphore == 'semaphore-pointer',
        'methods': [export_method(method) for method in owner.methods],
        'aliases': [export_alias(alias) for alias in owner.aliases],
    })
    return result


def export(bindings):
    """Return schema 1 from the parser's already-validated model."""
    return {
        'schema': 1,
        'includes': [
            {'name': header.name, 'depends': header.dependency}
            for header in bindings.headers
        ],
        'singletons': [export_owner(owner, False) for owner in bindings.singletons],
        'userdata': [export_owner(owner, True) for owner in bindings.userdata],
        'ap_objects': [export_owner(owner, True) for owner in bindings.ap_objects],
    }


def dumps(bindings):
    """Serialize schema 1 deterministically in the original compact form."""
    return json.dumps(export(bindings), separators=(',', ':')) + '\n'
