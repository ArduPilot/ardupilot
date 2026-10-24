# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Audited native adapter for the same Parameter helper used by Lua."""

from native_templates import render

ARGUMENTS = {
    'init': [('string', False, False)],
    'get': [('float', True, False)],
    'configured': [],
    'set': [('float', False, True)],
    'set_and_save': [('float', False, True)],
    'set_default': [('float', False, True)],
}
POLICY = 'v2:Parameter,placement-init,host-layout,trivial-drop,receiver-le-u64,checked-f32'
MAX_SIZE = 16 * 1024 * 1024
MAX_ALIGNMENT = 4096


def layout_policy():
    return dict(ownership='module-heap,trivial-drop', size_min=1, size_max=MAX_SIZE,
                alignment='power-of-two,size-multiple', alignment_max=MAX_ALIGNMENT)


def select(ir, config, first_id):
    if config != {'include': 'AP_Scripting/AP_Scripting_helpers.h', 'methods': sorted(ARGUMENTS)}:
        raise ValueError('unsupported Parameter selection')
    if not any(item == {'name': config['include'], 'depends': None} for item in ir['includes']):
        raise ValueError('unaudited Parameter include')
    owner = next((item for item in ir['userdata'] if item['name'] == 'Parameter'), None)
    if owner is None:
        raise ValueError('missing Parameter userdata')
    expected = dict(rename=None, depends=None, get=None, creation='lua_new_Parameter', creation_manual=True,
                    creation_args=0, has_fields=False, has_enums=False, operations=0, operator_getter=None,
                    literal=False, reference=False, semaphore=False, scheduler_semaphore=False,
                    semaphore_pointer=False, aliases=[])
    if any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited Parameter owner metadata')
    methods = {method['name']: method for method in owner['methods']}
    result = []
    for name, arguments in sorted(ARGUMENTS.items()):
        method = methods.get(name)
        if method is None:
            raise ValueError(f'missing Parameter.{name}')
        expected_types = [('boolean', False, False)] + arguments
        types = [method['return']] + method['arguments']
        if (any(method[key] is not None for key in ('rename', 'depends', 'deprecate')) or
                len(types) != len(expected_types) or any(
                    value != dict(kind=kind, name=None, nullable=nullable, reference=False,
                                  pointer=False, skip_check=skip_check, range=None)
                    for value, (kind, nullable, skip_check) in zip(types, expected_types))):
            raise ValueError(f'bindings.desc:{method["line"]}: unaudited Parameter.{name} signature')
        result.append(dict(id=first_id + len(result), name=name, arguments=method['arguments'], result=method['return']))
    return dict(policy=POLICY, methods=result,
                layout=dict(id=first_id + len(result), policy=layout_policy()))


def cpp_cases(contract):
    lines = [f'    case {contract["layout"]["id"]}: {{ // Parameter::__layout',
             '        BindingWire::write(output, uint32_t(sizeof(Parameter)), 4);',
             '        BindingWire::write(output + 4, uint32_t(alignof(Parameter)), 4);',
             '        return AP_MODULE_STATUS_OK;', '    }']
    for method in contract['methods']:
        name = method['name']
        setter = name.startswith('set')
        output = 5 if name == 'get' else 1
        lines += [f'    case {method["id"]}: {{ // Parameter::{name}',
                  f'        memset(output, 0, {output});',
                  '        const uint64_t address = uint64_t(BindingWire::read(input, 4)) |',
                  '                                 (uint64_t(BindingWire::read(input + 4, 4)) << 32);',
                  '        if (address == 0 || address > UINTPTR_MAX || address % alignof(Parameter) != 0) {',
                  '            return AP_MODULE_STATUS_INVALID;', '        }',
                  '        auto *parameter = reinterpret_cast<Parameter *>(uintptr_t(address));']
        if name == 'init':
            lines += ['        const char *name = BindingWire::read_string(input + 8);',
                      '        if (name == nullptr) {', '            return AP_MODULE_STATUS_INVALID;', '        }',
                      '        parameter = new (parameter) Parameter();',
                      '        output[0] = parameter->init(name);']
        elif name == 'get':
            lines += ['        float value;', '        if (parameter->get(value)) {', '            output[0] = 1;',
                      '            BindingWire::write_float(output + 1, value);', '        }']
        elif setter:
            lines += ['        const float value = BindingWire::read_float(input + 8);',
                      '        if (!BindingParam::valid_value(parameter->type(), value)) {',
                      '            return AP_MODULE_STATUS_INVALID;', '        }',
                      f'        output[0] = parameter->{name}(value);']
        else:
            lines += ['        output[0] = parameter->configured();']
        lines += ['        return AP_MODULE_STATUS_OK;', '    }']
    return lines


def rust(contract, notice):
    ids = {method['name']: method['id'] for method in contract['methods']}
    return render(
        'rust/parameter.rs', notice=notice, max_size=MAX_SIZE, max_alignment=MAX_ALIGNMENT,
        layout_id=contract['layout']['id'], init_id=ids['init'], get_id=ids['get'],
        configured_id=ids['configured'], set_id=ids['set'], set_and_save_id=ids['set_and_save'],
        set_default_id=ids['set_default'],
    )
