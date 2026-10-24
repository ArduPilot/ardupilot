# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Generate one Ada entry that retains every typed binding for ABI/packer tests."""

import sys

import build

sys.path.insert(0, str(build.SDK_ROOT / 'generator'))
import ada_bindings  # noqa: E402
import bindings  # noqa: E402


def source(directory):
    types, entries, plans = ada_bindings.describe(bindings.specification())
    path = directory / 'all_calls.adb'
    path.with_suffix('.ads').write_text('''with AP_Module;
package All_Calls is
   function Run (Ctx : AP_Module.Context) return AP_Module.Status
     with Export, Convention => C, External_Name => "ap_module_ada_run";
end All_Calls;
''')
    lines = ['with AP_Module.Bindings;', 'with AP_Module.Types;', 'with AP_Module.Statuses;',
             'with Interfaces;', 'with Interfaces.C;',
             'package body All_Calls is', '   use type AP_Module.Status;',
             '   function Run (Ctx : AP_Module.Context) return AP_Module.Status is',
             '      Status : AP_Module.Status;', '   begin']
    for name, inputs, results, _, _, _ in plans:
        group, fn = name.split('.', 1)
        group = 'parameter_cache' if group == 'Parameter' else group.lower()
        lines += ['      declare']
        args = ['Ctx']
        for index, (_, kind, _) in enumerate(inputs):
            if kind == 'StringView':
                args.append('"x"')
            else:
                item = types.get(kind)
                default = "Interfaces.C.C_bool'Val (1)" if kind == 'bool' else item.default
                lines += [f'         A{index} : {item.ada} := {default};']
                args.append(f'A{index}')
        for index, (_, kind) in enumerate(results):
            item = types.get(kind)
            lines += [f'         R{index} : {item.ada};']
            args.append(f'R{index}')
        lines += ['      begin',
                  f'         Status := AP_Module.Bindings.{ada_bindings.identifier(group)}.'
                  f'{ada_bindings.identifier(fn)} ({", ".join(args)});',
                  '         if Status /= AP_Module.Statuses.OK and then Status /= AP_Module.Statuses.UNAVAILABLE then',
                  '            return AP_Module.Statuses.HOST_ERROR;', '         end if;', '      end;']
    lines += ['      return AP_Module.Statuses.OK;', '   end Run;', 'end All_Calls;']
    path.write_text('\n'.join(lines) + '\n')
    return path, {entry['contract']['name'] for entry in entries}
