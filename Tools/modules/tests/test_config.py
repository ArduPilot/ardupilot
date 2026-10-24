# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compile-time scripting selections, including legacy board overrides."""

import subprocess

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]


def compile_config(defines, lua, rust, scripting, rust_first=False, periph=False):
    source = ('#include <AP_HAL_ChibiOS/hwdef/scripts/defaults_periph.h>\n' if periph else '')
    source += ('#include <AP_Module/AP_Module_config.h>\n' if rust_first else '')
    source += '#include <AP_Scripting/AP_Scripting_config.h>\n'
    for name, expected in [('AP_SCRIPTING_LUA_ENABLED', lua), ('AP_MODULE_ENABLED', rust),
                           ('AP_SCRIPTING_ENABLED', scripting), ('HAL_OWNED_THREADS_ENABLED', rust),
                           ('HAL_EXECUTABLE_MEMORY_ENABLED', rust)]:
        source += f'static_assert(bool({name}) == bool({expected}), "{name}");\n'
    return subprocess.run(['g++', '-x', 'c++', '-std=c++17', '-fsyntax-only', '-I', str(ROOT / 'libraries'),
                           '-DCONFIG_HAL_BOARD=99', '-DAP_FILESYSTEM_POSIX_ENABLED=1', '-DHAL_LOGGING_FILESYSTEM_ENABLED=0',
                           *[f'-D{name}={value}' for name, value in defines.items()], '-'],
                          input=source, text=True, capture_output=True)


@pytest.mark.parametrize('lua,rust', [(0, 0), (1, 0), (0, 1), (1, 1)])
@pytest.mark.parametrize('rust_first', [False, True])
def test_independent_runtimes(lua, rust, rust_first):
    result = compile_config({'AP_SCRIPTING_LUA_ENABLED': lua, 'AP_MODULE_ENABLED': rust},
                            lua, rust, lua or rust, rust_first)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('size,lua,rust', [(1024, 0, 0), (2048, 1, 0), (2049, 1, 1), (4096, 1, 1)])
@pytest.mark.parametrize('rust_first', [False, True])
def test_default_runtime_selection(size, lua, rust, rust_first):
    result = compile_config({'HAL_PROGRAM_SIZE_LIMIT_KB': size}, lua, rust, lua or rust, rust_first)
    assert result.returncode == 0, result.stderr


def test_large_board_can_disable_rust():
    result = compile_config({'HAL_PROGRAM_SIZE_LIMIT_KB': 4096, 'AP_MODULE_ENABLED': 0}, 1, 0, 1)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('rust_first', [False, True])
def test_webassembly_retains_lua_without_native_execution(rust_first):
    result = compile_config({'HAL_PROGRAM_SIZE_LIMIT_KB': 4096, '__EMSCRIPTEN__': 1},
                            1, 0, 1, rust_first)
    assert result.returncode == 0, result.stderr


def test_large_board_legacy_disable():
    result = compile_config({'HAL_PROGRAM_SIZE_LIMIT_KB': 4096, 'AP_SCRIPTING_ENABLED': 0}, 0, 0, 0)
    assert result.returncode == 0, result.stderr


def test_hal_only_default_selection():
    result = subprocess.run(['g++', '-x', 'c++', '-std=c++17', '-fsyntax-only',
                             '-I', str(ROOT / 'libraries'), '-DCONFIG_HAL_BOARD=99',
                             '-DHAL_PROGRAM_SIZE_LIMIT_KB=4096', '-'], text=True, capture_output=True,
                            input='#include <AP_HAL/AP_HAL_Boards.h>\n'
                            'static_assert(AP_MODULE_ENABLED && HAL_OWNED_THREADS_ENABLED && '
                            'HAL_EXECUTABLE_MEMORY_ENABLED, "HAL-only inclusion");\n')
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('enabled', [0, 1])
@pytest.mark.parametrize('periph', [False, True])
def test_legacy_umbrella_override(enabled, periph):
    result = compile_config({'AP_SCRIPTING_ENABLED': enabled, 'HAL_PROGRAM_SIZE_LIMIT_KB': 1024},
                            enabled, 0, enabled, periph=periph)
    assert result.returncode == 0, result.stderr


def test_runtime_cannot_bypass_disabled_umbrella():
    result = compile_config({'AP_SCRIPTING_ENABLED': 0, 'AP_MODULE_ENABLED': 1}, 0, 1, 0)
    assert result.returncode != 0
    assert 'A scripting runtime requires AP_SCRIPTING_ENABLED' in result.stderr


def test_umbrella_requires_a_runtime():
    result = compile_config({'AP_SCRIPTING_ENABLED': 1, 'AP_SCRIPTING_LUA_ENABLED': 0,
                             'AP_MODULE_ENABLED': 0}, 0, 0, 1)
    assert result.returncode != 0
    assert 'AP_SCRIPTING_ENABLED requires a scripting runtime' in result.stderr


@pytest.mark.parametrize('rust', [0, 1])
def test_periph_defaults_and_rust_only(rust):
    result = compile_config({'AP_MODULE_ENABLED': rust}, 0, rust, rust, periph=True)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('rust', [None, 0, 1])
@pytest.mark.parametrize('rust_first', [False, True])
@pytest.mark.parametrize('periph', ['', 1])
def test_periph_module_selection(rust, rust_first, periph):
    defines = {'HAL_PROGRAM_SIZE_LIMIT_KB': 4096, 'HAL_BUILD_AP_PERIPH': periph,
               'AP_SCRIPTING_LUA_ENABLED': 0}
    if rust is not None:
        defines['AP_MODULE_ENABLED'] = rust
    expected = rust or 0
    result = compile_config(defines, 0, expected, expected, rust_first)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('flags,enabled', [
    ([], 1), (['-m32'], 0), (['-D__CYGWIN__'], 0), (['-m32', '-DAP_MODULE_ENABLED=1'], 1),
])
def test_sitl_host_runtime_defaults(tmp_path, flags, enabled):
    # Only replace the board header, so the platform selection uses the real
    # HAL defaults without requiring a 32-bit host C++ library installation.
    board = tmp_path / 'AP_HAL/board/sitl.h'
    board.parent.mkdir(parents=True)
    board.write_text('#define HAL_PROGRAM_SIZE_LIMIT_KB 4096\n'
                     '#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE\n')
    source = '#include <AP_HAL/AP_HAL_Boards.h>\n'
    for name in ('AP_MODULE_ENABLED', 'HAL_OWNED_THREADS_ENABLED', 'HAL_EXECUTABLE_MEMORY_ENABLED'):
        source += f'static_assert(bool({name}) == bool({enabled}), "{name}");\n'
    result = subprocess.run(['g++', '-x', 'c++', '-std=c++17', '-fsyntax-only', '-nostdinc++',
                             '-I', str(tmp_path), '-I', str(ROOT / 'libraries'), '-DCONFIG_HAL_BOARD=3',
                             *flags, '-'], input=source, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
