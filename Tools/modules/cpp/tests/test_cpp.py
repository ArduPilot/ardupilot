# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compile both C++ target profiles and run Rust/C++ together in SITL."""

import subprocess
import sys

import apmod_debug
import pack
import pytest

from test_runtime import ROOT
from test_runtime import Vehicle

import build


@pytest.mark.parametrize('target', build.TARGETS)
@pytest.mark.parametrize('debug', [False, True])
def test_cpp_rebases_without_rust(tmp_path, target, debug, monkeypatch):
    monkeypatch.setenv('RUSTC', '/no/rust/installation')
    source = ROOT / 'libraries/AP_Module/tests/modules/lifecycle.cpp'
    first = build.build(source, tmp_path / 'first.apmod', target, 8192, debug=debug)
    second = build.build(source, tmp_path / 'second.apmod', target, 8192, base=0x24123000, debug=debug)
    assert first.read_bytes() == second.read_bytes()
    module = pack.unpack(first.read_bytes())
    assert {name for _, _, name in module['imports']} == {'ahrs.get_gyro', 'Vector3f.length', 'gcs.send_text'}
    assert module['contract']['heap_size'] == 4096
    if debug:
        symbols = apmod_debug.ModuleSymbols(first)
        assert not symbols.is_rust


def test_mixed_rust_cpp_sitl(tmp_path, monkeypatch):
    monkeypatch.setenv('AP_MODULE_LIFECYCLE', 'cleanup')
    images = {}
    for ext in ('cpp', 'rs'):
        source = ROOT / f'libraries/AP_Module/tests/modules/lifecycle.{ext}'
        image = build.build(source, tmp_path / f'{ext}.apmod', 'x86_64-unknown-linux-gnu', 8192)
        images[image.name] = image.read_bytes()
    vehicle = Vehicle(tmp_path / 'vehicle', images, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        vehicle.text('CPP ready')
        vehicle.text('Module: ready')
        for generation in range(1, 4):
            mark = len(vehicle.texts)
            vehicle.restart(generation, expected=2)
            vehicle.text('CPP ready', after=mark)
            vehicle.text('Module: ready', after=mark)
        vehicle.arm()
        vehicle.arm(False)
    finally:
        vehicle.stop()


@pytest.mark.parametrize('target', build.TARGETS)
def test_global_constructors_rejected(tmp_path, target):
    source = tmp_path / 'constructor.cpp'
    source.write_text('''#include <ap_module.hpp>
volatile unsigned initialized;
struct Init { Init() { initialized = 1; } } global;
static ap_module::Result<void> run(ap_module::Context &) { return {ap_module::Error::Ok}; }
AP_MODULE(run)
''')
    with pytest.raises((ValueError, subprocess.CalledProcessError)):
        build.build(source, tmp_path / 'constructor.apmod', target, 8192)


def test_allocations_and_parameter_raii_sitl(tmp_path):
    source = tmp_path / 'allocations.cpp'
    source.write_text('''#include <ap_module.hpp>
#include <new>
using namespace ap_module;
struct alignas(64) Aligned { uint32_t value; };
static Result<void> run(Context &c) {
    auto parameter = Parameter::create(c, "MOD_ENABLE");
    if (!parameter) { return {parameter.error}; }
    auto value = parameter.value.get();
    if (!value || !value.value.present || value.value.value != 1.0f) { return {Error::Host}; }
    auto *a = new Aligned{1234};
    auto *b = new uint32_t[10];
    if (reinterpret_cast<uintptr_t>(a) % 64 != 0 || a->value != 1234) { return {Error::Host}; }
    for (unsigned i = 0; i < 10; i++) {
        if (b[i] != 0) { return {Error::Host}; }
        b[i] = 0xabababab;
    }
    delete[] b;
    b = new uint32_t[10];
    for (unsigned i = 0; i < 10; i++) {
        if (b[i] != 0) { return {Error::Host}; }
    }
    b[9] = 5678;
    delete a; delete[] b;
    auto *large = new (std::nothrow) uint8_t[65536];
    if (large != nullptr) { delete[] large; return {Error::Host}; }
    return c.text(6, "CPP allocations ok");
}
AP_MODULE(run)
''')
    image = build.build(source, tmp_path / 'allocations.apmod', 'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        vehicle.text('CPP allocations ok')
        vehicle.text('Module: exited allocations.apmod')
        reports = [t for t in vehicle.texts if t.startswith('Module RAM: heap ')]
        assert 'used 0' in reports[-1], reports
        assert not any('panic' in t or 'returned -' in t for t in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()


@pytest.mark.parametrize('cpu', ['cortex-m4', 'cortex-m7'])
def test_arm_integer_float_conversion(tmp_path, cpu):
    source = tmp_path / 'conversion.cpp'
    source.write_text('''#include <ap_module.hpp>
volatile float timestamp;
static ap_module::Result<void> run(ap_module::Context &c) {
    timestamp = float(c.time_us());
    return {ap_module::Error::Ok};
}
AP_MODULE(run)
''')
    image = build.build(source, tmp_path / 'conversion.apmod', 'thumbv7em-none-eabihf', 8192, cpu=cpu)
    module = pack.unpack(image.read_bytes())
    assert module['memory_size'] - module['rx_size'] >= 4


@pytest.mark.parametrize('target', build.TARGETS)
def test_all_cpp_bindings_pack(tmp_path, target):
    # The host sanitizer test checks wire sizes/bytes. Also retain every client
    # here to catch target-specific compiler helpers or unsupported relocations.
    sys.path.insert(0, str(build.SDK_ROOT / 'generator'))
    import bindings
    import cpp_bindings
    _, entries, plans = cpp_bindings.describe(bindings.specification())
    lines = ['#include <ap_module.hpp>', 'using namespace ap_module;',
             'static volatile uint8_t checksum;', 'static volatile int32_t status;',
             '__attribute__((noinline)) static int32_t call(void *, uint32_t, const uint8_t *p, uint32_t ni,',
             '                                             uint8_t *out, uint32_t no) {',
             '    for (uint32_t i = 0; i < ni; i++) { checksum ^= p[i]; }',
             '    for (uint32_t i = 0; i < no; i++) { out[i] = 0; }',
             '    return status;', '}',
             'static Result<void> run(Context &) {',
             'AP_ModuleHostApi api{}; api.binding_call = call;', 'ContextBase base(api, nullptr); Context c(base);']
    for name, inputs, _, _, _, _ in plans:
        group, _, method = name.partition('.')
        group = 'parameter_cache' if group == 'Parameter' else group.lower()
        lines += ['{']
        args = []
        for i, (_, kind, _) in enumerate(inputs):
            lines += [f'{kind} a{i}' + ('("x");' if kind == 'StringView' else '{};')]
            args.append(f'a{i}')
        lines += [f'(void)c.{cpp_bindings.identifier(group)}().{cpp_bindings.identifier(method)}({", ".join(args)});', '}']
    lines += ['return {Error::Ok};', '}', 'AP_MODULE(run)']
    source = tmp_path / 'all.cpp'
    source.write_text('\n'.join(lines))
    image = build.build(source, tmp_path / 'all.apmod', target, 65536)
    assert {item[2] for item in pack.unpack(image.read_bytes())['imports']} == {
        entry['contract']['name'] for entry in entries}


@pytest.mark.parametrize('target', build.TARGETS)
def test_runtime_helpers_pack(tmp_path, target):
    build.build(ROOT / 'Tools/modules/cpp/probe.cpp', tmp_path / 'helpers.apmod', target, 8192)


def test_runtime_helpers_sitl(tmp_path):
    image = build.build(ROOT / 'Tools/modules/cpp/probe.cpp', tmp_path / 'helpers.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        vehicle.text('ready')
        vehicle.restart(1)
        vehicle.text('ready')
    finally:
        vehicle.stop()


def test_cli_requires_initial_heap(tmp_path):
    result = subprocess.run([sys.executable, str(ROOT / 'Tools/modules/build.py'),
                             str(ROOT / 'Tools/modules/cpp/probe.cpp'), str(tmp_path / 'unused.apmod'),
                             '--target', 'thumbv7em-none-eabihf', '--stack-size', '8192'],
                            capture_output=True, text=True)
    assert result.returncode == 2 and '--heap-size' in result.stderr


def test_rust_backend_cli_help():
    result = subprocess.run([sys.executable, str(ROOT / 'Tools/modules/rust/build.py'), '--help'],
                            capture_output=True, text=True)
    assert result.returncode == 0 and '--heap-size' in result.stdout
