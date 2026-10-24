# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compile every Ada API and exercise the real Ada/C bridge in a mock host."""

import subprocess

import pack
import pytest

from stress import ada_bindings
from stress import bindings
from stress import source
from test_ada import ARM
from test_ada import NATIVE
from test_ada import gnat  # noqa: F401
from test_math_runtime import chibios_probe
from test_runtime import ROOT
from test_runtime import Vehicle
from test_threads import renode  # noqa: F401

import build

PROBE = ROOT / 'libraries/AP_Module/tests/modules/bindings_ada.adb'


@pytest.mark.parametrize('target,cpu', [(NATIVE, 'x86-64'), (ARM, 'cortex-m4'), (ARM, 'cortex-m7')])
def test_every_ada_binding_packs(tmp_path, gnat, target, cpu):  # noqa: F811
    gnat(target)
    application, names = source(tmp_path)
    image = build.build(application, tmp_path / 'all.apmod', target, 65536, cpu=cpu)
    assert {entry[2] for entry in pack.unpack(image.read_bytes())['imports']} == names
    second = build.build(application, tmp_path / 'rebased.apmod', target, 65536, cpu=cpu, base=0x24123000)
    assert image.read_bytes() == second.read_bytes()


def test_every_ada_binding_executes(tmp_path, gnat):  # noqa: F811
    gnat(NATIVE)
    application, names = source(tmp_path)
    image = build.build(application, tmp_path / 'all.apmod', NATIVE, 65536)
    generated = build.GENERATED_ROOT
    _, _, plans = ada_bindings.describe(bindings.specification())
    bool_index = next(i for i, plan in enumerate(plans) if plan[0] == 'camera.record_video')
    driver = tmp_path / 'host.cpp'
    driver.write_text('''#include <ap_module.hpp>
#include <stdio.h>
#include <stdlib.h>
using namespace ap_module;
static unsigned calls;
static int32_t status;
static int32_t call(void *, uint32_t, const uint8_t *input, uint32_t ni, uint8_t *output, uint32_t size)
{
    if (calls == BOOL_CALL_INDEX && (ni != 2 || input[1] != 1)) { abort(); }
    calls++;
    for (uint32_t i = 0; i < size; i++) { output[i] = 0; }
    return status;
}
namespace ap_module {
[[noreturn]] void fatal(const char *message) { fprintf(stderr, "%s\\n", message); abort(); }
}
extern "C" int32_t ap_module_ada_run(const ContextBase *context);
int main()
{
    AP_ModuleHostApi api{};
    api.binding_call = call;
    ContextBase context(api, nullptr);
    for (status = AP_MODULE_STATUS_UNAVAILABLE; status <= 0; status += 3) {
        calls = 0;
        if (ap_module_ada_run(&context) != AP_MODULE_STATUS_OK) { return 1; }
        printf("%u\\n", calls);
    }
}
'''.replace('BOOL_CALL_INDEX', str(bool_index)))
    objects = sorted(image.with_suffix('.ada').glob('*/*.o'))
    assert len(objects) == 4
    executable = tmp_path / 'host'
    subprocess.run(['g++', '-std=c++17', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
                    '-fsanitize=address,undefined', '-g',
                    '-I' + str(ROOT / 'libraries/AP_Module/sdk/cpp'), '-I' + str(generated),
                    '-I' + str(generated / 'sdk/cpp'), str(driver),
                    str(ROOT / 'libraries/AP_Module/sdk/ada/bridge.cpp'), str(generated / 'sdk/ada/bridge.cpp'),
                    str(generated / 'sdk/cpp/imports.cpp'), *map(str, objects), '-o', str(executable)], check=True)
    assert subprocess.check_output([str(executable)], text=True).splitlines() == [str(len(names))] * 2


def test_generated_values_and_strings_sitl(tmp_path, gnat):  # noqa: F811
    gnat(NATIVE)
    image = build.build(PROBE, tmp_path / 'bindings.apmod', NATIVE, 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {})
    (vehicle.directory / 'ada-fixture.bin').write_bytes(b'abc')
    try:
        vehicle.start()
        for generation in range(3):
            mark = len(vehicle.texts) if generation else 0
            if generation:
                vehicle.restart(generation)
            vehicle.text('Ada AHRS checked', after=mark)
            vehicle.text('Ada bindings checked', after=mark)
            vehicle.text('MATH ok', after=mark)
        assert not any('returned -' in text or 'panic' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()


def test_generated_values_h7(tmp_path, renode, monkeypatch, gnat):  # noqa: F811
    gnat(ARM)
    builder = build.build

    def image(unused, output, target, stack_size):
        return builder(PROBE, output, target, stack_size, cpu='cortex-m7')
    monkeypatch.setattr(build, 'build', image)
    chibios_probe(tmp_path, renode, 'ada', files={'ada-fixture.bin': b'abc'})
