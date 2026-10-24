# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compile every Fortran binding and execute its real ISO C boundary."""

import os
import subprocess

import pack
import pytest

from stress_fortran import bindings
from stress_fortran import fortran_bindings
from stress_fortran import source
from test_math_runtime import chibios_probe
from test_runtime import ROOT
from test_runtime import Vehicle
from test_threads import renode  # noqa: F401

import build

NATIVE = 'x86_64-unknown-linux-gnu'
ARM = 'thumbv7em-none-eabihf'
PROBE = ROOT / 'libraries/AP_Module/tests/modules/bindings_fortran.f90'


@pytest.mark.parametrize('target,cpu', [(NATIVE, 'x86-64'), (ARM, 'cortex-m4'), (ARM, 'cortex-m7')])
def test_every_fortran_binding_packs(tmp_path, monkeypatch, target, cpu):
    if target == ARM:
        monkeypatch.setenv('FC', os.environ.get('ARM_FC', 'arm-linux-gnueabihf-gfortran'))
    application, names = source(tmp_path)
    image = build.build(application, tmp_path / 'all.apmod', target, 65536, cpu=cpu)
    assert {entry[2] for entry in pack.unpack(image.read_bytes())['imports']} == names
    second = build.build(application, tmp_path / 'rebased.apmod', target, 65536, cpu=cpu, base=0x24123000)
    assert image.read_bytes() == second.read_bytes()


def test_every_fortran_binding_executes(tmp_path):
    application, names = source(tmp_path)
    image = build.build(application, tmp_path / 'all.apmod', NATIVE, 65536)
    _, _, plans = fortran_bindings.describe(bindings.specification())
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
extern "C" int32_t ap_module_fortran_run(const ContextBase *context);
int main()
{
    AP_ModuleHostApi api{}; api.binding_call = call;
    ContextBase context(api, nullptr);
    for (status = AP_MODULE_STATUS_UNAVAILABLE; status <= 0; status += 3) {
        calls = 0;
        if (ap_module_fortran_run(&context) != AP_MODULE_STATUS_OK) { return 1; }
        printf("%u\\n", calls);
    }
}
'''.replace('BOOL_CALL_INDEX', str(next(i for i, p in enumerate(plans) if p[0] == 'camera.record_video'))))
    assert execute(tmp_path, image, driver) == [str(len(names))] * 2


def execute(directory, image, driver):
    generated = build.GENERATED_ROOT
    objects = sorted(image.with_suffix('.fortran').glob('*.o'))
    assert len(objects) == 3
    executable = directory / 'host'
    subprocess.run(['g++', '-std=c++17', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
                    '-fsanitize=address,undefined', '-g', '-I' + str(ROOT / 'libraries/AP_Module/sdk/cpp'),
                    '-I' + str(generated), '-I' + str(generated / 'sdk/cpp'), str(driver),
                    str(generated / 'sdk/fortran/bridge.cpp'), str(generated / 'sdk/cpp/imports.cpp'),
                    *map(str, objects), '-o', str(executable)], check=True)
    return subprocess.check_output([str(executable)], text=True).splitlines()


def test_fortran_unsigned_bits_booleans_and_strings(tmp_path):
    application = tmp_path / 'bits.f90'
    application.write_text('''function run(ctx) bind(C, name="ap_module_fortran_run") result(status)
    use ap_module_bindings
    implicit none
    type(c_ptr), value :: ctx
    integer(c_int32_t) :: status, word
    integer(c_int8_t) :: byte
    integer(c_int16_t) :: half
    integer(c_int64_t) :: wide
    logical(c_bool) :: flag
    status = gcs_send_text(ctx, mavseverity_info, "x" // c_null_char)
    if (status /= module_invalid) goto 900
    status = gcs_send_text(ctx, mavseverity_info, repeat('x', 256))
    if (status /= module_invalid) goto 900
    status = fence_get_breaches(ctx, byte)
    if (status /= module_ok .or. byte /= -1_c_int8_t) goto 900
    status = mission_num_commands(ctx, half)
    if (status /= module_ok .or. half /= -1_c_int16_t) goto 900
    status = fence_get_breach_time(ctx, word)
    if (status /= module_ok .or. word /= -1_c_int32_t) goto 900
    status = gps_time_epoch_usec(ctx, -1_c_int8_t, wide)
    if (status /= module_ok .or. wide /= -1_c_int64_t) goto 900
    status = camera_record_video(ctx, -1_c_int8_t, .true._c_bool, flag)
    if (status /= module_ok .or. .not. flag) goto 900
    status = gcs_send_text(ctx, 99_c_int32_t, 'x  ')
    if (status /= module_invalid) goto 900
    status = module_ok
    return
900 status = module_host_error
end function
''')
    image = build.build(application, tmp_path / 'bits.apmod', NATIVE, 8192)
    driver = tmp_path / 'bits.cpp'
    driver.write_text('''#include <ap_module.hpp>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
using namespace ap_module;
static unsigned calls;
static int32_t call(void *, uint32_t, const uint8_t *input, uint32_t ni, uint8_t *output, uint32_t size)
{
    static const unsigned expected_input[] = {0, 0, 0, 1, 2, 260};
    static const unsigned expected_output[] = {1, 2, 4, 8, 1, 0};
    assert(calls < 6 && ni == expected_input[calls] && size == expected_output[calls]);
    if (calls == 3) { assert(input[0] == 255); }
    if (calls == 4) { assert(input[0] == 255 && input[1] == 1); }
    for (uint32_t i = 0; i < size; i++) { output[i] = calls == 4 ? 1 : 255; }
    calls++;
    if (ni == 260) {
        assert(input[0] == 99 && input[4] == 'x' && input[5] == ' ' && input[6] == ' ' && input[7] == 0);
        return AP_MODULE_STATUS_INVALID;
    }
    return AP_MODULE_STATUS_OK;
}
namespace ap_module { [[noreturn]] void fatal(const char *) { abort(); } }
extern "C" int32_t ap_module_fortran_run(const ContextBase *);
int main() {
    AP_ModuleHostApi api{}; api.binding_call = call; ContextBase context(api, nullptr);
    assert(ap_module_fortran_run(&context) == 0 && calls == 6);
    printf("bits and strings checked\\n");
}
''')
    assert execute(tmp_path, image, driver) == ['bits and strings checked']


def test_generated_values_and_strings_sitl(tmp_path):
    image = build.build(PROBE, tmp_path / 'bindings.apmod', NATIVE, 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {})
    (vehicle.directory / 'fortran-fixture.bin').write_bytes(b'abc')
    try:
        vehicle.start()
        for generation in range(3):
            mark = len(vehicle.texts) if generation else 0
            if generation:
                vehicle.restart(generation)
            vehicle.text('Fortran AHRS checked', after=mark)
            vehicle.text('Fortran bindings checked', after=mark)
            vehicle.text('MATH ok', after=mark)
        assert not any('returned -' in text or 'panic' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()


def test_generated_values_h7(tmp_path, renode, monkeypatch):  # noqa: F811
    monkeypatch.setenv('FC', os.environ.get('ARM_FC', 'arm-linux-gnueabihf-gfortran'))
    builder = build.build

    def image(unused, output, target, stack_size):
        return builder(PROBE, output, target, stack_size, cpu='cortex-m7')
    monkeypatch.setattr(build, 'build', image)
    chibios_probe(tmp_path, renode, 'fortran', files={'fortran-fixture.bin': b'abc'})
