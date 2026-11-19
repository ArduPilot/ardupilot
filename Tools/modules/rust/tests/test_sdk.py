# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build and exercise the generated SDK ABI, independently of the runtime."""

import os
import re
import subprocess
import sys

from pathlib import Path

import pack
import pytest

import build

HERE = Path(__file__).resolve().parents[2]
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'Tools/renode/tests'))
from test_peripherals import renode  # noqa: E402,F401


@pytest.mark.parametrize('target', build.TARGETS)
def test_sdk_rebases_identically(tmp_path, target):
    first = build.build(HERE / 'rust/sdk_probe.rs', tmp_path / 'first.apmod', target, 8192)
    second = build.build(HERE / 'rust/sdk_probe.rs', tmp_path / 'second.apmod', target, 8192, base=0x24123000)
    assert first.read_bytes() == second.read_bytes()


@pytest.mark.parametrize('trait', ['Send', 'Sync'])
@pytest.mark.parametrize('kind', ["Context<'static>", "Parameter<'static, 'static>"])
def test_context_cannot_cross_threads(tmp_path, trait, kind):
    output = build.build(HERE / 'rust/sdk_probe.rs', tmp_path / 'probe.apmod', 'x86_64-unknown-linux-gnu', 8192)
    source = tmp_path / 'negative.rs'
    source.write_text(f'''#![no_std]
fn require<T: {trait}>() {{}}
pub fn check() {{ require::<ap_rust_sdk::{kind}>(); }}
''')
    result = subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--crate-type=rlib',
                             '-C', 'panic=abort', '--extern',
                             f'ap_rust_sdk={output.with_name("libprobe_sdk.rlib")}',
                             str(source), '-o', str(tmp_path / 'negative.rlib')], capture_output=True, text=True)
    assert result.returncode != 0
    assert 'between threads safely' in result.stderr, result.stderr


@pytest.mark.parametrize(('body', 'diagnostic'), [
    ('''pub fn escape(context: &ap_rust_sdk::Context<'static>) -> ap_rust_sdk::Parameter<'static, 'static> {
    context.parameter("VALUE").unwrap().unwrap()
}''', 'lifetime may not live long enough'),
    ('''pub fn reuse(context: &ap_rust_sdk::Context<'_>) {
    let parameter = context.parameter("VALUE").unwrap().unwrap();
    drop(parameter);
    let _ = parameter.get();
}''', 'borrow of moved value'),
    ('''fn require_send<T: FnOnce() + Send>(_: T) {}
pub fn share(context: &ap_rust_sdk::Context<'_>) {
    let parameter = context.parameter("VALUE").unwrap().unwrap();
    require_send(move || { let _ = parameter.get(); });
}''', 'between threads safely'),
])
def test_parameter_lifetime_protections(tmp_path, body, diagnostic):
    output = build.build(HERE / 'rust/sdk_probe.rs', tmp_path / 'probe.apmod', 'x86_64-unknown-linux-gnu', 8192)
    source = tmp_path / 'negative.rs'
    source.write_text('#![no_std]\n' + body + '\n')
    result = subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--crate-type=rlib',
                             '-C', 'panic=abort', '--extern',
                             f'ap_rust_sdk={output.with_name("libprobe_sdk.rlib")}',
                             str(source), '-o', str(tmp_path / 'negative.rlib')], capture_output=True, text=True)
    assert result.returncode != 0
    assert diagnostic in result.stderr, result.stderr


def test_enum_types_cannot_be_interchanged(tmp_path):
    output = build.build(HERE / 'rust/sdk_probe.rs', tmp_path / 'probe.apmod', 'x86_64-unknown-linux-gnu', 8192)
    source = tmp_path / 'wrong_enum.rs'
    source.write_text('''#![no_std]
pub fn check(context: &ap_rust_sdk::Context<'_>) {
    let _ = context.rc().get_aux_cached(ap_rust_sdk::enums::SrvChannelFunction(300));
}
''')
    result = subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--crate-type=rlib',
                             '-C', 'panic=abort', '--extern',
                             f'ap_rust_sdk={output.with_name("libprobe_sdk.rlib")}',
                             str(source), '-o', str(tmp_path / 'wrong_enum.rlib')], capture_output=True, text=True)
    assert result.returncode != 0
    assert 'expected `RcChannelAuxFunc`, found `SrvChannelFunction`' in result.stderr, result.stderr


@pytest.mark.parametrize('body,diagnostic', [
    ('ap_rust_sdk::RcChannel { token: core::num::NonZeroU32::new(1).unwrap() };', 'is private'),
    ('let channel: ap_rust_sdk::RcChannel = panic!(); channel.norm_input();', 'argument'),
])
def test_rc_handle_requires_host_context(tmp_path, body, diagnostic):
    output = build.build(HERE / 'rust/sdk_probe.rs', tmp_path / 'probe.apmod', 'x86_64-unknown-linux-gnu', 8192)
    source = tmp_path / 'bad_handle.rs'
    source.write_text('#![no_std]\npub fn check() { ' + body + ' }\n')
    result = subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--crate-type=rlib',
                             '-C', 'panic=abort', '--extern',
                             f'ap_rust_sdk={output.with_name("libprobe_sdk.rlib")}',
                             str(source), '-o', str(tmp_path / 'bad_handle.rlib')], capture_output=True, text=True)
    assert result.returncode != 0
    assert diagnostic in result.stderr, result.stderr


@pytest.mark.parametrize(('body', 'diagnostic'), [
    ('let mut v = ap_rust_sdk::Location::default(); v.lat = 999;', 'is private'),
    ('let v = ap_rust_sdk::MissionItem::default(); let _ = v.target_system;', 'is private'),
    ('let v = ap_rust_sdk::Location::default(); v.loiter_ccw();', 'no method named'),
    ('let v = ap_rust_sdk::Location::default(); v.set_alt(context, 1);', 'cannot borrow'),
    ('let mut v = ap_rust_sdk::Location::default(); v.get_distance(context, &mut v);', 'more than once'),
])
def test_value_assignment_and_alias_protections(tmp_path, body, diagnostic):
    output = build.build(HERE / 'rust/sdk_probe.rs', tmp_path / 'probe.apmod', 'x86_64-unknown-linux-gnu', 8192)
    source = tmp_path / 'bad_value.rs'
    source.write_text('#![no_std]\npub fn check(context: &ap_rust_sdk::Context<\'_>) { ' + body + ' }\n')
    result = subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--crate-type=rlib',
                             '-C', 'panic=abort', '--extern',
                             f'ap_rust_sdk={output.with_name("libprobe_sdk.rlib")}',
                             str(source), '-o', str(tmp_path / 'bad_value.rlib')], capture_output=True, text=True)
    assert result.returncode != 0
    assert diagnostic in result.stderr, result.stderr


@pytest.mark.parametrize(('cpu', 'ram'), [('cortex-m7', 0x24000000), ('cortex-m4', 0x20000000)])
def test_sdk_renode(tmp_path, renode, cpu, ram):  # noqa: F811
    module = build.build(HERE / 'rust/sdk_probe.rs', tmp_path / 'sdk.apmod', 'thumbv7em-none-eabihf', 8192)
    info = pack.unpack(module.read_bytes())
    assert info['memory_size'] <= 0x6000
    platform = tmp_path / 'sdk.repl'
    platform.write_text(f'''
cpu: CPU.CortexM @ sysbus
    cpuType: "{cpu}"
    nvic: nvic
nvic: IRQControllers.NVIC @ sysbus 0xE000E000
    IRQ -> cpu@0
flash: Memory.MappedMemory @ sysbus 0x08000000
    size: 0x20000
ram: Memory.MappedMemory @ sysbus 0x{ram:x}
    size: 0x20000
''')
    linker = tmp_path / 'sdk.ld'
    linker.write_text('ENTRY(reset)\nSECTIONS { . = 0x08000000; .vectors : { KEEP(*(.vectors)) } '
                      '.text : { *(.text*) *(.rodata*) } }')
    startup = tmp_path / 'startup.S'
    startup.write_text(f'''
.syntax unified
.thumb
.section .vectors,"a"
.word 0x{ram + 0x20000:x}
.word reset
.rept 14
.word fault
.endr
.section .text,"ax"
.thumb_func
reset:
 ldr r0, =0xe000ed88
 ldr r1, =0x00f00000
 str r1, [r0]
 dsb
 isb
 bl exercise
 b .
.thumb_func
fault:
 ldr r0, =0x{ram + 0x110:x}
 ldr r1, =0xbad00001
 str r1, [r0]
 b .
.global invoke
.thumb_func
invoke:
 push {{r9, lr}}
 mov r12, r0
 mov r0, r1
 mov r1, r2
 mov r9, r3
 blx r12
 pop {{r9, pc}}
''')
    source = tmp_path / 'sdk.cpp'
    source.write_text(f'''
#include <AP_Module/Image.h>
#include <AP_Module/HostApi.h>
#include <string.h>
static volatile uint32_t *const result = reinterpret_cast<volatile uint32_t *>(0x{ram + 0x100:x});
static uint8_t *allocate(void *context, uint32_t size, uint32_t alignment)
{{
    if (context != (void *)result || size != 256 || alignment != 4) {{ return nullptr; }}
    result[5]++;
    return reinterpret_cast<uint8_t *>(0x{ram + 0x200:x});
}}
static void deallocate(void *context, uint8_t *pointer)
{{
    if (context == (void *)result && pointer == (uint8_t *)0x{ram + 0x200:x}) {{ result[6]++; }}
}}
static uint64_t time_us(void *) {{ return 0; }}
static uint8_t stop_requested(void *) {{ return 0; }}
static int32_t sleep_ms(void *, uint32_t ms) {{ result[3] += ms; return 0; }}
static int32_t text(void *context, uint8_t severity, const uint8_t *message, uint32_t length)
{{
    const char *expected = result[2] % 2 == 0 ? "sdk start" : "sdk done";
    if (context != (void *)result || severity != 6 || length != strlen(expected) || memcmp(message, expected, length)) {{
        return AP_MODULE_STATUS_INVALID;
    }}
    result[2]++;
    return 0;
}}
static void panic(void *, const uint8_t *, uint32_t) {{ result[4] = 0xbad00002; while (true) {{}} }}
static int32_t binding_call(void *, uint32_t, const uint8_t *, uint32_t, uint8_t *, uint32_t)
{{ return AP_MODULE_STATUS_UNAVAILABLE; }}
// This raw CPU harness deliberately maps every import to an unavailable stub.
static uint32_t resolve(void *, uint64_t) {{ return 1; }}
static const AP_ModuleHostApi host {{ AP_MODULE_ABI_VERSION, sizeof(AP_ModuleHostApi), AP_MODULE_API_FINGERPRINT,
                                  allocate, deallocate, time_us, sleep_ms, stop_requested, text, panic, binding_call }};
extern "C" int32_t invoke(uintptr_t entry, const AP_ModuleHostApi *, void *, void *rw_base);
extern "C" void exercise()
{{
    for (unsigned i = 0; i < 7; i++) {{ result[i] = 0; }}
    const AP_Module::Image::Compatibility expected{{40, AP_MODULE_ABI_VERSION, 1, 8192, AP_MODULE_API_FINGERPRINT}};
    AP_Module::Image image;
    if (!image.parse((const uint8_t *)0x08010000, {module.stat().st_size}, 0x6000, expected)) {{ return; }}
    const uint32_t addresses[] = {{0x{ram + 0x4000:x}, 0x{ram + 0xa000:x}}};
    for (unsigned i = 0; i < 2; i++) {{
        void *rw = reinterpret_cast<void *>(addresses[i] + image.code_size());
        if (!image.load_split((void *)addresses[i], image.code_size(), rw,
                              0x6000 - image.code_size(), resolve)) {{ return; }}
        result[i] = invoke(addresses[i] + image.entry_offset(), &host, (void *)result, rw);
    }}
    result[4] = 0x600df00d;
}}
''')
    elf = tmp_path / 'sdk.elf'
    generated_include = build.generate_bindings().parent
    subprocess.run(['arm-none-eabi-g++', '-nostartfiles', f'-mcpu={cpu}', '-mthumb', '-mfpu=fpv4-sp-d16',
                    '-mfloat-abi=hard', '-std=gnu++17', '-Os', '-fno-exceptions', '-fno-rtti',
                    '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-DAP_MODULE_ENABLED=1',
                    '-DCONFIG_HAL_BOARD=99', '-I' + str(generated_include), '-I' + str(ROOT / 'libraries'),
                    '-T', str(linker), str(startup),
                    str(source), str(ROOT / 'libraries/AP_Module/Image.cpp'), str(ROOT / 'libraries/AP_Math/crc.cpp'),
                    '-o', str(elf)], check=True, capture_output=True)
    commands = (f'mach create\nmachine LoadPlatformDescription @{platform}\nsysbus LoadELF @{elf}\n'
                f'sysbus LoadBinary @{module} 0x08010000\nemulation RunFor "0.1"\n')
    for offset in range(0, 28, 4):
        commands += f'sysbus ReadDoubleWord 0x{ram + 0x100 + offset:x}\n'
    output = renode(commands)
    words = re.findall(r'^0x([0-9a-fA-F]{8})\s*$', output, re.M)
    assert [int(word, 16) for word in words] == [0, 0, 4, 120000, 0x600DF00D, 2, 2], output
