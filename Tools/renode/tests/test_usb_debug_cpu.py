# AP_FLAKE8_CLEAN
"""Guest DebugMonitor tests; requires patches/cortex-m-debug-monitor.patch."""

import shutil
import subprocess

from pathlib import Path

import pytest

from test_peripherals import renode  # noqa: F401

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def image(tmp_path, renode):  # noqa: F811
    compiler = shutil.which('arm-none-eabi-gcc')
    if compiler is None:
        pytest.skip('arm-none-eabi-gcc is required')
    platform = tmp_path / 'cpu.repl'
    platform.write_text('''
cpu: CPU.CortexM @ sysbus
    cpuType: "cortex-m7"
    nvic: nvic
nvic: IRQControllers.NVIC @ sysbus 0xE000E000
    IRQ -> cpu@0
flash: Memory.MappedMemory @ sysbus 0x08000000
    size: 0x10000
ram: Memory.MappedMemory @ sysbus 0x20000000
    size: 0x10000
''')
    # The packaged Renode used by CI does not yet include the guest-debug patch.
    # Check its API, not its behavior: broken implementations must still fail.
    members = renode(f'mach create\nmachine LoadPlatformDescription @{platform}\ncpu\n')
    if not all(name in members for name in ('SetDebugMonitorControl', 'SetFPBControl', 'SetFPBComparator')):
        pytest.skip('Renode requires Tools/renode/patches/cortex-m-debug-monitor.patch')
    linker = tmp_path / 'cpu.ld'
    linker.write_text('ENTRY(reset)\nSECTIONS { . = 0x08000000; .vectors : { *(.vectors) } '
                      '.text : { *(.text*) } }')

    def build(body):
        source = tmp_path / 'cpu.S'
        source.write_text('''
.syntax unified
.cpu cortex-m7
.thumb
.section .vectors,"a"
.word 0x20008000
.word reset
.rept 10
.word fault
.endr
.word debugmon
.word fault
.word fault
.word fault
.section .text,"ax"
.thumb_func
fault:
 ldr r0, =0x20000100
 ldr r1, =0xbad00003
 str r1, [r0]
 b .
''' + body)
        elf = tmp_path / 'cpu.elf'
        subprocess.run([compiler, '-nostdlib', '-mcpu=cortex-m7', '-mthumb',
                        '-T', str(linker), str(source), '-o', str(elf)], check=True, capture_output=True)
        return f'mach create\nmachine LoadPlatformDescription @{platform}\nsysbus LoadELF @{elf}\n'

    return build


@pytest.mark.parametrize('enabled', [False, True])
def test_bkpt_monitor_enable(image, renode, enabled):  # noqa: F811
    script = image('''
.thumb_func
reset:
 ldr r0, =0xe000edfc
 ldr r1, =%d
 str r1, [r0]
 bkpt #42
 b .
.thumb_func
debugmon:
 ldr r0, =0x20000100
 ldr r1, =0xdead0012
 str r1, [r0]
 b .
''' % (0x10000 if enabled else 0))
    output = renode(script + 'cpu Step 40\nsysbus ReadDoubleWord 0x20000100\n'
                    'sysbus ReadDoubleWord 0xe000ed30\nsysbus ReadDoubleWord 0xe000ed2c\n')
    assert ('0xDEAD0012' if enabled else '0xBAD00003') in output, output
    assert '0x00000002' in output, output  # DFSR.BKPT
    if not enabled:
        assert '0x80000000' in output, output  # HFSR.DEBUGEVT


def test_fpb_then_single_step(image, renode, tmp_path):  # noqa: F811
    script = image('''
.thumb_func
reset:
 ldr r0, =0xe000edfc
 ldr r1, =0x10000
 str r1, [r0]
 ldr r0, =0xe0002008
 ldr r1, =target+1
 str r1, [r0]
 ldr r0, =0xe0002000
 movs r1, #3
 str r1, [r0]
 movs r4, #40
 b target
target:
 adds r4, #1
after:
 ldr r0, =0x20000110
 str r4, [r0]
 b .
.thumb_func
debugmon:
 ldr r0, =0xe000ed30
 ldr r1, [r0]
 str r1, [r0]
 cmp r1, #2
 bne stepped
 ldr r0, =0xe0002008
 movs r1, #0
 str r1, [r0]
 ldr r0, =0xe000edfc
 ldr r1, =0x50000
 str r1, [r0]
 bx lr
stepped:
 ldr r0, =0x20000100
 str r1, [r0]
 ldr r1, [sp, #24]
 ldr r2, =after
 subs r1, r1, r2
 str r1, [r0, #4]
 ldr r0, =0xe000edfc
 ldr r1, =0x10000
 str r1, [r0]
 bx lr
''')
    fpb = tmp_path / 'fpb.repl'
    fpb.write_text('fpb: Miscellaneous.AP_CortexM_FPB @ sysbus 0xE0002000\n    cpu: cpu\n')
    output = renode(f'include @{ROOT}/Tools/renode/peripherals/cpu/AP_CortexM_FPB.cs\n' + script +
                    f'machine LoadPlatformDescription @{fpb}\ncpu Step 100\n'
                    'sysbus ReadDoubleWord 0x20000100\nsysbus ReadDoubleWord 0x20000104\n'
                    'sysbus ReadDoubleWord 0x20000110\n')
    assert '0x00000001' in output, output  # DFSR.HALTED
    assert '0x00000000' in output, output  # Stopped immediately after the add
    assert '0x00000029' in output, output  # Target instruction ran exactly once


def test_hardfault_preempts_masked_debugmonitor(image, renode):  # noqa: F811
    script = image('''
.thumb_func
reset:
 ldr r0, =0xe000edfc
 ldr r1, =0x30000
 str r1, [r0]
 b .
.thumb_func
debugmon:
 cpsid i
 bkpt #42
 ldr r0, =0x20000100
 ldr r1, =0xdead0012
 str r1, [r0]
 b .
''')
    output = renode(script + 'cpu Step 80\nsysbus ReadDoubleWord 0x20000100\n'
                    'sysbus ReadDoubleWord 0xe000ed2c\n')
    assert '0xBAD00003' in output, output
    assert '0x80000000' in output, output


def test_masked_mpu_fault_escalates(image, renode):  # noqa: F811
    script = image('''
.thumb_func
reset:
 ldr r0, =0xe000ed98
 movs r1, #0
 str r1, [r0]
 ldr r1, =0x20001000
 str r1, [r0, #4]
 ldr r1, =0x16000009
 str r1, [r0, #8]
 movs r1, #7
 str r1, [r0, #-4]
 dsb
 isb
 cpsid i
 ldr r0, =0x20001000
 movs r1, #99
 strb r1, [r0]
 b .
.thumb_func
debugmon:
 b .
''')
    output = renode(script + 'cpu Step 80\nsysbus ReadDoubleWord 0x20000100\n'
                    'sysbus ReadDoubleWord 0x20001000\nsysbus ReadDoubleWord 0xe000ed2c\n')
    assert '0xBAD00003' in output, output
    assert '0x00000000' in output, output
    assert '0x40000000' in output, output


@pytest.mark.parametrize('stepping', [False, True])
@pytest.mark.parametrize('function,access', [(5, 'ldr'), (6, 'str'), (7, 'ldr'), (7, 'str')])
def test_dwt_guest_watchpoint(image, renode, tmp_path, function, access, stepping):  # noqa: F811
    script = image('''
.thumb_func
reset:
 ldr r0, =0xe000edfc
 ldr r1, =0x10000
 str r1, [r0]
 ldr r0, =0xe0001020
 ldr r1, =0x20000110
 str r1, [r0]
 movs r1, #0
 str r1, [r0, #4]
 movs r1, #%d
 str r1, [r0, #8]
 ldr r0, =0x20000110
 movs r1, #42
 %s r1, [r0]
after:
 b .
.thumb_func
debugmon:
 ldr r0, =0x20000100
 ldr r1, =0xdead0012
 str r1, [r0]
 ldr r1, [sp, #24]
 ldr r2, =after
 subs r1, r1, r2
 str r1, [r0, #4]
 b .
''' % (function, access))
    members = renode(script + 'cpu\n')
    if 'RequestDWTTrap' not in members:
        pytest.skip('Renode requires the USB debug extensions CPU patch')
    dwt = tmp_path / 'dwt.repl'
    dwt.write_text('dwt: Miscellaneous.AP_DWT @ sysbus 0xE0001000\n    cpu: cpu\n')
    output = renode(f'include @{ROOT}/Tools/renode/peripherals/common/AP_DWT.cs\n' + script +
                    f'machine LoadPlatformDescription @{dwt}\n' +
                    ('cpu Step 100\n' if stepping else 'emulation RunFor \"0.001\"\n') +
                    'sysbus ReadDoubleWord 0x20000100\nsysbus ReadDoubleWord 0x20000104\n'
                    'sysbus ReadDoubleWord 0xe000ed30\nsysbus ReadDoubleWord 0xe0001028\n')
    assert '0xDEAD0012' in output, output
    assert '0x00000000' in output, output  # Stacked PC follows the access
    assert '0x00000004' in output, output  # DFSR.DWTTRAP
    assert f'0x0100000{function:X}' in output, output  # FUNCTION.MATCHED


def test_masked_dwt_waits_for_interrupt_enable(image, renode, tmp_path):  # noqa: F811
    script = image('''
.thumb_func
reset:
 ldr r0, =0xe000edfc
 ldr r1, =0x1010000
 str r1, [r0]
 ldr r0, =0xe0001020
 ldr r1, =0x20000110
 str r1, [r0]
 movs r1, #6
 str r1, [r0, #8]
 cpsid i
 ldr r0, =0x20000110
 movs r1, #42
 str r1, [r0]
 adds r1, #1
 str r1, [r0, #4]
 cpsie i
 b .
.thumb_func
debugmon:
 ldr r0, =0x20000100
 ldr r1, =0xdead0012
 str r1, [r0]
 b .
''')
    if 'RequestDWTTrap' not in renode(script + 'cpu\n'):
        pytest.skip('Renode requires the USB debug extensions CPU patch')
    dwt = tmp_path / 'dwt.repl'
    dwt.write_text('dwt: Miscellaneous.AP_DWT @ sysbus 0xE0001000\n    cpu: cpu\n')
    output = renode(f'include @{ROOT}/Tools/renode/peripherals/common/AP_DWT.cs\n' + script +
                    f'machine LoadPlatformDescription @{dwt}\nemulation RunFor "0.001"\n'
                    'sysbus ReadDoubleWord 0x20000100\nsysbus ReadDoubleWord 0x20000114\n'
                    'sysbus ReadDoubleWord 0xe000ed2c\n')
    assert '0xDEAD0012' in output, output
    assert '0x0000002B' in output, output  # The masked instruction sequence completed
    assert '0x00000000' in output, output  # No HardFault escalation


def test_dwt_capability_detection(renode, tmp_path):  # noqa: F811
    platform = tmp_path / 'dwt.repl'
    platform.write_text('''
cpu: CPU.CortexM @ sysbus
    cpuType: "cortex-m7"
    nvic: nvic
nvic: IRQControllers.NVIC @ sysbus 0xE000E000
    IRQ -> cpu@0
dwt: Miscellaneous.AP_DWT @ sysbus 0xE0001000
    cpu: cpu
''')
    output = renode(f'include @{ROOT}/Tools/renode/peripherals/common/AP_DWT.cs\n'
                    f'mach create\nmachine LoadPlatformDescription @{platform}\n'
                    'cpu\nsysbus ReadDoubleWord 0xe0001000\n')
    # Packaged Renode must still compile/load the model, advertising no hardware
    # watchpoints instead of failing on missing guest-debug CPU APIs.
    expected = '0x40000000' if 'RequestDWTTrap' in output else '0x00000000'
    assert expected in output, output
