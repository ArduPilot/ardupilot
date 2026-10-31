# AP_FLAKE8_CLEAN

"""Exercise reset, RAM clocks and flash behavior without vehicle firmware."""

import os
import re
import subprocess
import sys

from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import gen_board  # noqa: E402

from launch import clean_monitor_text  # noqa: E402
from process_utils import terminate_process_group  # noqa: E402


@pytest.fixture
def renode(tmp_path):
    executable = Path(os.environ.get('RENODE', ROOT / 'build/renode/renode'))
    if not executable.is_file():
        pytest.skip('build Renode or set RENODE')

    def run(script):
        path = tmp_path / 'test.resc'
        path.write_text(script)
        process = subprocess.Popen(
            [str(executable.resolve()), '--disable-xwt', '--console',
             '-e', 'include @%s' % path, '-e', 'quit'],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=dict(os.environ, TMPDIR=str(tmp_path),
                     XDG_CONFIG_HOME=str(tmp_path / 'xdg')),
            start_new_session=True)
        try:
            stdout, _ = process.communicate(timeout=60)
        except subprocess.TimeoutExpired:
            terminate_process_group(process, graceful_timeout=0.2)
            stdout, _ = process.communicate()
            pytest.fail(clean_monitor_text(stdout))
        output = clean_monitor_text(stdout)
        assert process.returncode == 0, output
        assert 'There was an error' not in output, output
        return output

    return run


def test_generated_gpio_defaults_survive_reset(tmp_path, renode):
    generated = gen_board.generate(ROOT, 'CubeOrangePlus', tmp_path / 'generated')
    script = generated['resc'].read_text()
    reset = re.search(r'macro reset\n""".*?"""\nrunMacro \$reset', script, re.S)
    assert reset is not None
    platform = tmp_path / 'gpio.repl'
    platform.write_text('''
flash: Memory.MappedMemory @ sysbus 0x08000000
    size: 0x200000
nvic: IRQControllers.NVIC @ sysbus 0xE000E000
    IRQ -> cpu@0
cpu: CPU.CortexM @ sysbus
    cpuType: "cortex-m7"
    nvic: nvic
gpioPortA: GPIOPort.STM32_GPIOPort @ sysbus <0x58020000, +0x400>
gpioPortB: GPIOPort.STM32_GPIOPort @ sysbus <0x58020400, +0x400>
gpioPortC: GPIOPort.STM32_GPIOPort @ sysbus <0x58020800, +0x400>
gpioPortE: GPIOPort.STM32_GPIOPort @ sysbus <0x58021000, +0x400>
''')
    output = renode('''
mach create
machine LoadPlatformDescription @%s
$vector_base=0x08020000
%s
sysbus ReadDoubleWord 0x58020410
machine Reset
sysbus ReadDoubleWord 0x58020410
cpu VectorTableOffset
''' % (platform, reset.group()))
    assert output.count('0x000000A0') == 2, output
    assert '0x08020000' in output, output


@pytest.mark.parametrize(('platform_name', 'address', 'page_size'), [
    ('stm32f103_base.repl', 0x08000800, 1024),
    ('stm32f105_base.repl', 0x08020800, 2048),
])
def test_f1_flash_page_geometry(renode, platform_name, address, page_size):
    includes = (HERE / 'scripts/ardupilot_f103.resc').read_text().split('mach create')[0]
    output = renode('''
$repo=@%s
include $repo/Tools/renode/peripherals/common/AP_SigrokInterface.cs
%s
mach create
machine LoadPlatformDescription @%s
sysbus WriteDoubleWord 0x%X 0x12345678
sysbus WriteDoubleWord 0x%X 0x12345678
sysbus WriteDoubleWord 0x%X 0x12345678
sysbus WriteDoubleWord 0x40022004 0x45670123
sysbus WriteDoubleWord 0x40022004 0xCDEF89AB
sysbus WriteDoubleWord 0x40022014 0x%X
sysbus WriteDoubleWord 0x40022010 0x42
sysbus ReadDoubleWord 0x%X
sysbus ReadDoubleWord 0x%X
sysbus ReadDoubleWord 0x%X
''' % (ROOT, includes, HERE / 'platforms' / platform_name,
       address, address + page_size - 4, address + page_size,
       address, address, address + page_size - 4, address + page_size))
    assert output.count('0xFFFFFFFF') == 2, output
    assert '0x12345678' in output, output


@pytest.mark.parametrize('blocked_write', [False, True])
def test_persistent_flash_replaces_existing_image(tmp_path, renode, blocked_write):
    image = tmp_path / 'flash.img'
    image.write_bytes(b'old image')
    temporary = image.with_name(image.name + '.tmp')
    if blocked_write:
        temporary.mkdir()
    platform = tmp_path / 'flash.repl'
    platform.write_text('''
flash: Memory.MappedMemory @ sysbus 0x08000000
    size: 0x1000
persistence: Miscellaneous.AP_PersistentMemory @ none
    fileName: "%s"
    address: 0x08000000
    size: 0x1000
''' % image)
    output = renode('''
include @%s
mach create
machine LoadPlatformDescription @%s
sysbus WriteDoubleWord 0x08000000 0x12345678
start
pause
''' % (HERE / 'peripherals/common/AP_PersistentMemory.cs', platform))
    if blocked_write:
        assert 'Failed to persist' in output
        assert image.read_bytes() == b'old image'
    else:
        assert image.read_bytes() == b'\x78\x56\x34\x12' + bytes(4092)
        assert not temporary.exists()


@pytest.fixture
def h7_sram_platform(tmp_path, request):
    cpu1_registers, cpu2_in_stop = getattr(request, 'param', (False, True))
    platform = tmp_path / 'h7-sram.repl'
    platform.write_text('''
cpu: CPU.CortexM @ sysbus
    cpuType: "cortex-m7"
    nvic: nvic
nvic: IRQControllers.NVIC @ sysbus 0xE000E000
    IRQ -> cpu@0
flash: Memory.MappedMemory @ sysbus 0x08000000
    size: 0x1000
axiSram: Memory.MappedMemory @ sysbus 0x24000000
    size: 0x1000
sram1: Memory.MappedMemory @ sysbus 0x30000000
    size: 0x20000
sram2: Memory.MappedMemory @ sysbus 0x30020000
    size: 0x20000
sram3: Memory.MappedMemory @ sysbus 0x30040000
    size: 0x8000
rcc: Miscellaneous.AP_STM32H7_RCC @ sysbus 0x58024400
    hasCpu1Registers: %s
    cpu2InStop: %s
    nvic: nvic
    sram1: sram1
    sram2: sram2
    sram3: sram3
''' % (str(cpu1_registers).lower(), str(cpu2_in_stop).lower()))
    return '''
include @%s
mach create
logLevel 3
machine LoadPlatformDescription @%s
''' % (HERE / 'peripherals/stm32/AP_STM32H7_RCC.cs', platform)


def test_h7_sram_shares_domain_clock_and_preserves_contents(renode, h7_sram_platform):
    script = [h7_sram_platform]
    expected = []
    banks = [(0x30000000, 0x20000), (0x30020000, 0x20000), (0x30040000, 0x8000)]

    def read(address, value):
        script.append('sysbus ReadDoubleWord 0x%X' % address)
        expected.append(value)

    read(0x580244DC, 0)
    for bank, (address, size) in enumerate(banks):
        # Both ends of each bank must be inaccessible at reset.
        for offset in (0, size - 4):
            script.append('sysbus WriteDoubleWord 0x%X 0xDEADBEEF' % (address + offset))
            read(address + offset, 0)
        script.append('sysbus WriteDoubleWord 0x580244DC 0x%X' % (1 << (29 + bank)))
        # Allocating any SRAM keeps D2 running, making all three banks usable.
        for other_address, _ in banks:
            script.append('sysbus WriteDoubleWord 0x%X 0x12345678' % other_address)
            read(other_address, 0x12345678)
        script.append('sysbus WriteDoubleWord 0x%X 0x87654321' % (address + size - 4))
        script.append('sysbus WriteDoubleWord 0x580244DC 0')
        script.append('sysbus WriteDoubleWord 0x%X 0xDEADBEEF' % address)
        read(address, 0)
        script.append('sysbus WriteDoubleWord 0x580244DC 0x%X' % (1 << (29 + bank)))
        read(address, 0x12345678)
        read(address + size - 4, 0x87654321)
        script.append('sysbus WriteDoubleWord 0x580244DC 0')

    script += ['sysbus WriteDoubleWord 0x580244DC 0xE0000000', 'machine Reset']
    read(0x580244DC, 0)
    for address, _ in banks:
        read(address, 0)
    script.append('sysbus WriteDoubleWord 0x24000000 0xABCDEF12')
    read(0x24000000, 0xABCDEF12)
    output = renode('\n'.join(script))
    values = [int(value, 16) for value in re.findall(r'^0x([0-9A-Fa-f]{8})\s*$', output, re.M)]
    assert values == expected, output


@pytest.mark.parametrize(('h7_sram_platform', 'alias'), [((False, True), 0), ((True, True), 0x60)],
                         indirect=['h7_sram_platform'])
def test_h7_d2_peripheral_allocations_keep_all_sram_accessible(renode, h7_sram_platform, alias):
    script = [h7_sram_platform]
    expected = []

    def read(address, value):
        script.append('sysbus ReadDoubleWord 0x%X' % address)
        expected.append(value)

    # DMA1, RNG, TIM5, FDCAN and USART1 are on the five D2 enable registers.
    for register, mask in ((0xD8, 1), (0xDC, 0x40), (0xE8, 8), (0xEC, 0x100), (0xF0, 0x10)):
        script.append('sysbus WriteDoubleWord 0x%X 0x%X' % (0x58024400 + register + alias, mask))
        read(0x58024400 + register, mask)
        # All SRAM allocations remain clear; the peripheral keeps D2 running.
        read(0x580244DC, mask if register == 0xDC else 0)
        for address in (0x30000000, 0x30020000, 0x30040000):
            script.append('sysbus WriteDoubleWord 0x%X 0x12345678' % address)
            read(address, 0x12345678)
        script.append('sysbus WriteDoubleWord 0x%X 0' % (0x58024400 + register))
        read(0x58024400 + register + alias, 0)
        read(0x30000000, 0)

    # A D3 GPIO allocation must not keep D2 alive.
    script.append('sysbus WriteDoubleWord 0x580244E0 1')
    read(0x30000000, 0)
    # Clearing one allocation must leave the other effective, until reset.
    script += ['sysbus WriteDoubleWord 0x580244D8 1',
               'sysbus WriteDoubleWord 0x580244DC 0x40',
               'sysbus WriteDoubleWord 0x580244D8 0']
    read(0x30000000, 0x12345678)
    script.append('machine Reset')
    read(0x30000000, 0)
    output = renode('\n'.join(script))
    values = [int(value, 16) for value in re.findall(r'^0x([0-9A-Fa-f]{8})\s*$', output, re.M)]
    assert values == expected, output


@pytest.mark.parametrize('h7_sram_platform', [(True, False)], indirect=True)
def test_h757_cpu2_hold_keeps_d2_sram_accessible(renode, h7_sram_platform):
    script = [h7_sram_platform]
    for address in (0x30000000, 0x30020000, 0x30040000):
        script += ['sysbus WriteDoubleWord 0x%X 0x12345678' % address,
                   'sysbus ReadDoubleWord 0x%X' % address]
    script += ['sysbus WriteDoubleWord 0x5802453C 0xE0000000',
               'sysbus WriteDoubleWord 0x5802453C 0',
               'sysbus ReadDoubleWord 0x30000000',
               'machine Reset',
               'sysbus WriteDoubleWord 0x30000000 0x87654321',
               'sysbus ReadDoubleWord 0x30000000',
               'sysbus ReadDoubleWord 0x580244DC']
    output = renode('\n'.join(script))
    values = [int(value, 16) for value in re.findall(r'^0x([0-9A-Fa-f]{8})\s*$', output, re.M)]
    assert values == [0x12345678] * 4 + [0x87654321, 0], output


def test_h7_sram_clock_gates_cpu_stack_and_revokes_mapping(renode, h7_sram_platform):
    output = renode(h7_sram_platform + '''
# Vector table and Thumb instructions: push {r0}; pop {r1}; b .
sysbus WriteDoubleWord 0x08000000 0x30001000
sysbus WriteDoubleWord 0x08000004 0x08000009
sysbus WriteWord 0x08000008 0xB401
sysbus WriteWord 0x0800000A 0xBC02
sysbus WriteWord 0x0800000C 0xE7FE
cpu VectorTableOffset 0x08000000
cpu SetRegister 0 0x12345678
cpu Step 2
cpu GetRegister 1
sysbus WriteDoubleWord 0x580244DC 0x20000000
cpu PC 0x08000008
cpu Step 2
cpu GetRegister 1
sysbus WriteDoubleWord 0x580244DC 0
cpu PC 0x08000008
cpu SetRegister 0 0xDEADBEEF
cpu Step 2
cpu GetRegister 1
sysbus WriteDoubleWord 0x580244DC 0x20000000
sysbus ReadDoubleWord 0x30000FFC
pause
machine Reset
cpu VectorTableOffset 0x08000000
cpu SetRegister 0 0xDEADBEEF
cpu Step 2
cpu GetRegister 1
''')
    # Step prints a 64-bit PC; register/bus reads below are at most 32 bits.
    values = [int(value, 16) for value in re.findall(r'^0x([0-9A-Fa-f]{1,8})\s*$', output, re.M)]
    assert 'CPU was halted' not in output, output
    assert values == [0, 0x12345678, 0, 0x12345678, 0], output


def test_h7_cpu_enables_sram_clock_before_stack_access(renode, h7_sram_platform):
    output = renode(h7_sram_platform + '''
# str r3, [r2]; push {r0}; pop {r1}; b .
sysbus WriteDoubleWord 0x08000000 0x30001000
sysbus WriteDoubleWord 0x08000004 0x08000009
sysbus WriteWord 0x08000008 0x6013
sysbus WriteWord 0x0800000A 0xB401
sysbus WriteWord 0x0800000C 0xBC02
sysbus WriteWord 0x0800000E 0xE7FE
cpu VectorTableOffset 0x08000000
cpu SetRegister 0 0x12345678
cpu SetRegister 2 0x580244DC
cpu SetRegister 3 0x20000000
cpu Step 3
cpu GetRegister 1
sysbus ReadDoubleWord 0x580244DC
''')
    values = [int(value, 16) for value in re.findall(r'^0x([0-9A-Fa-f]{1,8})\s*$', output, re.M)]
    assert 'CPU was halted' not in output, output
    assert values == [0x12345678, 0x20000000], output


@pytest.mark.parametrize(('h7_sram_platform', 'cpu1_registers'), [((False, True), False), ((True, True), True)],
                         indirect=['h7_sram_platform'])
def test_h7_cpu1_sram_clock_register_alias(renode, h7_sram_platform, cpu1_registers):
    output = renode(h7_sram_platform + '''
sysbus WriteDoubleWord 0x5802453C 0x20000000
sysbus ReadDoubleWord 0x580244DC
sysbus WriteDoubleWord 0x30000000 0x12345678
sysbus ReadDoubleWord 0x30000000
sysbus WriteDoubleWord 0x580244DC 0
sysbus ReadDoubleWord 0x5802453C
sysbus ReadDoubleWord 0x30000000
sysbus WriteDoubleWord 0x580244DC 0x40000000
sysbus ReadDoubleWord 0x5802453C
machine Reset
sysbus ReadDoubleWord 0x580244DC
sysbus ReadDoubleWord 0x5802453C
''')
    values = [int(value, 16) for value in re.findall(r'^0x([0-9A-Fa-f]{8})\s*$', output, re.M)]
    if cpu1_registers:
        assert values == [0x20000000, 0x12345678, 0, 0, 0x40000000, 0, 0], output
    else:
        assert values == [0, 0, 0x20000000, 0, 0x20000000, 0, 0], output
