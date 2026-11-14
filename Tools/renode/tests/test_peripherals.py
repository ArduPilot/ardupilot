# AP_FLAKE8_CLEAN

"""Exercise reset and flash behavior in Renode without vehicle firmware."""

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


def family_platform(script_name, platform_name):
    """Monitor commands creating a machine from a family base platform."""
    includes = (HERE / 'scripts' / script_name).read_text().split('mach create')[0]
    return '''
$repo=@%s
include $repo/Tools/renode/peripherals/common/AP_SigrokInterface.cs
%s
mach create
machine LoadPlatformDescription @%s
''' % (ROOT, includes, HERE / 'platforms' / platform_name)


def register_values(output):
    return re.findall(r'^0x[0-9A-F]{8}$', output, re.M)


def test_f1_pvd_trips_with_hysteresis_and_latches_exti(renode):
    # PVDE with PLS 7 (2.78V falling, 2.88V rising); EXTI line 16 rising
    # edge, unmasked; read PWR_CSR then EXTI_PR after each supply change
    output = renode(family_platform('ardupilot_f103.resc', 'stm32f103_base.repl') + '''
sysbus WriteDoubleWord 0x40007000 0xF0
sysbus WriteDoubleWord 0x40010408 0x10000
sysbus WriteDoubleWord 0x40010400 0x10000
pwr Vdd 2.82
sysbus ReadDoubleWord 0x40007004
sysbus ReadDoubleWord 0x40010414
pwr Vdd 2.7
sysbus ReadDoubleWord 0x40007004
sysbus ReadDoubleWord 0x40010414
pwr Vdd 2.82
sysbus ReadDoubleWord 0x40007004
pwr Vdd 3.3
sysbus ReadDoubleWord 0x40007004
sysbus ReadDoubleWord 0x40010414
sysbus WriteDoubleWord 0x40010414 0x10000
sysbus ReadDoubleWord 0x40010414
''')
    assert register_values(output) == [
        '0x00000000', '0x00000000',  # above the falling threshold
        '0x00000004', '0x00010000',  # tripped and latched
        '0x00000004',                # hysteresis holds below 2.88V
        '0x00000000', '0x00010000',  # recovered, still latched
        '0x00000000',                # cleared by writing 1
    ], output


def test_f1_pvd_dip_is_timed_and_latched(renode):
    output = renode(family_platform('ardupilot_f103.resc', 'stm32f103_base.repl') + '''
cpu IsHalted true
sysbus WriteDoubleWord 0x40007000 0xF0
sysbus WriteDoubleWord 0x40010408 0x10000
sysbus WriteDoubleWord 0x40010400 0x10000
pwr Dip 2.5 0.2
sysbus ReadDoubleWord 0x40007004
emulation RunFor "0.001"
sysbus ReadDoubleWord 0x40007004
sysbus ReadDoubleWord 0x40010414
pwr PvdTrips
''')
    assert register_values(output) == [
        '0x00000004', '0x00000000', '0x00010000'], output
    assert '0x0000000000000001' in output, output


def test_f1_reset_flags_record_the_cause(renode):
    # RCC_CSR after power-on, then after a software reset, an IWDG
    # timeout (PR=3, RLR=99: 80ms at 40kHz) and a power cycle, clearing
    # the flags with RMVF before each
    output = renode(family_platform('ardupilot_f103.resc', 'stm32f103_base.repl') + '''
cpu IsHalted true
sysbus ReadDoubleWord 0x40021024
sysbus WriteDoubleWord 0x40021024 0x01000000
machine Reset
sysbus ReadDoubleWord 0x40021024
sysbus WriteDoubleWord 0x40021024 0x01000000
sysbus WriteDoubleWord 0x40003000 0x5555
sysbus WriteDoubleWord 0x40003004 3
sysbus WriteDoubleWord 0x40003008 99
sysbus WriteDoubleWord 0x40003000 0xCCCC
iwdg EnforceReset true
emulation RunFor "0.07"
sysbus ReadDoubleWord 0x40021024
emulation RunFor "0.02"
sysbus ReadDoubleWord 0x40021024
rcc PowerCycle
emulation RunFor "0.001"
sysbus ReadDoubleWord 0x40021024
''')
    assert register_values(output) == [
        '0x0C000000',  # PORRSTF | PINRSTF
        '0x14000000',  # SFTRSTF | PINRSTF
        '0x00000000',  # watchdog not yet expired at 70ms
        '0x24000000',  # IWDGRSTF | PINRSTF
        '0x0C000000',
    ], output


def test_h7_pvd_trips_and_latches_exti(renode):
    # PVDEN with PLS 6 (2.76V falling); EXTI RTSR1 and CPU1 IMR1 line 16;
    # CSR1 always reads ACTVOSRDY as well
    output = renode(family_platform('ardupilot_h743.resc', 'stm32h743_base.repl') + '''
sysbus WriteDoubleWord 0x58024800 0xD0
sysbus WriteDoubleWord 0x58000000 0x10000
sysbus WriteDoubleWord 0x58000080 0x10000
sysbus ReadDoubleWord 0x58024804
pwr Vdd 2.7
sysbus ReadDoubleWord 0x58024804
pwr Vdd 3.3
sysbus ReadDoubleWord 0x58024804
sysbus ReadDoubleWord 0x58000088
''')
    assert register_values(output) == [
        '0x00002000', '0x00002010', '0x00002000', '0x00010000'], output
