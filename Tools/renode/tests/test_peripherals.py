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
