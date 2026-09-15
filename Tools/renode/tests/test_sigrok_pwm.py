# AP_FLAKE8_CLEAN

"""PWM waveform reconstruction for the sigrok logic analyser."""

import os
import re
import socket
import struct
import subprocess
import sys
import time

from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import gen_board  # noqa: E402

from process_utils import terminate_process_group  # noqa: E402

PLATFORM = '''
nvic: IRQControllers.NVIC @ sysbus 0xE000E000
    IRQ -> cpu@0
cpu: CPU.CortexM @ sysbus
    cpuType: "cortex-m7"
    nvic: nvic
flash: Memory.MappedMemory @ sysbus 0x08000000
    size: 0x1000
timer1: Timers.STM32_Timer @ sysbus 0x40010000
    initialLimit: 0xffff
    frequency: 240000000
sigrok: Miscellaneous.AP_Sigrok @ sysbus 0x60000110
timer1Waveform: Miscellaneous.AP_STM32_Timer_Waveform @ sysbus 0x60000300
    timer: timer1
    analyzer: sigrok
    channel1: 5
    channel2: 6
    advanced: true
    name: "TIM1"
'''

# The register sequence ChibiOS pwm_lld_start() and pwmEnableChannel() use:
# 1 MHz tick, 20 ms frame, PWM mode 1 with preload on channels 1 and 2,
# 1500 us and 1850 us pulses, one update event, MOE, then ARPE|CEN.
PROGRAM = '''
sysbus WriteDoubleWord 0x40010028 239
sysbus WriteDoubleWord 0x4001002C 19999
sysbus WriteDoubleWord 0x40010018 0x6868
sysbus WriteDoubleWord 0x40010020 0x11
sysbus WriteDoubleWord 0x40010034 1500
sysbus WriteDoubleWord 0x40010038 1850
sysbus WriteDoubleWord 0x40010014 1
sysbus WriteDoubleWord 0x40010044 0x8000
sysbus WriteDoubleWord 0x40010000 0x81
'''

# A PWM-capable pin as a generated board now wires it: routed both to the
# timer's waveform source and to the GPIO fan-out, on the same channel.
SHARED_PIN_PLATFORM = PLATFORM + '''
gpioPortA: GPIOPort.STM32_GPIOPort @ sysbus <0x58020000, +0x400>
    numberOfAFs: 16
    0 -> sigrok@5
'''

INCLUDES = '''
$repo=@%s
include $repo/Tools/renode/peripherals/common/AP_SigrokInterface.cs
include $repo/Tools/renode/peripherals/common/AP_Sigrok.cs
include $repo/Tools/renode/peripherals/stm32/AP_STM32_Timer_Waveform.cs
'''


def renode_executable():
    executable = Path(os.environ.get('RENODE', ROOT / 'build/renode/renode'))
    if not executable.is_file():
        pytest.skip('build Renode or set RENODE')
    return executable.resolve()


def free_port():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


def wait_for_port(port, process, timeout=60):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise AssertionError('Renode exited early')
        try:
            socket.create_connection(('127.0.0.1', port), timeout=1).close()
            return
        except OSError:
            time.sleep(0.2)
    raise AssertionError('port %u never opened' % port)


def receive_exact(sock, length):
    data = b''
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        assert chunk, 'renode-la connection closed'
        data += chunk
    return data


def capture(port, rate, channels, seconds):
    '''Capture the given channels through the renode-la protocol and return
    (channel names, samples, unit size).'''
    sock = socket.create_connection(('127.0.0.1', port), timeout=60)
    greeting = receive_exact(sock, 24)
    assert greeting[:8] == b'RenodeLA'
    channel_count, metadata_length = struct.unpack_from('<HI', greeting, 10)
    metadata = receive_exact(sock, metadata_length)
    names = []
    offset = 0
    for _ in range(channel_count + 1):
        length = struct.unpack_from('<H', metadata, offset)[0]
        offset += 2
        names.append(metadata[offset:offset + length].decode())
        offset += length
    names = names[1:]
    mask = bytearray((channel_count + 7) // 8)
    for channel in channels:
        mask[channel // 8] |= 1 << (channel % 8)
    payload = struct.pack('<Q', rate) + bytes(mask)
    sock.sendall(bytes([1, 0, 0, 0]) + struct.pack('<I', len(payload)) + payload)
    unit = (len(channels) + 7) // 8
    wanted = int(rate * seconds) * unit
    samples = bytearray()
    while len(samples) < wanted:
        header = receive_exact(sock, 8)
        length = struct.unpack_from('<I', header, 4)[0]
        payload = receive_exact(sock, length)
        if header[0] == 0x81:
            raise AssertionError('renode-la error: %s' % payload.decode())
        if header[0] == 0x80:
            samples += payload
    sock.sendall(bytes([2, 0, 0, 0]) + struct.pack('<I', 0))
    sock.close()
    return names, samples, unit


def run_lengths(samples, unit, bit):
    '''Return (level, length) runs for one packed channel bit.'''
    result = []
    current = None
    count = 0
    for index in range(0, len(samples), unit):
        level = (samples[index + bit // 8] >> (bit % 8)) & 1
        if level == current:
            count += 1
            continue
        if current is not None:
            result.append((current, count))
        current, count = level, 1
    result.append((current, count))
    return result


def steady_runs(samples, unit, bit):
    '''Distinct high and low run lengths, excluding the partial ends.'''
    runs = run_lengths(samples, unit, bit)[1:-1]
    return (sorted({n for v, n in runs if v}),
            sorted({n for v, n in runs if not v}))


@pytest.fixture
def live_renode(tmp_path):
    '''Run a monitor script that leaves the machine started; yield helpers.'''
    executable = renode_executable()
    processes = []

    def start(script):
        path = tmp_path / 'live.resc'
        path.write_text(script)
        process = subprocess.Popen(
            [str(executable), '--disable-xwt', '-P', str(free_port()),
             '-e', 'include @%s' % path],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=dict(os.environ, TMPDIR=str(tmp_path),
                     XDG_CONFIG_HOME=str(tmp_path / 'xdg')),
            start_new_session=True)
        processes.append(process)
        return process

    yield start
    for process in processes:
        terminate_process_group(process, graceful_timeout=0.5)


def test_waveform_registers_follow_chibios_preload(tmp_path):
    '''The register mirror answers period and pulse queries and honours
    OCxPE preload: a zero written to CCR2 waits for the update event.'''
    executable = renode_executable()
    platform = tmp_path / 'pwm.repl'
    platform.write_text(PLATFORM)
    script = INCLUDES % ROOT + '''
mach create
machine LoadPlatformDescription @%s
sysbus.sigrok ConfigureSignals "TX|RX|SCK|MOSI|MISO|PWM5|PWM6"
%s
sysbus.timer1Waveform PeriodUs
sysbus.timer1Waveform PulseWidthUs 1
sysbus.timer1Waveform PulseWidthUs 2
sysbus WriteDoubleWord 0x40010038 0
sysbus.timer1Waveform PulseWidthUs 2
sysbus WriteDoubleWord 0x40010014 1
sysbus.timer1Waveform PulseWidthUs 2
sysbus WriteDoubleWord 0x40010044 0
sysbus.timer1Waveform PulseWidthUs 1
''' % (platform, PROGRAM)
    path = tmp_path / 'test.resc'
    path.write_text(script)
    output = subprocess.run(
        [str(executable), '--disable-xwt', '--console',
         '-e', 'include @%s' % path, '-e', 'quit'],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        env=dict(os.environ, TMPDIR=str(tmp_path),
                 XDG_CONFIG_HOME=str(tmp_path / 'xdg')),
        timeout=120, check=True).stdout.decode(errors='replace')
    output = re.sub(r'\x1b\[[0-9;]*m', '', output)
    assert 'There was an error' not in output, output
    values = [line.strip() for line in output.splitlines()
              if re.fullmatch(r'\s*\d+(\.\d+)?\s*', line)]
    assert values == ['20000', '1500', '1850', '1850', '0', '0'], output


def test_complementary_output_follows_ccxne(tmp_path):
    '''A pin declared TIMx_CHyN is driven from CCxNE, which ChibiOS sets
    while leaving CCxE clear.  Reading only CCxE holds it low for ever.'''
    executable = renode_executable()
    complementary = PLATFORM.replace(
        '    channel2: 6\n',
        '    channel2: 6\n    complementary1: true\n'
        '    complementary2: true\n')
    # CC1NE is bit 2 and CC2NE bit 6, so 0x44 is what ChibiOS leaves in
    # CCER for two complementary channels; 0x11 is the plain CCxE case.
    program = PROGRAM.replace('0x40010020 0x11', '0x40010020 0x44')

    def widths(platform_text, program_text):
        platform = tmp_path / ('pwm-%u.repl' % abs(hash(platform_text)))
        platform.write_text(platform_text)
        script = INCLUDES % ROOT + '''
mach create
machine LoadPlatformDescription @%s
sysbus.sigrok ConfigureSignals "TX|RX|SCK|MOSI|MISO|PWM5|PWM6"
%s
sysbus.timer1Waveform PeriodUs
sysbus.timer1Waveform PulseWidthUs 1
sysbus.timer1Waveform PulseWidthUs 2
''' % (platform, program_text)
        path = tmp_path / ('cmpl-%u.resc' % abs(hash(platform_text)))
        path.write_text(script)
        output = subprocess.run(
            [str(executable), '--disable-xwt', '--console',
             '-e', 'include @%s' % path, '-e', 'quit'],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=dict(os.environ, TMPDIR=str(tmp_path),
                     XDG_CONFIG_HOME=str(tmp_path / 'xdg')),
            timeout=120, check=True).stdout.decode(errors='replace')
        output = re.sub(r'\x1b\[[0-9;]*m', '', output)
        assert 'There was an error' not in output, output
        return [line.strip() for line in output.splitlines()
                if re.fullmatch(r'\s*\d+(\.\d+)?\s*', line)], output

    declared, output = widths(complementary, program)
    assert declared == ['20000', '1500', '1850'], output
    # without the declaration the same CCER leaves the pin flat, which is
    # what every TIMx_CHyN board saw before
    undeclared, output = widths(PLATFORM, program)
    assert undeclared == ['20000', '0', '0'], output


def test_waveform_edges_reach_the_capture(tmp_path, live_renode):
    '''A running timer produces the programmed frames in a renode-la
    capture, and a mid-frame MOE clear truncates the pulse.'''
    platform = tmp_path / 'pwm.repl'
    platform.write_text(PLATFORM)
    sigrok_port = free_port()
    process = live_renode(INCLUDES % ROOT + '''
mach create
machine LoadPlatformDescription @%s
sysbus WriteDoubleWord 0x08000000 0x20001000
sysbus WriteDoubleWord 0x08000004 0x08000009
sysbus WriteDoubleWord 0x08000008 0xe7fee7fe
cpu VectorTableOffset 0x08000000
cpu PerformanceInMips 50
emulation SetGlobalQuantum "0.001"
sysbus.sigrok ConfigureSignals "TX|RX|SCK|MOSI|MISO|PWM5|PWM6"
%s
sysbus.sigrok Port %u
start
''' % (platform, PROGRAM, sigrok_port))
    wait_for_port(sigrok_port, process)
    names, samples, unit = capture(sigrok_port, 1000000, [5, 6], 0.1)
    assert names[5:] == ['PWM5', 'PWM6']
    # The stock timer overflows after ARR ticks, so the low half of each
    # frame is one tick short; timestamps also round onto the 1 us grid.
    for bit, (high, low) in ((0, (1500, 18499)), (1, (1850, 18149))):
        highs, lows = steady_runs(samples, unit, bit)
        assert highs and all(abs(n - high) <= 1 for n in highs), highs
        assert lows and all(abs(n - low) <= 1 for n in lows), lows


def test_gpio_on_a_pwm_capable_pin_is_captured(tmp_path, live_renode):
    '''Firmware can drive a PWM-capable pin as a GPIO at runtime, through
    SERVO_GPIO_MASK or SERVOx_FUNCTION, and AP_Relay does.  With the timer
    output disabled the waveform source must release the channel, so the
    pin's real level reaches the capture instead of a flat low.'''
    platform = tmp_path / 'shared.repl'
    platform.write_text(SHARED_PIN_PLATFORM)
    sigrok_port = free_port()
    process = live_renode(INCLUDES % ROOT + '''
mach create
machine LoadPlatformDescription @%s
sysbus WriteDoubleWord 0x08000000 0x20001000
sysbus WriteDoubleWord 0x08000004 0x08000009
sysbus WriteDoubleWord 0x08000008 0xe7fee7fe
cpu VectorTableOffset 0x08000000
cpu PerformanceInMips 50
emulation SetGlobalQuantum "0.001"
sysbus.sigrok ConfigureSignals "TX|RX|SCK|MOSI|MISO|PWM5|PWM6"
sysbus WriteDoubleWord 0x58020000 0x1
sysbus WriteDoubleWord 0x58020018 0x1
sysbus.sigrok Port %u
start
''' % (platform, sigrok_port))
    wait_for_port(sigrok_port, process)
    _, samples, unit = capture(sigrok_port, 1000000, [5], 0.002)
    levels = {(samples[i] >> 0) & 1 for i in range(0, len(samples), unit)}
    # timer1 channel 1 shares this pin but its output is not enabled, so
    # the GPIO owns it and the capture must show the pin driven high
    assert levels == {1}, levels


def test_generated_board_reconstructs_pwm_pins(tmp_path):
    '''Every PWM timer with sigrok-visible pins gets a waveform peripheral
    wired to those channels, and those pins leave the GPIO fan-out so the
    reconstruction is their only edge source.'''
    generated = gen_board.generate(
        ROOT, 'YJUAV_A6SE_H743', tmp_path / 'generated', sigrok=True,
        state_dir=tmp_path / 'state')
    repl = (tmp_path / 'generated' / 'YJUAV_A6SE_H743.repl').read_text()
    signals = generated['sigrok_signals']
    pwm1 = signals.index('PA15/TIM2_CH1/GPIO50/PWM1')
    pwm8 = signals.index('PE14/TIM1_CH4/GPIO57/PWM8')
    timer2 = re.search(r'timer2Waveform:.*?\n\n', repl, re.S).group(0)
    timer1 = re.search(r'timer1Waveform:.*?\n\n', repl, re.S).group(0)
    assert '    channel1: %u\n' % pwm1 in timer2
    assert '    channel4: %u\n' % pwm8 in timer1
    assert '    advanced: true' in timer1
    assert '    advanced: true' not in timer2
    # A PWM-capable pin keeps its GPIO route as well as its waveform
    # source, because firmware can drive it as a GPIO at runtime through
    # SERVO_GPIO_MASK or SERVOx_FUNCTION.  The analyser arbitrates between
    # the two; dropping the route here lost those transitions entirely.
    port_a = re.search(r'gpioPortA:\n(.*?)\n\n', repl, re.S).group(1)
    assert 'sigrok@%u' % pwm1 in port_a
    assert 'sigrok@' in repl  # chip selects are still routed
    assert generated['warnings'] == []
