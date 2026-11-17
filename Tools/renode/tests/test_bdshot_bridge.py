# AP_FLAKE8_CLEAN

"""Unit tests for the Renode bidirectional DShot bridge wiring and test ESCs."""

import socket
import sys
import time

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'Tools' / 'renode'))

import dshot_esc  # noqa: E402
import gen_board  # noqa: E402


def decode_erpm(frame):
    '''RCOutput::bdshot_decode_telemetry_from_erpm, returning eRPM.'''
    value = frame >> 4
    assert frame & 0xF == ~dshot_esc.checksum(value) & 0xF
    if value == 0x0FFF:
        return 0
    period = (value & 0x1FF) << (value >> 9)
    return (600000 + period // 2) // period * 100


def test_erpm_telemetry_frame_round_trip():
    assert decode_erpm(dshot_esc.erpm_telemetry_frame(0)) == 0
    for erpm in (1000, 21000, 100000, 300000):
        decoded = decode_erpm(dshot_esc.erpm_telemetry_frame(erpm))
        assert abs(decoded - erpm) <= erpm * 0.02
        # normalised eRPM, not mistaken for extended telemetry
        value = dshot_esc.erpm_telemetry_frame(erpm) >> 4
        assert value >> 9 == 0 or value & 0x100


def test_bridge_is_generated_only_when_requested(tmp_path):
    plain = gen_board.generate(ROOT, 'MatekH743-bdshot', tmp_path / 'plain',
                               state_dir=tmp_path)
    assert 'AP_STM32_Timer_BDShot' not in plain['repl'].read_text()
    assert 'AP_STM32_Timer_BDShot' not in plain['resc'].read_text()

    generated = gen_board.generate(ROOT, 'MatekH743-bdshot', tmp_path / 'bdshot',
                                   state_dir=tmp_path, esc_signal_port=57733)
    repl = generated['repl'].read_text()
    assert 'AP_STM32_Timer_BDShot.cs' in generated['resc'].read_text()
    # PWM1/2 are TIM3_CH3/CH4 sharing TIM3_UP/CH3 on DMA2 stream 2
    timer3 = repl.split('timer3BDShot:')[1].split('\n\n')[0]
    assert 'timerAddress: 0x40000400' in timer3
    assert 'stream1: 10' in timer3
    assert 'esc3Port: 57733' in timer3
    assert 'esc4Port: 57743' in timer3
    timer2 = repl.split('timer2BDShot:')[1].split('\n\n')[0]
    assert 'esc1Port: 57753' in timer2
    assert 'esc2Port: 57763' in timer2


def test_built_in_escs_are_generated(tmp_path):
    generated = gen_board.generate(ROOT, 'MatekH743-bdshot', tmp_path / 'sim',
                                   state_dir=tmp_path, esc_sim=True)
    repl = generated['repl'].read_text()
    resc = generated['resc'].read_text()
    assert resc.index('AP_DShotESC.cs') < resc.index('AP_STM32_Timer_BDShot.cs')
    timer3 = repl.split('timer3BDShot:')[1].split('\n\n')[0]
    assert 'escSim: true' in timer3
    assert 'esc3Output: 1' in timer3
    assert 'esc4Output: 2' in timer3
    assert 'Port:' not in timer3


def test_inherited_platforms_are_bridged(tmp_path):
    # the F767 base takes its DMA and timers from the F746 platform it uses
    generated = gen_board.generate(ROOT, 'MatekF765-Wing-bdshot', tmp_path / 'f7',
                                   state_dir=tmp_path, esc_signal_port=57733)
    assert generated['repl'].read_text().count('AP_STM32_Timer_BDShot @') > 0
    assert not [w for w in generated['warnings'] if 'DShot' in w]


def wait_for(condition, timeout=2):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        time.sleep(0.01)
    return condition()


def test_scripted_esc_replies_only_to_bidirectional_frames():
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(('127.0.0.1', 0))
    probe.settimeout(2)
    escs = [dshot_esc.ScriptedESC(0, erpm=21000),
            dshot_esc.ScriptedESC(0, erpm=21000, silent=True)]
    try:
        value = 48 << 1
        normal = (value << 4) | dshot_esc.checksum(value)
        inverted = (value << 4) | (~dshot_esc.checksum(value) & 0xF)
        packet = dshot_esc.PACKET
        probe.sendto(packet.pack(dshot_esc.MAGIC, 2, 4, 0, normal), ('127.0.0.1', escs[0].port))
        probe.sendto(packet.pack(dshot_esc.MAGIC, 2, 4, 1, inverted), ('127.0.0.1', escs[0].port))
        probe.sendto(packet.pack(dshot_esc.MAGIC, 2, 4, 1, inverted), ('127.0.0.1', escs[1].port))
        reply = packet.unpack(probe.recv(64))
        assert reply[0] == dshot_esc.MAGIC and reply[2] == 4
        assert abs(decode_erpm(reply[4]) - 21000) <= 420
        assert wait_for(lambda: escs[1].bidirectional_frames == 1)
        assert [esc.frames for esc in escs] == [2, 1]
        assert [esc.replies for esc in escs] == [1, 0]
    finally:
        for esc in escs:
            esc.close()
        probe.close()
