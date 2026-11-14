# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compile/call every C++ wrapper, and freeze the audited manual wire sizes."""

import json
import struct
import subprocess

import bindings
import call_model
import cpp_bindings
import generate
import pytest

# Audited against the fixed Rust buffers and host codecs, not sizeof(C++).
# Keep these independent of the C++ type-description parser.
MANUAL_SIZES = '''
Parameter.__layout 0 8
shared_state.open 8 4
shared_state.read 10 2056
shared_state.write 2066 8
shared_state.close 4 0
file.read_at 262 259
logger.write 358 0
string.strtof 256 6
string.format_printf 585 258
battery.handle_scripting 91 1
DroneCan.subscribe 11 4
DroneCanSubscription.check_message 4 269
DroneCanHandle.check_message 4 269
DroneCanSubscription.close 4 0
DroneCan.open 12 4
DroneCanHandle.broadcast 262 1
DroneCanHandle.close 4 0
DroneCanHandle.request 263 1
gcs.run_command_int 31 4
Serial.find_serial 1 4
Serial.find_simulated_device 3 4
SerialPort.begin 9 0
SerialPort.configure_parity 5 0
SerialPort.set_stop_bits 5 0
SerialPort.set_unbuffered_writes 5 0
SerialPort.set_flow_control 5 0
SerialPort.available 4 4
SerialPort.read 6 258
SerialPort.write 262 2
mavlink.init 2 0
mavlink.register_rx_msgid 4 1
mavlink.receive_chan 0 304
mavlink.send_chan 267 2
mavlink.block_command 2 1
arming.get_aux_auth_id 0 2
arming.set_aux_auth_passed 1 0
arming.set_aux_auth_failed 43 0
efi.get_backend 1 1
EfiBackend.handle_scripting 102 1
efi.get_state 0 101
RangeFinder.get_backend 1 1
RangeFinderBackend.type 1 1
RangeFinderBackend.status 1 1
RangeFinderBackend.handle_script_distance 5 1
RangeFinderBackend.handle_script_state 14 1
Proximity.get_backend 1 1
ProximityBackend.type 1 1
ProximityBackend.handle_script_distance_msg 14 1
ProximityBackend.set_distance_min_max 9 1
ProximityBackend.update_virtual_boundary 1 1
RangeFinderBackend.get_state 1 13
RangeFinderBackend.orientation 1 4
ProximityBackend.handle_script_3d_msg 14 1
filesystem.stat 256 10
filesystem.read_at 262 1027
filesystem.dir_entry 258 258
filesystem.truncate 256 1
filesystem.append 1282 1
filesystem.unlink 256 1
filesystem.format 0 1
network.socket_open 1 4
network.socket_close 4 0
network.socket_bind 262 1
network.socket_listen 5 1
network.socket_set_blocking 5 0
network.socket_is_connected 4 1
network.socket_pollin 6 1
network.socket_pollout 6 1
network.socket_accept 4 4
network.socket_recv 6 1027
network.socket_send 1030 3
network.socket_sendfile 260 1
network.socket_connect 262 1
network.socket_recv_from 6 1034
network.socket_is_pending 4 1
network.socket_reuse_address 4 1
network.socket_send_to 1036 3
CAN.get_device 1 4
CAN.get_device2 1 4
CanBuffer.read_frame 4 71
CanBuffer.write_frame 78 1
CanBuffer.add_filter 12 1
CanBuffer.close 4 0
esc_telem.update_telem_data 12 0
servo_telem.get_telem 1 6
servo_telem.update_telem_data 33 0
airspeed.get_backend 1 1
AirspeedBackend.handle_script_airspeed 5 1
AirspeedBackend.handle_script_differential_pressure 5 1
AirspeedBackend.handle_script_temperature 5 1
periph.get_vehicle_state 0 8
parameter.old_value 7 5
MotorsDynamic.init 1 1
MotorsDynamic.add_motor 2 1
MotorsDynamic.load_factors 512 0
MotorsMatrix.init 1 1
MotorsMatrix.add_motor_raw 14 0
MotorsMatrix.set_throttle_factor 5 1
Motors.set_frame_string 33 0
Motors6DoF.init 1 1
Motors6DoF.add_motor 27 0
serialLED.set_num_neopixel 2 1
serialLED.set_num_neopixel_rgb 2 1
serialLED.set_num_profiled 2 1
onvif.start 768 1
gps.time_epoch_usec 1 8
gps.inject_data 1026 0
camera.set_stream_information 215 0
networking.add_route 7 1
scripting.restart_all 0 0
crsf.add_menu 16 4
CRSFMenu.add_menu 20 4
CRSFMenu.add_parameter 260 4
CRSFParameter.id 4 1
CRSFParameter.data 4 256
crsf.peek_menu_event 0 62
crsf.pop_menu_event 2 1
crsf.send_write_response 256 1
crsf.send_response 0 1
crsf.get_menu_event 1 61
CRSFMenu.id 4 1
CRSFMenu.name 4 16
CRSFMenu.num_params 4 1
vehicle.set_target_velocity_ned 13 1
srv_channels.get_safety_state 0 1
mission.receive 0 19
DroneCan.get_flex_debug 8 261
i2c.open 7 4
i2c.close 4 0
i2c.set_retries 5 0
i2c.set_address 5 0
i2c.write_register 6 1
i2c.read_registers 6 256
i2c.transfer 261 256
camera.get_state 1 31
camera.set_information 231 0
vehicle.register_custom_mode 40 4
CustomMode.allow_entry 4 1
CustomMode.set_allow_entry 5 0
analog.channel 0 4
AnalogSource.set_pin 5 1
AnalogSource.voltage_average 4 4
AnalogSource.voltage_latest 4 4
AnalogSource.voltage_average_ratiometric 4 4
pwm_source.open 0 4
pwm_source.close 4 0
PwmSource.set_pin 5 1
PwmSource.get_pwm_us 4 2
PwmSource.get_pwm_avg_us 4 2
'''


@pytest.fixture(scope='module')
def specification():
    return bindings.specification()


def test_fortran_bridge_compiles_without_fortran(tmp_path, monkeypatch):
    monkeypatch.setenv('FC', '/no/fortran/installation')
    monkeypatch.setenv('RUSTC', '/no/rust/installation')
    generated = tmp_path / 'generated'
    generate.write_outputs(generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())), generated)
    subprocess.run([
        'g++', '-std=c++17', '-ffreestanding', '-fno-exceptions', '-fno-rtti',
        '-I' + str(generate.ROOT / 'sdk/cpp'), '-I' + str(generated), '-I' + str(generated / 'sdk/cpp'),
        '-c', str(generate.ROOT / 'sdk/fortran/bridge.cpp'), '-o', str(tmp_path / 'bridge.o'),
    ], check=True)


def test_every_cpp_contract_and_buffer_size(tmp_path, specification):
    types, entries, plans = cpp_bindings.describe(specification)
    assert {entry['contract']['name'] for entry in entries} == {p[0] for p in plans}
    manual = {name: (int(ni), int(no)) for name, ni, no in (line.split() for line in MANUAL_SIZES.splitlines() if line)}
    for method in specification['libm']:
        manual['libm.' + method['name']] = (method['arity'] * 4, 4)
    generated = tmp_path / 'generated'
    generate.write_outputs(generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())), generated)
    lines = ['#include <ap_module.hpp>', '#include <stdio.h>', 'using namespace ap_module;',
             'static unsigned current;', 'static bool called;',
             'static int32_t call(void *, uint32_t, const uint8_t *, uint32_t ni, uint8_t *, uint32_t no) {',
             '    called = true; printf("%u %u %u\\n", current, ni, no); return AP_MODULE_STATUS_UNAVAILABLE;', '}',
             'int main() { AP_ModuleHostApi api{}; api.binding_call = call;',
             'ContextBase base(api, nullptr); Context c(base);']
    expected = []
    for index, (name, inputs, outputs, copybacks, sizes, method) in enumerate(plans):
        encoded_input = sum(0 if policy and policy.get('out_only') else
                            (17 if policy['type'] == 'string' else 256) if kind == 'StringView' else types.size(kind)
                            for _, kind, policy in inputs)
        encoded_output = sum(types.size(kind) for _, kind in outputs + copybacks)
        if sizes is None:
            actual = (sum(types.size(t) for _, t, _ in inputs), sum(types.size(t) for _, t in outputs))
            sizes = actual
            assert name in manual, name
        if name in manual:
            assert sizes == manual.pop(name), name
        if isinstance(method, call_model.Call) and isinstance(method.result, call_model.SelectedHandle):
            # These replies carry only presence; the selected index is in the request.
            assert sizes == (encoded_input, 1), name
        else:
            assert sizes == (encoded_input, encoded_output), name
        expected.append(f'{index} {sizes[0]} {sizes[1]}')
        group, _, method = name.partition('.')
        group = 'parameter_cache' if group == 'Parameter' else group.lower()
        lines += ['{', f'current = {index}; called = false;']
        args = []
        for i, (_, kind, _) in enumerate(inputs):
            lines += [f'{kind} a{i}' + ('("x");' if kind == 'StringView' else '{};')]
            args.append(f'a{i}')
        lines += [f'(void)c.{cpp_bindings.identifier(group)}().{cpp_bindings.identifier(method)}({", ".join(args)});',
                  'if (!called) { return 1; }', '}']
    assert not manual
    lines += ['}']
    source = tmp_path / 'all.cpp'
    source.write_text('\n'.join(lines))
    executable = tmp_path / 'all'
    subprocess.run(['g++', '-std=c++17', '-fsanitize=address,undefined', '-g',
                    '-I' + str(generate.ROOT / 'sdk/cpp'), '-I' + str(generated), '-I' + str(generated / 'sdk/cpp'),
                    str(source), str(generated / 'sdk/cpp/imports.cpp'), '-o', str(executable)], check=True)
    result = subprocess.check_output([str(executable)], text=True)
    assert result.splitlines() == expected


def test_unknown_cpp_wire_contract_is_rejected():
    with pytest.raises(ValueError, match='undescribed'):
        cpp_bindings.Types().type('a-new-unaudited-type')


def test_backend_selectors_decode_presence_only_and_keep_input_index(tmp_path):
    generated = tmp_path / 'generated'
    generate.write_outputs(generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())), generated)
    source = tmp_path / 'selectors.cpp'
    source.write_text('''#include <ap_module.hpp>
using namespace ap_module;
static uint8_t presence;
static int32_t status;
static unsigned calls;
static int32_t call(void *, uint32_t, const uint8_t *input, uint32_t ni, uint8_t *output, uint32_t no) {
    calls++;
    if (ni != 1 || no != 1 || input[0] != 197) { return AP_MODULE_STATUS_INVALID; }
    output[0] = presence;
    return status;
}
int main() {
    AP_ModuleHostApi api{}; api.binding_call = call;
    ContextBase base(api, nullptr); Context c(base);
    for (presence = 0; presence <= 2; presence++) {
        auto a = c.airspeed().get_backend(197);
        auto e = c.efi().get_backend(197);
        auto r = c.rangefinder().get_backend(197);
        auto p = c.proximity().get_backend(197);
        if (presence == 2) {
            if (a.error != Error::Host || e.error != Error::Host ||
                r.error != Error::Host || p.error != Error::Host) { return 1; }
        } else {
            if (!a || !e || !r || !p || a.value.present != bool(presence) ||
                e.value.present != bool(presence) || r.value.present != bool(presence) ||
                p.value.present != bool(presence)) { return 2; }
            if (presence && (a.value.value != 197 || e.value.value.value != 197 ||
                             r.value.value.value != 197 || p.value.value.value != 197)) { return 3; }
        }
    }
    status = AP_MODULE_STATUS_UNAVAILABLE;
    if (c.airspeed().get_backend(197).error != Error::Unavailable ||
        c.efi().get_backend(197).error != Error::Unavailable ||
        c.rangefinder().get_backend(197).error != Error::Unavailable ||
        c.proximity().get_backend(197).error != Error::Unavailable) { return 4; }
    return calls == 16 ? 0 : 5;
}
''')
    executable = tmp_path / 'selectors'
    subprocess.run([
        'g++', '-std=c++17', '-fsanitize=address,undefined',
        '-I' + str(generate.ROOT / 'sdk/cpp'), '-I' + str(generated), '-I' + str(generated / 'sdk/cpp'),
        str(source), str(generated / 'sdk/cpp/imports.cpp'), '-o', str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)


def test_manual_wire_order(tmp_path):
    generated = tmp_path / 'generated'
    generate.write_outputs(generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())), generated)
    source = tmp_path / 'packets.cpp'
    source.write_text('''#include <ap_module.hpp>
#include <stdio.h>
using namespace ap_module;
static int32_t call(void *, uint32_t, const uint8_t *p, uint32_t ni, uint8_t *, uint32_t) {
    for (uint32_t i = 0; i < ni; i++) { printf("%02x", p[i]); } puts("");
    return AP_MODULE_STATUS_UNAVAILABLE;
}
int main() {
    AP_ModuleHostApi api{}; api.binding_call = call;
    ContextBase base(api, nullptr); Context c(base);
    Array<uint8_t, 1024> data{}; data.data[0] = 'a'; data.data[1] = 'b'; data.data[2] = 'c';
    c.network().socket_send_to(0x11223344, 3, 0xaabbccdd, 0x1234, data);
    c.serialport().begin(0x11223344, Optional<uint32_t>{true, 0x55667788});
    c.serialport().configure_parity(0x11223344, SerialParity::Even);
    c.serialport().set_stop_bits(0x11223344, SerialStopBits::Two);
    c.serialport().set_flow_control(0x11223344, SerialFlowControl::Auto);
    Array<uint8_t, 41> message{}; message.data[0] = 'a'; message.data[1] = 'b'; message.data[2] = 'c';
    c.arming().set_aux_auth_failed(7, 3, message);
    Array<uint8_t, 32> full{}; full.data[0] = 'a'; full.data[1] = 'b'; full.data[2] = 'c';
    Array<uint8_t, 5> short_name{}; short_name.data[0] = 'x';
    c.vehicle().register_custom_mode(7, 3, full, 1, short_name);
    VideoStreamInformation video{};
    video.framerate = 1.5f; video.bitrate = 0x12345678; video.flags = 0x1122;
    video.resolution_h = 3; video.resolution_v = 4; video.rotation = 5; video.hfov = 6;
    video.stream_id = 7; video.count = 8; video.stream_type = 9; video.encoding = 10;
    video.name.data[0] = 0xa1; video.uri.data[0] = 0xb1;
    c.camera().set_stream_information(11, video);
}
''')
    executable = tmp_path / 'packets'
    subprocess.run(['g++', '-std=c++17', '-fsanitize=address,undefined',
                    '-I' + str(generate.ROOT / 'sdk/cpp'), '-I' + str(generated), '-I' + str(generated / 'sdk/cpp'),
                    str(source), str(generated / 'sdk/cpp/imports.cpp'), '-o', str(executable)], check=True)
    actual = [bytes.fromhex(line) for line in subprocess.check_output([str(executable)], text=True).splitlines()]
    assert actual == [
        struct.pack('<IHIH', 0x11223344, 3, 0xaabbccdd, 0x1234) + b'abc' + bytes(1021),
        struct.pack('<IBI', 0x11223344, 1, 0x55667788),
        struct.pack('<IB', 0x11223344, 2),
        struct.pack('<IB', 0x11223344, 2),
        struct.pack('<IB', 0x11223344, 2),
        bytes([7, 3]) + b'abc' + bytes(38),
        bytes([7, 3]) + b'abc' + bytes(29) + bytes([1]) + b'x' + bytes(4),
        bytes([11]) + struct.pack('<fI5H3B', 1.5, 0x12345678, 0x1122, 3, 4, 5, 6, 7, 8, 9) +
        bytes([0xa1]) + bytes(31) + bytes([0xb1]) + bytes(159) + bytes([10]),
    ]
