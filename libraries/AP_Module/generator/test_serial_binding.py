# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise the real serial adapter with fake physical and simulated ports."""

import ctypes
import os
import struct
import subprocess

import bindings
import pytest
import serial_binding


@pytest.fixture
def serial_host(tmp_path):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '#pragma once\n',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { class Semaphore { public: bool take(uint32_t) { return true; } void give() {} }; }
using HAL_Semaphore = AP_HAL::Semaphore;
''',
        'AP_HAL/UARTDriver.h': '''#pragma once
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
namespace AP_HAL {
class UARTDriver {
public:
    enum flow_control { FLOW_CONTROL_DISABLE=0, FLOW_CONTROL_ENABLE=1,
                        FLOW_CONTROL_AUTO=2, FLOW_CONTROL_RTS_DE=3 };
    virtual ~UARTDriver() = default;
    virtual void begin(uint32_t baud) { last_baud=baud; }
    virtual uint32_t get_baud_rate() const { return 57600; }
    virtual void configure_parity(uint8_t value) { parity=value; }
    virtual void set_stop_bits(int value) { stop=value; }
    virtual bool set_unbuffered_writes(bool value) { unbuffered=value; return true; }
    virtual void set_flow_control(flow_control value) { flow=value; }
    virtual uint32_t available() { return available_value; }
    virtual ssize_t read(uint8_t *out, uint16_t count);
    virtual size_t write(const uint8_t *, size_t count) { writes += count; return count; }
    uint32_t last_baud{}, available_value{4}, writes{};
    uint8_t parity{}; int stop{}; bool unbuffered{}; flow_control flow{FLOW_CONTROL_DISABLE};
};
}
''',
        'AP_SerialManager/AP_SerialManager.h': '''#pragma once
#include <AP_HAL/UARTDriver.h>
#define AP_SERIALMANAGER_ENABLED 1
class AP_SerialManager {
public:
    enum SerialProtocol { SerialProtocol_Scripting=28 };
    AP_HAL::UARTDriver *find_serial(SerialProtocol, uint8_t instance, bool apply_options = true);
};
namespace AP { AP_SerialManager &serialmanager(); }
''',
        'AP_Scripting/AP_Scripting_config.h': '''#pragma once
#define AP_SCRIPTING_ENABLED 1
#define AP_SCRIPTING_SERIALDEVICE_ENABLED 1
''',
        'AP_Scripting/AP_Scripting_SerialDevice.h': '''#pragma once
#include <AP_SerialManager/AP_SerialManager.h>
class AP_Scripting_SerialDevice {
public:
    class Port : public AP_HAL::UARTDriver {
    public:
        size_t device_write(const uint8_t *, size_t count) { device_writes += count; return count; }
        ssize_t device_read(uint8_t *out, uint16_t count);
        uint32_t device_available() { return 9; }
        size_t device_writes{};
    } port;
    static AP_Scripting_SerialDevice *get_singleton();
    uint8_t generation() const { return generation_value; }
    Port *find_port(int8_t protocol, uint16_t instance, uint8_t generation) {
        return enabled && protocol == 28 && instance == 3 && generation == generation_value ? &port : nullptr;
    }
    bool enabled{true}; uint8_t generation_value{5};
};
''',
        'BindingConfig.h': '''#pragma once
#define AP_MODULE_SERIAL_TRANSFER_MAX 256U
''',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    source, library = tmp_path / 'serial.cpp', tmp_path / 'serial.so'
    source.write_text('''#include "BindingSerial.h"
#include <AP_Scripting/AP_Scripting_SerialDevice.h>
#include <string.h>
static AP_HAL::UARTDriver physical;
static AP_SerialManager manager;
static AP_Scripting_SerialDevice broker;
static bool physical_present=true;
ssize_t AP_HAL::UARTDriver::read(uint8_t *out, uint16_t count) {
    const uint16_t n=count < 3 ? count : 3; memcpy(out, "phy", n); return n;
}
ssize_t AP_Scripting_SerialDevice::Port::device_read(uint8_t *out, uint16_t count) {
    const uint16_t n=count < 3 ? count : 3; memcpy(out, "dev", n); return n;
}
AP_HAL::UARTDriver *AP_SerialManager::find_serial(SerialProtocol, uint8_t instance, bool) {
    return physical_present && instance == 2 ? &physical : nullptr;
}
AP_SerialManager &AP::serialmanager() { return manager; }
AP_Scripting_SerialDevice *AP_Scripting_SerialDevice::get_singleton() { return &broker; }
extern "C" int call(unsigned op, const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    using namespace AP_Module;
    switch (op) {
    case 0: return binding_serial_find(in,ni,out,no);
    case 1: return binding_serial_find_simulated(in,ni,out,no);
    case 2: return binding_serial_begin(in,ni,out,no);
    case 3: return binding_serial_configure_parity(in,ni,out,no);
    case 4: return binding_serial_set_stop_bits(in,ni,out,no);
    case 5: return binding_serial_set_unbuffered_writes(in,ni,out,no);
    case 6: return binding_serial_set_flow_control(in,ni,out,no);
    case 7: return binding_serial_available(in,ni,out,no);
    case 8: return binding_serial_read(in,ni,out,no);
    case 9: return binding_serial_write(in,ni,out,no);
    }
    return 99;
}
extern "C" void physical_enable(bool value) { physical_present=value; }
extern "C" void next_generation() { broker.generation_value++; }
extern "C" unsigned physical_baud() { return physical.last_baud; }
extern "C" unsigned physical_settings() {
    return physical.parity | (unsigned(physical.stop)<<8) | (unsigned(physical.unbuffered)<<16) |
           (unsigned(physical.flow)<<24);
}
''')
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-DAP_MODULE_ENABLED=1', '-DAP_MODULE_SERIAL_TRANSFER_MAX=256U',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
                    str(bindings.ROOT / 'BindingSerial.cpp'), str(source), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    return lib


def invoke(lib, op, packet, output_size):
    output = ctypes.create_string_buffer(output_size) if output_size else None
    status = lib.call(op, packet, len(packet), output, output_size)
    return status, output.raw if output is not None else b''


def test_revalidated_selectors_and_operations(serial_host):
    status, output = invoke(serial_host, 0, b'\2', 4)
    assert status == 0
    physical = output
    assert physical == struct.pack('<I', 3)
    status, output = invoke(serial_host, 1, struct.pack('<BH', 28, 3), 4)
    assert status == 0
    simulated = output
    assert struct.unpack('<I', simulated)[0] & 0x80000000

    assert invoke(serial_host, 2, physical + b'\1' + struct.pack('<I', 115200), 0)[0] == 0
    assert invoke(serial_host, 3, physical + b'\2', 0)[0] == 0
    assert invoke(serial_host, 4, physical + b'\2', 0)[0] == 0
    assert invoke(serial_host, 5, physical + b'\1', 0)[0] == 0
    assert invoke(serial_host, 6, physical + b'\3', 0)[0] == 0
    assert serial_host.physical_baud() == 115200
    assert serial_host.physical_settings() == 0x03010202
    assert invoke(serial_host, 7, physical, 4) == (0, struct.pack('<I', 4))
    assert invoke(serial_host, 7, simulated, 4) == (0, struct.pack('<I', 9))

    status, output = invoke(serial_host, 8, physical + struct.pack('<H', 6), 258)
    assert status == 0 and output[:5] == b'\3\0phy' and output[5:] == bytes(253)
    packet = simulated + struct.pack('<H', 3) + b'abc' + bytes(253)
    assert invoke(serial_host, 9, packet, 2) == (0, struct.pack('<H', 3))

    serial_host.physical_enable(False)
    assert invoke(serial_host, 7, physical, 4)[0] == -3
    serial_host.next_generation()
    assert invoke(serial_host, 7, simulated, 4)[0] == -3


def test_serial_rejects_noncanonical_packets(serial_host):
    physical = struct.pack('<I', 3)
    bad = [
        (2, physical + b'\0' + struct.pack('<I', 1), 0),
        (3, physical + b'\3', 0),
        (4, physical + b'\0', 0),
        (6, physical + b'\4', 0),
        (8, physical + struct.pack('<H', 0), 258),
        (8, physical + struct.pack('<H', 257), 258),
        (9, physical + struct.pack('<H', 1) + b'x' + b'y' + bytes(254), 2),
    ]
    for op, packet, output_size in bad:
        assert invoke(serial_host, op, packet, output_size)[0] == -1


def test_serial_contract_and_sdk_are_explicit():
    spec = serial_binding.specification(40)
    assert [method['id'] for method in spec['methods']] == list(range(40, 50))
    assert all('revalidate-every-call' in method['policy'] for method in spec['methods'])
    rust = serial_binding.rust(spec, '')
    assert 'pub struct SerialPort' in rust
    assert 'SERIAL_TRANSFER_MAX: usize = 256' in rust
    assert 'pub fn read_byte' in rust and 'pub fn write_byte' in rust
