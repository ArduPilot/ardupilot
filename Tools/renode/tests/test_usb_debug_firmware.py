# AP_FLAKE8_CLEAN
"""Opt-in real-firmware tests over userspace USB/IP; see patches/README.md.

USB_DEBUG_ELF must match a freshly launched Renode CubeOrangePlus Plane image.
No kernel USB/IP attachment or physical board is used.
"""

import os
import queue
import re
import socket
import struct
import subprocess
import threading
import time

from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree

import pytest

from usb_debug_transport import GCS
from usb_debug_transport import GDB
from usb_debug_transport import USB
from usb_debug_transport import gdb


def register(debug, number):
    return int.from_bytes(bytes.fromhex(debug.packet(f'p{number:x}'.encode()).decode()), 'little')


def set_register(debug, number, value, size=4):
    assert debug.packet(f'P{number:x}={value.to_bytes(size, "little").hex()}'.encode()) == b'OK'


def read_memory(debug, address, size):
    return bytes.fromhex(debug.packet(f'm{address:x},{size:x}'.encode()).decode())


def write_memory(debug, address, data):
    assert debug.packet(f'M{address:x},{len(data):x}:{data.hex()}'.encode()) == b'OK'


def resume(debug):
    debug.write(gdb.rsp_packet(b'c'))
    return gdb.read_packet(debug, time.monotonic()+30)


def monitor(command):
    port = os.environ.get('USB_DEBUG_MONITOR_PORT')
    if not port:
        pytest.skip('set USB_DEBUG_MONITOR_PORT for interrupt/register injection')
    with socket.create_connection(('127.0.0.1', int(port)), timeout=2) as stream:
        stream.settimeout(.1)
        stream.sendall(b'\x15'+command.encode()+b'\r')
        output = b''
        deadline = time.monotonic()+.3
        while time.monotonic() < deadline:
            try:
                output += stream.recv(65536)
            except socket.timeout:
                pass
        return output


def monitor_read(address):
    output = monitor(f'sysbus ReadDoubleWord 0x{address:x}')
    return int(re.findall(rb'0x([0-9a-fA-F]{8})', output)[-1], 16)


def monitor_write(address, value):
    monitor(f'sysbus WriteDoubleWord 0x{address:x} 0x{value:x}')


def interrupt_running(debug):
    debug.write(gdb.rsp_packet(b'c'))
    deadline = time.monotonic()+5
    ack = b''
    while not ack and time.monotonic() < deadline:
        ack = debug.read(1)
    assert ack == b'+'
    time.sleep(.3)
    debug.write(b'\x03')
    assert gdb.read_packet(debug, time.monotonic()+15).startswith(b'T02')


@pytest.fixture(scope='module')
def session():
    elf = os.environ.get('USB_DEBUG_ELF')
    if not elf:
        pytest.skip('set USB_DEBUG_ELF and launch a matching Renode --usb instance')
    assert Path(elf).is_file()
    pytest.importorskip('pymavlink')
    port = int(os.environ.get('USB_DEBUG_USBIP_PORT', '23451'))
    usb = USB(port)
    startup = os.environ.get('USB_DEBUG_STARTUP_WAIT') == '1'
    if not startup:
        assert GCS(usb).recv('HEARTBEAT', 60)
    # A running application-owned SysTick must not prevent attachment or be
    # reprogrammed by the debugger. Leave its interrupt off until the dedicated
    # coexistence test installs a handler; the application's vector may be weak.
    if os.environ.get('USB_DEBUG_MONITOR_PORT'):
        saved_systick = (monitor_read(0xe000e010) & 7, monitor_read(0xe000e014))
        saved_vtor = monitor_read(0xe000ed08)
        saved_usb_priority = monitor_read(0xe000e464)
        monitor_write(0xe000e014, 0x54321)
        monitor_write(0xe000e018, 0)
        monitor_write(0xe000e010, 5)
    usb.write(4, gdb.TRIGGER)
    # Wait for the device to leave ChibiOS before configuring the monitor's
    # connection. A fixed sleep races main-loop scheduling on slower hosts.
    deadline = time.monotonic()+15
    while usb.running and time.monotonic() < deadline:
        time.sleep(.1)
    assert not usb.running, 'firmware did not switch to the USB monitor'
    usb.close()
    usb = USB(port)
    debug = GDB(usb)
    assert b'ap-usb-debug+' in debug.packet(b'qSupported')
    assert debug.packet(b'?').startswith(b'T05')
    if os.environ.get('USB_DEBUG_MONITOR_PORT'):
        assert monitor_read(0xe000e010) & 7 == 5
        assert monitor_read(0xe000e014) == 0x54321
        assert monitor_read(0xe000ed08) != saved_vtor
        # CubeOrangePlus OTG1 is IRQ 101, the second byte in this IPR word.
        # Verify that this really is the vector redirected to our raw handler.
        symbols = subprocess.check_output(['arm-none-eabi-nm', elf], text=True)
        handler_symbol = next((line for line in symbols.splitlines()
                               if line.endswith(' USB_Debug_Handler')), None)
        assert handler_symbol is not None, 'USB_Debug_Handler is missing from the test ELF'
        handler = int(handler_symbol.split()[0], 16)
        assert monitor_read(monitor_read(0xe000ed08)+(16+101)*4) == handler | 1
        assert monitor_read(0xe000e464) & 0xff00 == 0
    if startup:
        # setup() runs once, so reaching this breakpoint after attachment proves
        # the initial wait stopped before vehicle setup, not after it.
        symbols = subprocess.check_output(['arm-none-eabi-nm', '-C', elf], text=True)
        address = int(next(line.split()[0] for line in symbols.splitlines()
                           if line.endswith(' T AP_Vehicle::setup()')), 16)
        assert debug.packet(f'Z1,{address:x},2'.encode()) == b'OK'
        assert resume(debug).startswith(b'T05')
        assert register(debug, 15) == address
        assert debug.packet(f'z1,{address:x},2'.encode()) == b'OK'
    # Always borrow stack space from the main thread after setup. The idle
    # thread's tiny stack cannot accommodate scratch code and test data.
    symbols = subprocess.check_output(['arm-none-eabi-nm', '-C', elf], text=True)
    address = int(next(line.split()[0] for line in symbols.splitlines()
                       if line.endswith(' T Plane::one_second_loop()')), 16)
    assert debug.packet(f'Z1,{address:x},2'.encode()) == b'OK'
    assert resume(debug).startswith(b'T05')
    assert register(debug, 15) == address
    assert debug.packet(f'z1,{address:x},2'.encode()) == b'OK'
    saved = debug.packet(b'g')
    assert len(saved) == 400
    # Borrow unused space below the interrupted thread's SP, clear of its
    # architectural exception frame. Restore it before normal execution.
    scratch = (register(debug, 13)-512) & ~31
    layout = subprocess.check_output(['arm-none-eabi-gdb', '-nx', '-batch', elf,
                                      '-ex', 'p/x (unsigned long)&((thread_t*)0)->wabase',
                                      '-ex', 'p sizeof(thread_t)'], text=True)
    base_offset = int(re.search(r'\$1 = 0x([0-9a-f]+)', layout)[1], 16)
    thread_size = int(re.search(r'\$2 = (\d+)', layout)[1])
    thread = int(debug.packet(b'qC')[2:], 16)
    base = int.from_bytes(read_memory(debug, thread+base_offset, 4), 'little')
    assert scratch >= base+thread_size, 'scratch would overlap thread/kernel state'
    original = read_memory(debug, scratch, 128)
    state = SimpleNamespace(debug=debug, usb=usb, elf=elf, scratch=scratch, saved=saved, original=original)
    yield state
    assert debug.packet(b'G'+saved) == b'OK'
    write_memory(debug, scratch, original)
    assert debug.packet(b'D') == b'OK'
    deadline = time.monotonic()+10
    while usb.running and time.monotonic() < deadline:
        time.sleep(.05)
    assert not usb.running, 'firmware did not disconnect USB on detach'
    time.sleep(.5)
    usb.close()
    usb = USB(port)
    assert GCS(usb).recv('HEARTBEAT', 60)
    if os.environ.get('USB_DEBUG_MONITOR_PORT'):
        assert monitor_read(0xe000e010) & 7 == 5
        assert monitor_read(0xe000e014) == 0x54321
        assert monitor_read(0xe000ed08) == saved_vtor
        assert monitor_read(0xe000e464) == saved_usb_priority
        monitor_write(0xe000e010, 0)
        monitor_write(0xe000e014, saved_systick[1])
        monitor_write(0xe000e010, saved_systick[0])
    usb.close()


@pytest.fixture
def stopped(session):
    yield session
    assert session.debug.packet(b'G'+session.saved) == b'OK'
    write_memory(session.debug, session.scratch, session.original)


def test_memory_assignment(stopped):
    d, address = stopped.debug, stopped.scratch
    data = bytes(range(64))
    write_memory(d, address, data)
    assert read_memory(d, address, len(data)) == data
    binary = b'\x00#$}*\xff\x01\x02'
    escaped = bytearray()
    for value in binary:
        if value in b'#$}*':
            escaped.extend((ord('}'), value ^ 0x20))
        else:
            escaped.append(value)
    assert d.packet(f'X{address+1:x},{len(binary):x}:'.encode()+escaped) == b'OK'
    assert read_memory(d, address+1, len(binary)) == binary
    assert d.packet(b'X0,0:') == b'OK'
    for packet in (b'M08020000,1:00', b'Me000ed00,1:00', b'Mffffffff,2:0000',
                   f'M{address:x},2:01'.encode(), f'M{address:x},1:gg'.encode()):
        assert d.packet(packet).startswith(b'E')
    before = read_memory(d, address, 2)
    for data in (b'12g4', b'12\xff4', b'12\x804'):
        assert d.packet(f'M{address:x},2:'.encode()+data) == b'E01'
        assert read_memory(d, address, 2) == before
    assert d.packet(f'M{address:x},2:AbCd'.encode()) == b'OK'
    assert read_memory(d, address, 2) == b'\xab\xcd'


def test_target_description(stopped):
    d = stopped.debug

    def chunk(offset, length):
        return d.packet(f'qXfer:features:read:target.xml:{offset:x},{length:x}'.encode())

    description = b''
    while True:
        part = chunk(len(description), 79)
        assert part[:1] in (b'm', b'l')
        assert len(part) <= 80
        description += part[1:]
        if part[:1] == b'l':
            break
        assert len(part) > 1
        assert len(description) < 4096
    target = ElementTree.fromstring(description)
    assert target.find('architecture').text == 'arm'
    registers = target.findall('feature/reg')
    assert [r.attrib['name'] for r in registers] == (
        [f'r{i}' for i in range(13)] + ['sp', 'lr', 'pc', 'xpsr'] +
        [f'd{i}' for i in range(16)]+['fpscr'])
    assert [r.attrib['bitsize'] for r in registers] == ['32']*17+['64']*16+['32']
    assert registers[13].attrib['type'] == 'data_ptr'
    assert registers[15].attrib['type'] == 'code_ptr'
    assert registers[16].attrib['regnum'] == '25'
    for i, reg in enumerate(registers[17:33]):
        assert reg.attrib['type'] == 'ieee_double'
        assert reg.attrib['regnum'] == str(26+i)
    assert registers[-1].attrib['regnum'] == '42'
    # Request slices across element boundaries, at EOF, and with capped lengths.
    for offset in (0, 1, len(description)//2, len(description)-1, len(description)):
        for length in (0, 1, 7, 512, 0xffffffff):
            data = description[offset:offset+min(length, 511)]
            last = offset+len(data) == len(description)
            assert chunk(offset, length) == (b'l' if last else b'm')+data
    assert chunk(len(description)+1, 1) == b'E01'
    assert chunk(0xffffffff, 1) == b'E01'


def test_invalid_checksum(stopped):
    d = stopped.debug
    for checksum in (b'z0', b'0z', b'\xff0', b'00'):
        d.write(b'$qC#'+checksum)
        deadline = time.monotonic()+5
        ack = b''
        while not ack and time.monotonic() < deadline:
            ack = d.read(1)
        assert ack == b'-'
        assert d.packet(b'qC').startswith(b'QC')


def test_registers_and_suspended_thread(stopped):
    d = stopped.debug
    for number, value, size in [(0, 0x12345678, 4), (4, 0x87654321, 4), (26, 0x4008000000000000, 8)]:
        old = d.packet(f'p{number:x}'.encode())
        set_register(d, number, value, size)
        assert register(d, number) == value
        assert d.packet(f'P{number:x}='.encode()+old) == b'OK'
    current = int(d.packet(b'qC')[2:], 16)
    threads = [int(value, 16) for value in d.packet(b'qfThreadInfo')[1:].split(b',')]
    other = next(thread for thread in threads if thread != current)
    assert d.packet(f'Hg{other:x}'.encode()) == b'OK'
    try:
        for number, size in [(4, 4), (34, 8)]:  # r4 and d8, saved by __port_switch
            old = d.packet(f'p{number:x}'.encode())
            assert b'x' not in old
            set_register(d, number, 123, size)
            assert register(d, number) == 123
            assert d.packet(f'P{number:x}='.encode()+old) == b'OK'
        # An IRQ-suspended thread also has caller-saved registers, while a
        # cooperative switch does not. Exercise whichever frame was captured.
        old = d.packet(b'p0')
        if b'x' in old:
            assert d.packet(b'P0=00000000').startswith(b'E')
        else:
            set_register(d, 0, 123)
            assert register(d, 0) == 123
            assert d.packet(b'P0='+old) == b'OK'
        assert d.packet(b'Pd='+d.packet(b'pd')).startswith(b'E')
    finally:
        assert d.packet(f'Hg{current:x}'.encode()) == b'OK'
    assert d.packet(b'G'+stopped.saved) == b'OK'
    assert d.packet(b'P0=xyz').startswith(b'E')


@pytest.mark.parametrize('kind,offset,reason', [(2, 6, b'watch:'), (3, 8, b'rwatch:'), (4, 6, b'awatch:')])
def test_watchpoint(stopped, kind, offset, reason):
    d, address = stopped.debug, stopped.scratch
    # movs r0,41; adds r0,1; str r0,[r1]; ldr r2,[r1]; bkpt
    write_memory(d, address, bytes.fromhex('2920013008600a6800be'))
    set_register(d, 1, address+64)
    set_register(d, 15, address)
    assert d.packet(f'Z{kind},{address+64:x},4'.encode()) == b'OK'
    assert reason in resume(d)
    assert register(d, 15) == address+offset
    assert read_memory(d, address+64, 4) == (42).to_bytes(4, 'little')
    assert d.packet(f'z{kind},{address+64:x},4'.encode()) == b'OK'
    assert resume(d).startswith(b'T05')
    assert register(d, 15) == address+8
    assert d.packet(f'Z{kind},{address+65:x},4'.encode()).startswith(b'E')


def test_ram_breakpoint(stopped):
    d, address = stopped.debug, stopped.scratch
    code = bytes.fromhex('2920013008600a6800be')
    write_memory(d, address, code)
    assert d.packet(f'Z0,{address+2:x},2'.encode()) == b'OK'
    assert read_memory(d, address, len(code)) == code
    set_register(d, 1, address+64)
    set_register(d, 15, address)
    assert resume(d).startswith(b'T05')
    assert register(d, 15) == address+2
    assert d.packet(b's').startswith(b'T05')
    assert register(d, 15) == address+4
    assert register(d, 0) == 42
    assert resume(d).startswith(b'T05')
    set_register(d, 15, address)
    assert resume(d).startswith(b'T05')
    assert register(d, 15) == address+2
    assert resume(d).startswith(b'T05')  # automatic step-over
    assert register(d, 15) == address+8
    assert register(d, 0) == 42
    write_memory(d, address+2, bytes.fromhex('0230'))
    assert read_memory(d, address+2, 2) == bytes.fromhex('0230')
    assert d.packet(f'z0,{address+2:x},2'.encode()) == b'OK'
    set_register(d, 15, address)
    assert resume(d).startswith(b'T05')
    assert register(d, 0) == 43


def test_floating_point_execution(stopped, tmp_path):
    d, address = stopped.debug, stopped.scratch
    assembly = tmp_path/'fp.S'
    assembly.write_text('.syntax unified\n.thumb\n.fpu fpv5-sp-d16\n'
                        'nop\nvadd.f32 s0,s0,s1\nvstr s0,[r1]\nbkpt #0\n')
    obj, binary = tmp_path/'fp.o', tmp_path/'fp.bin'
    subprocess.run(['arm-none-eabi-gcc', '-c', '-mcpu=cortex-m7', '-mthumb',
                    str(assembly), '-o', str(obj)], check=True, capture_output=True)
    subprocess.run(['arm-none-eabi-objcopy', '-O', 'binary', '-j', '.text', str(obj), str(binary)], check=True)
    write_memory(d, address, binary.read_bytes())
    set_register(d, 26, int.from_bytes(struct.pack('<ff', 1., 2.), 'little'), 8)
    set_register(d, 42, 0x01000000)  # FPSCR.FZ must survive exception entry
    set_register(d, 1, address+64)
    set_register(d, 15, address)
    assert d.packet(f'Z0,{address+2:x},4'.encode()) == b'OK'
    assert resume(d).startswith(b'T05')
    assert register(d, 15) == address+2
    assert d.packet(b's').startswith(b'T05')
    assert register(d, 15) == address+6  # One 32-bit VADD instruction
    assert d.packet(f'z0,{address+2:x},4'.encode()) == b'OK'
    assert resume(d).startswith(b'T05')
    assert read_memory(d, address+64, 4) == struct.pack('<f', 3.)
    assert register(d, 26) & 0xffffffff == 0x40400000
    assert register(d, 42) & 0x01000000


def test_fault_recovery(stopped):
    d, address = stopped.debug, stopped.scratch
    write_memory(d, address, bytes.fromhex('00de00be'))  # UDF then BKPT
    set_register(d, 15, address)
    assert resume(d).startswith(b'T04')
    status = bytes.fromhex(d.packet(b'qRcmd,6661756c74').decode()).decode()
    assert re.search(r'exception=[36] ', status)
    assert 'CFSR=00010000' in status
    assert d.packet(b's').startswith(b'E')
    set_register(d, 15, address+2)
    assert resume(d).startswith(b'T05')
    assert register(d, 15) == address+2


def test_cdc_line_coding(stopped):
    # Windows configures each CDC with SET followed immediately by GET. Keep
    # both interfaces working while the CPU is stopped in the debug monitor.
    for interface in (0, 2):
        original = stopped.usb.ctrl(0xa1, 0x21, 0, interface, 7)
        try:
            for baud in (9600, 115200, 57600, 115200):
                coding = baud.to_bytes(4, 'little') + bytes((0, 0, 8))
                stopped.usb.ctrl(0x21, 0x22, 0, interface, 0)
                stopped.usb.ctrl(0x21, 0x20, 0, interface, 7, coding)
                assert stopped.usb.ctrl(0xa1, 0x21, 0, interface, 7) == coding
        finally:
            stopped.usb.ctrl(0x21, 0x20, 0, interface, 7, original)


def test_real_gdb_function_call(stopped):
    server = socket.socket()
    server.bind(('127.0.0.1', 0))
    server.listen(1)
    done = threading.Event()
    workers = []

    def bridge():
        conn, _ = server.accept()

        def forward_usb():
            while not done.is_set():
                try:
                    data = stopped.usb.queues[4].get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    conn.sendall(data)
                except OSError:
                    break
        worker = threading.Thread(target=forward_usb, daemon=True)
        workers.append(worker)
        worker.start()
        try:
            while not done.is_set():
                data = conn.recv(4096)
                if not data:
                    break
                stopped.usb.write(4, data)
        finally:
            conn.close()

    worker = threading.Thread(target=bridge, daemon=True)
    worker.start()
    commands = ['set pagination off', 'set remotetimeout 20',
                f'target remote 127.0.0.1:{server.getsockname()[1]}',
                'set $oldr4=$r4', 'set $r4=0x12345678', 'printf "R4=%x\\n", $r4', 'set $r4=$oldr4',
                'set $oldd0=$d0', 'set $d0=3.5', 'printf "D0=%g\\n", $d0', 'set $d0=$oldd0',
                'printf "MILLIS=%u\\n", AP_HAL::millis()', 'disconnect', 'quit']
    args = ['arm-none-eabi-gdb', '-nx', '-batch', stopped.elf]
    for command in commands:
        args.extend(['-ex', command])
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=90)
        output = result.stdout+result.stderr
        assert result.returncode == 0, output
        assert 'R4=12345678' in output, output
        assert 'D0=3.5' in output, output
        assert re.search(r'MILLIS=\d+', output), output
    finally:
        done.set()
        server.close()
        worker.join(2)
        for worker in workers:
            worker.join(2)


def test_guarded_mpu_write(stopped):
    """A real guest MPU fault during M must return E01, not hang DebugMonitor."""
    read = monitor_read
    write = monitor_write
    control, selected = read(0xe000ed94), read(0xe000ed98)
    write(0xe000ed98, 15)
    base, attributes = read(0xe000ed9c), read(0xe000eda0)
    d, address = stopped.debug, stopped.scratch+64
    original = read_memory(d, address, 4)
    try:
        write(0xe000ed9c, address)
        write(0xe000eda0, 0x16000009)  # 32 bytes, read-only, execute-never
        write(0xe000ed94, 7)  # enforce even in HardFault, privileged background
        assert d.packet(f'M{address:x},4:01020304'.encode()).startswith(b'E')
        assert read_memory(d, address, 4) == original
        assert d.packet(b'g') == stopped.saved
    finally:
        write(0xe000ed94, control)
        write(0xe000ed98, 15)
        write(0xe000ed9c, base)
        write(0xe000eda0, attributes)
        write(0xe000ed98, selected)


@pytest.mark.parametrize('basepri', [0, 0x10])
def test_usb_interrupts_busy_thread(stopped, basepri):
    """Ctrl-C preempts a thread even with the ChibiOS kernel priority masked."""
    d, address = stopped.debug, stopped.scratch
    # movs r0,#mask; msr BASEPRI,r0; spin: nop; b spin;
    # clear: movs r0,#0; msr BASEPRI,r0; bkpt
    code = bytes([basepri, 0x20])+bytes.fromhex('80f3118800bffde7002080f3118800be')
    write_memory(d, address, code)
    set_register(d, 15, address)
    interrupt_running(d)
    assert register(d, 15) in (address+6, address+8)
    assert register(d, 25) & 0x1ff == 0
    # Restore BASEPRI by executing the cleanup, before restoring thread registers.
    set_register(d, 15, address+10)
    assert resume(d).startswith(b'T05')


def test_systick_runs_alongside_usb(stopped):
    """An application SysTick at priority one coexists with Ctrl-C and stepping."""
    d, address = stopped.debug, stopped.scratch
    vtor = monitor_read(0xe000ed08)
    old_vector = monitor_read(vtor+15*4)
    old_priority = monitor_read(0xe000ed20)
    old_reload = monitor_read(0xe000e014)
    old_control = monitor_read(0xe000e010) & 7
    # Thread spins; handler increments a counter and exception-returns.
    write_memory(d, address, bytes.fromhex('00bffde7'))
    handler = bytes.fromhex('0248016801310160704700bf')+struct.pack('<I', address+96)
    write_memory(d, address+32, handler)
    write_memory(d, address+96, bytes(4))
    monitor_write(vtor+15*4, (address+32) | 1)
    monitor_write(0xe000ed20, (old_priority & 0x00ffffff) | 0x10000000)
    monitor_write(0xe000e014, 400000)
    monitor_write(0xe000e018, 0)
    monitor_write(0xe000e010, 7)
    try:
        set_register(d, 15, address)
        interrupt_running(d)
        assert int.from_bytes(read_memory(d, address+96, 4), 'little') > 0
        assert monitor_read(0xe000e010) & 7 == 7
        assert monitor_read(0xe000e014) == 400000
        # MON_STEP must mask only our USB IRQ, leaving SysTick configured.
        d.write(gdb.rsp_packet(b's'))
        assert gdb.read_packet(d, time.monotonic()+15).startswith(b'T05')
        assert monitor_read(0xe000e010) & 7 == 7
        assert monitor_read(0xe000e014) == 400000
    finally:
        monitor_write(0xe000e010, 0)
        monitor_write(0xe000ed04, 1 << 25)  # clear any pending SysTick
        monitor_write(vtor+15*4, old_vector)
        monitor_write(0xe000ed20, old_priority)
        monitor_write(0xe000e014, old_reload)
        monitor_write(0xe000e010, old_control)


def test_usb_interrupts_irq_storm(stopped):
    """A self-pending priority-six ISR cannot starve the USB debugger."""
    d, address = stopped.debug, stopped.scratch
    irq = 130  # otherwise unused in this CubeOrangePlus test instance
    vtor = monitor_read(0xe000ed08)
    old_vector = monitor_read(vtor+(16+irq)*4)
    old_priority = monitor_read(0xe000e480)
    assert monitor_read(0xe000e110) & 4 == 0
    # The ISR pends itself again and returns, creating continuous tail-chaining.
    write_memory(d, address, bytes.fromhex('00bffde7'))
    handler = bytes.fromhex('0148042101607047')+struct.pack('<I', 0xe000e210)
    write_memory(d, address+32, handler)
    monitor_write(vtor+(16+irq)*4, (address+32) | 1)
    monitor_write(0xe000e480, (old_priority & 0xff00ffff) | 0x00600000)
    monitor_write(0xe000e110, 4)
    monitor_write(0xe000e210, 4)
    try:
        set_register(d, 15, address)
        interrupt_running(d)
        assert register(d, 25) & 0x1ff == 16+irq
        assert address+32 <= register(d, 15) < address+40
        # Remove the storm, then let the interrupted ISR return to the thread.
        monitor_write(0xe000e190, 4)
        monitor_write(0xe000e290, 4)
        set_register(d, 15, address+38)  # bx lr
        interrupt_running(d)
        assert register(d, 25) & 0x1ff == 0
    finally:
        monitor_write(0xe000e190, 4)
        monitor_write(0xe000e290, 4)
        monitor_write(vtor+(16+irq)*4, old_vector)
        monitor_write(0xe000e480, old_priority)
