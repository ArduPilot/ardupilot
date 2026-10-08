#!/usr/bin/env python3
'''
Boot an AP_HAL_Zephyr firmware under Renode and wait for a MAVLink heartbeat.

Zephyr boards have no Renode platform of their own. They do not need one: a
Zephyr board built for the same silicon as a ChibiOS board runs on that board's
generated platform, because run.py builds the platform from the MCU in
hwdef.dat and not from anything ChibiOS-specific. CubeOrangeZephyr is the same
H743 as CubeOrange, so:

    Tools/renode/zephyr_boot_check.py

is CubeOrangeZephyr's firmware on CubeOrange's platform. Exits 0 once a
HEARTBEAT arrives on the emulated UART, 1 on timeout.

Deliberately not named test_*: Tools/renode/tests is collected by pytest in CI,
and this needs a Zephyr toolchain and a built ELF that CI does not have.
AP_FLAKE8_CLEAN
'''

import argparse
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from process_utils import terminate_process_group  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# MAVLink v2 framing: 0xFD, len, incompat, compat, seq, sysid, compid, msgid[3]
MAVLINK2_MAGIC = 0xFD
MSGID_HEARTBEAT = 0

# The monitor colours its prompt.
ANSI = re.compile(rb'\x1b\[[0-9;]*[A-Za-z]')

# The monitor prompt is the current machine in parentheses: "(monitor)" before
# a machine is selected, "(machine-0)" after. Anchored, so a VALUE that merely
# happens to end in ')' is not mistaken for the prompt.
PROMPT = re.compile(rb'\([^()\r\n]*\)\s*$')


def _drain_quiet(sock, deadline, quiet=0.4):
    '''Read and discard until the socket has been silent for `quiet` seconds.

    NOT "until a prompt". The monitor session is shared and its output is
    replayed to a new connection, so what is waiting when we connect is an
    unbounded amount of history - the banner, the .resc echo, "Starting
    emulation...", and a prompt after each. Stopping at the first prompt
    leaves the rest queued, and the next read then returns THAT instead of the
    answer, one step behind for the rest of the session.
    '''
    sock.settimeout(quiet)
    while time.time() < deadline:
        try:
            if not sock.recv(4096):
                break
        except socket.timeout:
            break


def monitor_query(port, command, timeout=10.0):
    '''Ask the Renode monitor for one value; None if it could not be read.

    Reads until the reply is complete rather than for a fixed number of
    recv()s: the monitor sends the echo, the value and the prompt in whatever
    chunking it likes, and waiting for the socket timeout on every query turns
    a handful of them into minutes.

    The answer is located by OUR OWN ECHO, not by the prompt. Two things make
    prompt-hunting wrong here. Renode greets a connection with a version
    banner and a prompt, so the first read-to-prompt returns "Renode, version
    1.16.1" rather than the value. And the monitor session is shared, so a
    connection also inherits whatever was queued before it - the .resc echo,
    "Starting emulation...", each with its own prompt. Anchoring on the prompt
    therefore returns the PREVIOUS command's output: observed live as
    BytesMoved -> None, then BeatsPerChannel -> 'sysbus.edma0 BytesMoved'.

    None of that announces itself. check_edma() either reports "could not
    read" or fails to parse, and the rt1176 job then annotates a board that
    booted perfectly well as not having booted.

    So: drain until the socket is quiet, send, then read until our echo has
    been seen AND a prompt has arrived after it, and take the value from
    between them.

    Note the monitor prints numbers in HEX, without a leading 0x on some
    builds, and cannot read CPU registers r0-r7 on Cortex-M - use the GDB
    server for those.
    '''
    try:
        sock = socket.create_connection(('127.0.0.1', port), timeout=timeout)
    except OSError:
        return None
    echo = command.encode()
    try:
        deadline = time.time() + timeout
        _drain_quiet(sock, deadline)
        sock.settimeout(1.0)
        sock.sendall(echo + b'\n')
        buf = b''
        while time.time() < deadline:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            buf += chunk
            clean = ANSI.sub(b'', buf)
            at = clean.rfind(echo)
            if at >= 0 and PROMPT.search(clean[at + len(echo):].rstrip()):
                break
    finally:
        sock.close()
    clean = ANSI.sub(b'', buf)
    at = clean.rfind(echo)
    if at < 0:
        return None                     # never saw our command come back
    tail = clean[at + len(echo):].decode('ascii', 'replace')
    for line in (raw.strip() for raw in tail.splitlines()):
        if not line:
            continue
        if PROMPT.match(line.encode()):
            continue                    # the prompt that closes the reply
        return line
    return None


def check_edma(port):
    '''Assert the emulated eDMA actually carried traffic for this guest.

    A heartbeat on its own does not prove the DMA path ran. UARTDriver::_begin
    falls back to the interrupt-driven path whenever uart_callback_set() fails
    or a DMA buffer cannot be allocated, and prints one line before carrying on
    happily - so a board with a completely dead eDMA still boots and still
    heartbeats. The model keeps counters so the question can be asked directly.
    '''
    moved = monitor_query(port, 'sysbus.edma0 BytesMoved')
    beats = monitor_query(port, 'sysbus.edma0 BeatsPerChannel')
    if moved is None:
        print('could not read sysbus.edma0 BytesMoved from the monitor - is '
              'this a platform with AP_IMXRT_EDMA.cs in it?')
        return 1
    try:
        total = int(moved, 0) if moved.startswith('0x') else int(moved, 16)
    except ValueError:
        print('unparsable BytesMoved from the monitor: %r' % moved)
        return 1
    if total == 0:
        print('eDMA moved 0 bytes: %s' % (beats or 'no per-channel detail'))
        print('the guest booted on the interrupt path, not the DMA path - '
              'check for a "DMA pool exhausted" line on the console, and that '
              'CONFIG_UART_ASYNC_API is not being forced off')
        return 1
    print('eDMA moved %d bytes (%s)' % (total, beats or 'no per-channel detail'))
    return 0


def monitor_quit(port, timeout=10.0):
    '''Ask Renode to quit through the monitor rather than be killed.

    A persistent SdCardFromFile image is only guaranteed to reach its backing
    file on a clean shutdown; the SIGTERM in the finally block below does not
    give it that chance, and a run whose logs never appear on the card then
    looks like a card that does not work.'''
    try:
        sock = socket.create_connection(('127.0.0.1', port), timeout=timeout)
        sock.settimeout(timeout)
        time.sleep(0.5)
        try:
            sock.recv(65536)          # banner and anything queued
        except (socket.timeout, OSError):
            pass
        sock.sendall(b'quit\n')
        time.sleep(2.0)
        sock.close()
    except OSError as error:
        print('monitor quit failed: %s' % error)


def find_heartbeat(buf):
    '''True when buf holds a complete-looking v2 HEARTBEAT frame.'''
    for i in range(len(buf) - 10):
        if buf[i] != MAVLINK2_MAGIC:
            continue
        msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        if msgid == MSGID_HEARTBEAT:
            return True
    return False


MSGID_STATUSTEXT = 253


def statustexts(buf, already):
    '''Yield STATUSTEXT payloads out of a MAVLink v2 stream.

    Same thing the copter mission job surfaces with "vehicle: ..." lines - that
    one works, so this reads the same messages rather than scraping the byte
    soup for anything that looks like text. ArduPilot's boot messages reach a
    GCS as STATUSTEXT, so this is where they actually are.

    `already` is the set of texts printed so far: the heartbeat search keeps a
    rolling window and the same frame can be seen twice.'''
    out = []
    i = 0
    while i < len(buf) - 12:
        if buf[i] != MAVLINK2_MAGIC:
            i += 1
            continue
        payload_len = buf[i + 1]
        msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        frame_end = i + 10 + payload_len + 2
        if frame_end > len(buf):
            break
        if msgid == MSGID_STATUSTEXT:
            payload = buf[i + 10:i + 10 + payload_len]
            # severity is the first byte, then up to 50 bytes of text
            text = payload[1:51].split(b'\x00')[0]
            text = text.decode('utf-8', errors='replace').strip()
            if text and text not in already:
                already.add(text)
                out.append(text)
        i = frame_end
    return out


def log_tail(path, lines=30):
    '''Last few lines of Renode's log, for a failure message.'''
    try:
        with open(path, errors='replace') as stream:
            tail = stream.read().splitlines()[-lines:]
    except OSError:
        return ''
    return '--- Renode output (tail) ---\n' + '\n'.join(tail)


def read_elf_word(path, vaddr):
    '''Read the 32-bit little-endian word at a virtual address in an ELF.

    Only the program headers are needed, so this does not pull in pyelftools.
    '''
    with open(path, 'rb') as elf:
        data = elf.read()
    if data[:4] != b'\x7fELF' or data[4] != 1:
        raise RuntimeError('%s is not a 32-bit ELF' % path)
    e_phoff, = struct.unpack_from('<I', data, 0x1c)
    e_phentsize, e_phnum = struct.unpack_from('<HH', data, 0x2a)
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        p_type, p_offset, p_vaddr, _p_paddr, p_filesz = struct.unpack_from(
            '<IIIII', data, off)
        if p_type != 1:                       # PT_LOAD
            continue
        if p_vaddr <= vaddr < p_vaddr + p_filesz:
            at = p_offset + (vaddr - p_vaddr)
            return struct.unpack_from('<I', data, at)[0]
    raise RuntimeError('0x%08x is not in any loadable segment of %s'
                       % (vaddr, path))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--board', default='CubeOrangeZephyr',
                    help='Zephyr board whose firmware to boot')
    ap.add_argument('--platform', default='CubeOrange',
                    help='ChibiOS board whose Renode platform to use; must be '
                         'the same MCU as --board')
    ap.add_argument('--resc',
                    help='Renode script to run instead of borrowing a ChibiOS '
                         'platform. Needed by boards whose silicon no ChibiOS '
                         'board uses, such as the i.MX RT1176.')
    ap.add_argument('--uart', default='sysbus.lpuart1',
                    help='peripheral to attach --port to when using --resc')
    ap.add_argument('--vector-base', type=lambda v: int(v, 0), default=0x30022000,
                    help='address of the application vector table, for --resc. '
                         'Must match the $vector_base the script expects.')
    ap.add_argument('--linger', type=float, default=0.0,
                    help='after the heartbeat, keep reading the UART for this '
                         'many seconds and print every STATUSTEXT, then quit '
                         'Renode cleanly. A heartbeat alone does not show what '
                         'happened next: sdcard_init() reports over MAVLink and '
                         'AP_Logger complains every ~30 s if it cannot write.')
    ap.add_argument('--elf', help='override the firmware ELF path')
    ap.add_argument('--renode', default='build/renode/renode')
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--port', type=int, default=5762,
                    help='emulated serial port to listen on')
    ap.add_argument('--monitor-port', type=int, default=5811)
    ap.add_argument('--assert-edma', action='store_true',
                    help='after the heartbeat, require that the emulated eDMA '
                         'moved at least one byte (rt1176 only). A heartbeat '
                         'alone does not prove it: the UART driver falls back '
                         'to the interrupt path without failing.')
    args = ap.parse_args()

    elf = args.elf or os.path.join(
        ROOT, 'build', args.board, 'zephyr_build', 'zephyr', 'zephyr.elf')
    if not os.path.isfile(elf):
        print('no firmware at %s - build it first:' % elf)
        print('    ./waf configure --board=%s && ./waf copter' % args.board)
        return 1

    tempdir = tempfile.mkdtemp(prefix='zephyr_boot_check_')
    if args.resc:
        # Straight to Renode: run.py builds its platform out of ChibiOS hwdef,
        # which a Zephyr-only board does not have.
        #
        # One script rather than a chain of -e flags. Renode reports a failing
        # -e only on its monitor, which is a telnet socket here, so a command
        # that does not take is silent and the ones after it still run - the
        # machine ends up assembled but never started.
        #
        # $repo has to be set: inside a .resc "@." is relative to the Renode
        # binary, so the script's own $repo?=@. fallback resolves to
        # build/renode and every include under it fails.
        #
        # No --console either: it takes the monitor for itself, --port then
        # never opens, and there is nothing left to attach the UART socket to.
        boot_resc = os.path.join(tempdir, 'boot.resc')
        with open(boot_resc, 'w') as script:
            # The microSD card, made the way run.py makes it for a ChibiOS
            # board - same helper, same size and geometry, same $sdcard
            # variable, and NOT behind a flag. run.py:1104 creates one for any
            # board whose platform has an SD controller and nobody asks it to;
            # the --resc path simply had no equivalent, which is the whole
            # reason the RT1176 ran with no card and every log write came back
            # FR_NOT_READY - surfaced as
            # "Failed to create log directory /APM/logs : EBUSY".
            #
            # State dir mirrors run.py's: <repo>/renode/<board>, persistent, so
            # the logs written during a run survive it and
            # Tools/renode/extract_logs.py can read them out afterwards. A
            # tempdir would delete exactly the thing the card exists to keep.
            # fat_image.create_image() keeps an existing image and validates
            # its geometry, so re-running does not wipe the previous flight.
            #
            # It is not optional in practice either: the board script's
            # SdCardFromFile line ABORTS THE REST OF THE SCRIPT when $sdcard
            # is @none, so start never runs, the UART socket never opens, and
            # the board is indistinguishable from a hang.
            # The microSD and parameter images, shared with
            # tests/test_physics_flight.py (see zephyr_state.py for why the
            # card is not optional and why the persistence node is a file).
            import zephyr_state
            state_dir = os.path.join(ROOT, 'renode', args.board)
            try:
                sd_image, params_image, params_repl = zephyr_state.prepare(state_dir)
            except (OSError, RuntimeError, ValueError) as error:
                return 'board state: %s' % error

            script.write(
                '$repo = @%s\n'
                '$elf = @%s\n'
                '$vector_base = %#x\n'
                '%s'
                'include @%s\n'
                # Renode leaves SP at zero: nothing here plays the part the
                # bootloader plays on the board, and machine Reset does not
                # load it from the vector table either. Without this the first
                # push lands in unmapped memory and the core aborts a few
                # thousand instructions in.
                'cpu SP %#x\n'
                # The machine the include created is not still selected when
                # the include returns, and connector Connect then fails with
                # "Select active machine first". A failing line aborts the rest
                # of the script, so start never runs and the board sits at zero
                # virtual time looking like a hang.
                'mach set 0\n'
                '%s'
                'emulation CreateServerSocketTerminal %d "bootuart" false\n'
                'connector Connect %s bootuart\n'
                'start\n'
                % (ROOT, elf, args.vector_base,
                   zephyr_state.lines_before_include(sd_image),
                   os.path.join(ROOT, args.resc),
                   read_elf_word(elf, args.vector_base),
                   zephyr_state.lines_after_machine(params_image, params_repl),
                   args.port, args.uart))
        cmd = [args.renode, '--disable-xwt', '--port',
               str(args.monitor_port), boot_resc]
        print('booting %s with %s' % (os.path.basename(elf), args.resc))
    else:
        # --uart-port as well as --port: run.py's --port is its monitor, and
        # its UART lives on a separate --uart-port that defaults to 5762.
        # Without this, asking this script for any other --port connects to a
        # closed socket and reads zero bytes, which is indistinguishable from a
        # board that never booted.
        cmd = [sys.executable, os.path.join(ROOT, 'Tools', 'renode', 'run.py'),
               args.platform, '--elf', elf, '--renode', args.renode,
               '--no-xterm', '--port', str(args.monitor_port),
               '--uart-port', str(args.port), '--exec', 'start']
        print('booting %s on the %s platform'
              % (os.path.basename(elf), args.platform))
    # To a file, never a pipe. Renode logs every access to an address the
    # platform does not model, and a boot produces thousands of those lines.
    # Nothing here reads that pipe until the run is over, so the 64KB buffer
    # fills and Renode blocks on write - the board freezes a few hundred
    # thousand instructions in and looks exactly like a firmware hang.
    renode_log = os.path.join(tempdir, 'renode.log')
    log = open(renode_log, 'w')
    # Own session, so the cleanup below reaches Renode itself. Without this,
    # terminate() stops run.py and leaves its Renode child holding the UART
    # port: the NEXT run of this script connects to the previous emulator,
    # reads its heartbeat within seconds, and exits 0 having measured nothing.
    proc = subprocess.Popen(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=(os.name == 'posix'))
    deadline = time.time() + args.timeout
    buf = b''
    console = b''      # everything, unlike buf which is trimmed to a window
    seen_texts = set()  # STATUSTEXTs already printed, see statustexts()
    try:
        sock = None
        while time.time() < deadline:
            if proc.poll() is not None:
                print('renode exited early with %d' % proc.returncode)
                print(log_tail(renode_log))
                return 1
            if sock is None:
                try:
                    sock = socket.create_connection(('127.0.0.1', args.port),
                                                    timeout=5)
                    sock.settimeout(5)
                except OSError:
                    time.sleep(2)          # machine still being assembled
                    continue
            try:
                chunk = sock.recv(512)
            except socket.timeout:
                continue
            except OSError:
                sock = None
                continue
            if not chunk:
                sock = None
                continue
            buf += chunk
            console += chunk
            for text in statustexts(buf, seen_texts):
                print('vehicle: %s' % text, flush=True)
            if find_heartbeat(buf):
                print('HEARTBEAT after %.0fs, %d bytes'
                      % (args.timeout - (deadline - time.time()), len(buf)))
                rc = 0
                if args.assert_edma:
                    rc = check_edma(args.monitor_port)
                if args.linger > 0:
                    print('lingering %.0fs for STATUSTEXT' % args.linger, flush=True)
                    # ArduPilot sends STATUSTEXT only to a link it has seen a
                    # GCS heartbeat on. Reading alone leaves this link
                    # inactive - 19 bytes in 150 s, and neither
                    # sdcard_init()'s report nor AP_Logger's complaints ever
                    # arrive. So be a GCS: one heartbeat a second.
                    gcs_hb = None
                    try:
                        from pymavlink import mavutil
                        mav = mavutil.mavlink.MAVLink(None, srcSystem=255,
                                                      srcComponent=190)
                        gcs_hb = mav.heartbeat_encode(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0).pack(mav)
                    except (ImportError, AttributeError) as error:   # pymavlink absent or too old: still listen
                        print('no GCS heartbeat (%s); STATUSTEXT may not flow' % error)
                    next_hb = 0.0
                    linger_until = time.time() + args.linger
                    while time.time() < linger_until:
                        if gcs_hb and time.time() >= next_hb:
                            try:
                                sock.sendall(gcs_hb)
                            except OSError:
                                break
                            next_hb = time.time() + 1.0
                        try:
                            chunk = sock.recv(512)
                        except socket.timeout:
                            continue
                        except OSError:
                            break
                        if not chunk:
                            break
                        buf += chunk
                        for text in statustexts(buf, seen_texts):
                            print('vehicle: %s' % text, flush=True)
                        buf = buf[-4096:]
                    monitor_quit(args.monitor_port)
                return rc
            buf = buf[-4096:]
        print('no heartbeat within %.0fs (%d bytes seen)' % (args.timeout, len(buf)))
        texts = statustexts(console, set())
        if texts:
            print('the board did say this before giving up:')
            for text in texts:
                print('vehicle: %s' % text)
        else:
            print('the board sent no STATUSTEXT on %s at all' % args.uart)
        print('--- renode log tail (emulator, not the guest) ---')
        print(log_tail(renode_log))
        return 1
    finally:
        if os.name == 'posix':
            try:
                terminate_process_group(proc)
            except subprocess.TimeoutExpired:
                print('warning: Renode did not stop; it may still hold port %d'
                      % args.port)
        else:
            proc.terminate()
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                proc.kill()
        log.close()
        # Keep the emulator's own log next to the board's persistent state
        # (where sdcard.img lives) before the tempdir goes. It was destroyed on
        # every run, and only ever PRINTED on a timeout - so a boot that
        # succeeded but never mounted its card left no record of what the SD
        # controller was asked to do, and "0 usdhc1 accesses" in the capture
        # read as "the guest never touched it" when it meant "never shown".
        try:
            keep_dir = os.path.join(ROOT, 'renode', args.board)
            os.makedirs(keep_dir, exist_ok=True)
            src = os.path.join(tempdir, 'renode.log')
            if os.path.exists(src):
                dst = os.path.join(keep_dir, 'last_renode.log')
                shutil.copyfile(src, dst)
                print('emulator log kept at %s' % os.path.relpath(dst, ROOT))
        except OSError as error:
            print('could not keep the emulator log: %s' % error)
        shutil.rmtree(tempdir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
