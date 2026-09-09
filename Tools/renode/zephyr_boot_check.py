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
'''

import argparse
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# MAVLink v2 framing: 0xFD, len, incompat, compat, seq, sysid, compid, msgid[3]
MAVLINK2_MAGIC = 0xFD
MSGID_HEARTBEAT = 0


def find_heartbeat(buf):
    '''True when buf holds a complete-looking v2 HEARTBEAT frame.'''
    for i in range(len(buf) - 10):
        if buf[i] != MAVLINK2_MAGIC:
            continue
        msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        if msgid == MSGID_HEARTBEAT:
            return True
    return False


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
    ap.add_argument('--elf', help='override the firmware ELF path')
    ap.add_argument('--renode', default='build/renode/renode')
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--port', type=int, default=5762,
                    help='emulated serial port to listen on')
    ap.add_argument('--monitor-port', type=int, default=5811)
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
            script.write(
                '$repo = @%s\n'
                '$elf = @%s\n'
                '$vector_base = %#x\n'
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
                'emulation CreateServerSocketTerminal %d "bootuart" false\n'
                'connector Connect %s bootuart\n'
                'start\n'
                % (ROOT, elf, args.vector_base,
                   os.path.join(ROOT, args.resc),
                   read_elf_word(elf, args.vector_base),
                   args.port, args.uart))
        cmd = [args.renode, '--disable-xwt', '--port',
               str(args.monitor_port), boot_resc]
        print('booting %s with %s' % (os.path.basename(elf), args.resc))
    else:
        cmd = [sys.executable, os.path.join(ROOT, 'Tools', 'renode', 'run.py'),
               args.platform, '--elf', elf, '--renode', args.renode,
               '--no-xterm', '--port', str(args.monitor_port), '--exec', 'start']
        print('booting %s on the %s platform'
              % (os.path.basename(elf), args.platform))
    # To a file, never a pipe. Renode logs every access to an address the
    # platform does not model, and a boot produces thousands of those lines.
    # Nothing here reads that pipe until the run is over, so the 64KB buffer
    # fills and Renode blocks on write - the board freezes a few hundred
    # thousand instructions in and looks exactly like a firmware hang.
    renode_log = os.path.join(tempdir, 'renode.log')
    log = open(renode_log, 'w')
    proc = subprocess.Popen(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    deadline = time.time() + args.timeout
    buf = b''
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
            if find_heartbeat(buf):
                print('HEARTBEAT after %.0fs, %d bytes'
                      % (args.timeout - (deadline - time.time()), len(buf)))
                return 0
            buf = buf[-4096:]
        print('no heartbeat within %.0fs (%d bytes seen)' % (args.timeout, len(buf)))
        print(log_tail(renode_log))
        return 1
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()
        shutil.rmtree(tempdir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
