#!/usr/bin/env python3

# flake8: noqa

"""
Script to catch and give backtrace of a HardFault Crash

    Usage:
        python crash_debugger.py <elf_file> --swd-debug --gdb-port <gdb_port> --dump-fileout <dump_fileout>
        python crash_debugger.py <elf_file> --dump-debug --dump-filein <dump_filein>
    Copyright Siddharth Bharat Purohit, CubePilot Pty. Ltd. 2021
    based on http://www.cyrilfougeray.com/2020/07/27/firmware-logs-with-stack-trace.html
    Released under GNU GPL version 3 or later
"""

import sys
import subprocess
import argparse
import os
import shlex
import tempfile
import time
from queue import Queue, Empty
from threading  import Thread
import signal

from crashdump_info import CrashDumpError, describe_trailer, read_dump_info, verify_elf


def gdb_shell_quote(value):
    if os.name == 'nt':
        return subprocess.list2cmdline([value])
    return shlex.quote(value)


def swd_debug(args):
    global spinner, process_cmd
    openocd_proc = None
    try:
        # Get BackTrace
        ON_POSIX = 'posix' in sys.builtin_module_names

        def enqueue_output(out, queue):
            for line in iter(out.readline, b''):
                queue.put(line)
            out.close()
        hardfault_detected = False
        # Check if already in hardfault
        # p = subprocess.Popen(['arm-none-eabi-gdb', '-nx', '--batch',
        #         '-ex', 'target extended-remote {}'.format(args.gdb_port),
        #         '-ex', 'bt',
        #         args.elf_file], stdout=subprocess.PIPE, close_fds=ON_POSIX)
        # q = Queue()
        # t = Thread(target=enqueue_output, args=(p.stdout, q))
        # t.daemon = True # thread dies with the program
        # t.start()

        # print("Checking if already Crashed")
        # while p.poll() is None:
        #     try:
        #         line = q.get(False)
        #         if b"HardFault_Handler" in line:
        #             hardfault_detected = True
        #             break
        #     except Empty:
        #         pass
        #     sys.stdout.write(next(spinner))
        #     sys.stdout.flush()
        #     sys.stdout.write('\b')
        if not hardfault_detected:
            # lets place breakpoint at HardFault_Handler and wait for it to hit
            cmd = ['arm-none-eabi-gdb', '-nx', '--batch',
                    '-ex', 'target extended-remote {}'.format(args.gdb_port),
                    '-ex', 'b *&HardFault_Handler',
                    '-ex', 'continue',
                    '-ex', 'run',
                    args.elf_file]
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, close_fds=ON_POSIX)
            q = Queue()
            t = Thread(target=enqueue_output, args=(p.stdout, q))
            t.daemon = True # thread dies with the program
            t.start()
            print(' '.join(cmd))
            # Wait for HardFault_Handler to hit
            running = False
            while p.poll() is None:
                try:
                    line = q.get(False)
                    # print(line.decode('utf-8'))
                    if b"Breakpoint" in line:
                        time.sleep(1)
                        p.send_signal(signal.SIGINT)
                        running = True
                    if b"HardFault_Handler" in line and running:
                        hardfault_detected = True
                        break
                except Empty:
                    pass
                sys.stdout.write(next(spinner))
                sys.stdout.flush()
                sys.stdout.write('\b')
        if hardfault_detected:
            dir_path = os.path.dirname(os.path.realpath(__file__))
            # generate crash log
            print("Crash detected, retrieving crash info, please be patient...")
            cmd = ['arm-none-eabi-gdb', '-nx', '--batch',
                    '-ex', 'target extended-remote {}'.format(args.gdb_port),
                    '-ex', 'set logging file {}'.format(args.dump_fileout),
                    '-x', os.path.join(dir_path, 'crash_dump.scr'),
                    args.elf_file]
            # We are now storing the stack dump into the file
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, close_fds=ON_POSIX)
            q = Queue()
            t = Thread(target=enqueue_output, args=(p.stdout, q))
            t.daemon = True # thread dies with the program
            t.start()
            print(' '.join(cmd))
            # Wait for HardFault_Handler to hit
            # TODO: a progress bar would be nice here
            while p.poll() is None:
                sys.stdout.write(next(spinner))
                sys.stdout.flush()
                sys.stdout.write('\b')
            print("Crash info retrieved.\n")
            return True
        else:
            print("No crash detected")
            raise KeyboardInterrupt
    except KeyboardInterrupt:
        # kill openocd if running
        if openocd_proc is not None and openocd_proc.poll() is None:
            openocd_proc.kill()
    return False

if __name__ == '__main__':
    global spinner, process_cmd
    parser = argparse.ArgumentParser(description='manipulate parameter defaults in an ArduPilot firmware')

    parser.add_argument('elf_file')
    parser.add_argument('--dump-debug', action='store_true', help='generate stack trace from dump file')
    parser.add_argument('--dump-filein', help='log file to use to generate stack trace')
    parser.add_argument('--swd-debug', action='store_true', help='enable swd debug')
    parser.add_argument('--gdb-port', default=':3333', help='set gdb port')
    parser.add_argument('--dump-fileout', help='filename to dump crash dump')
    parser.add_argument('--threads', action='store_true',
                        help='show saved ChibiOS thread backtraces (requires debug symbols)')

    args = parser.parse_args()

    if not args.swd_debug and not args.dump_debug:
        parser.error('Must enable either --swd-debug or --dump-debug')

    if args.dump_debug and not args.dump_filein:
        parser.error('--dump-debug requires --dump-filein')

    #get directory of the script
    dir_path = os.path.dirname(os.path.realpath(__file__))
    debug_interface_script = str(os.path.join(dir_path, "debug_interface.py"))
    def spinning_cursor():
        while True:
            for cursor in '|/-\\':
                yield cursor

    spinner = spinning_cursor()
    dump_file = None
    temporary_dump_file = None
    if args.swd_debug:
        if args.dump_fileout is None:
            args.dump_fileout = "last_crash_dump_gdb.txt"
        if (swd_debug(args)):
            dump_file = args.dump_fileout
    elif args.dump_debug:
        dump_file = args.dump_filein
        # if the dump file has 0xFF padding (from SD card pre-allocated crash dump),
        # truncate to the actual dump size stored in the last 4 bytes of the last sector
        try:
            info = read_dump_info(dump_file)
            if info.trailer is not None:
                verify_elf(info.trailer, args.elf_file)
                print(describe_trailer(info.trailer) + " (ELF matches)")
            elif info.dump_size is not None:
                print("Warning: legacy crash dump has no firmware identity trailer")
            if info.dump_size is not None:
                with open(dump_file, 'rb') as f:
                    data = f.read(info.dump_size)
                tmp = tempfile.NamedTemporaryFile(suffix='.bin', delete=False)
                tmp.write(data)
                tmp.close()
                print(f"Truncated dump from {os.path.getsize(dump_file)} to {info.dump_size} bytes")
                dump_file = tmp.name
                temporary_dump_file = tmp.name
        except (CrashDumpError, OSError, subprocess.SubprocessError) as error:
            print(f"Crash dump verification failed: {error}", file=sys.stderr)
            sys.exit(1)

    if dump_file is not None:
        print(debug_interface_script)
        print("Processing Crash Dump.\n")
        debug_interface_command = '{} {} --elf {} --dump {}'.format(
            gdb_shell_quote(sys.executable), gdb_shell_quote(debug_interface_script), gdb_shell_quote(args.elf_file),
            gdb_shell_quote(dump_file))
        process_cmd = [
            'arm-none-eabi-gdb', '-nx', '--batch', '--quiet', args.elf_file,
            '-ex', 'set target-charset ASCII',
            '-ex', 'target remote | {}'.format(debug_interface_command),
            '-ex', 'set print pretty on',
        ]
        if args.threads:
            process_cmd.extend([
                '-ex', 'info threads',
                '-ex', 'thread apply all bt 12',
            ])
        else:
            process_cmd.extend(['-ex', 'bt full'])
        process_cmd.extend(['-ex', 'quit'])
        print(shlex.join(process_cmd))
        # We can call GDB and CrashDebug using the command and print the results
        try:
            process = subprocess.Popen(process_cmd, stdout=subprocess.PIPE)
            output, error = process.communicate()
        finally:
            if temporary_dump_file is not None:
                os.unlink(temporary_dump_file)

        print(output.decode("utf-8", errors="replace"))
        print("---------\n")
        line = b""
    else:
        print("No crash detected")
    print("\nExiting!")
