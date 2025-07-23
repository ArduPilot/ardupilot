#!/usr/bin/env python3

# flake8: noqa

"""
Script to catch and give backtrace of a HardFault Crash

    Usage:
        python crash_debugger.py <elf_file> --ser-debug --ser-port <serial_port> --dump-fileout <dump_fileout>
        python crash_debugger.py <elf_file> --swd-debug --gdb-port <gdb_port> --dump-fileout <dump_fileout>
        python crash_debugger.py <elf_file> --dump-debug --dump-fileout <dump_fileout>
    Copyright Siddharth Bharat Purohit, CubePilot Pty. Ltd. 2021
    based on http://www.cyrilfougeray.com/2020/07/27/firmware-logs-with-stack-trace.html
    Released under GNU GPL version 3 or later
"""

from serial import Serial
import sys
import subprocess
import argparse
import os
import time
from queue import Queue, Empty
from threading  import Thread
import signal

def serial_debug(args):
    global spinner, process_cmd
    try:
        ser = Serial(args.ser_port, 921600) # Configure speed depending on your config        
        print('Detecting Crash...')
        ser.write("dump_crash_log".encode('utf-8')) # Send a newline to start the dump
        while 1:
            # Read line by line (waiting for '\n')
            sys.stdout.write(next(spinner))
            sys.stdout.flush()
            line = ser.readline()
            sys.stdout.write('\b')

            if not line:
                break
            # When crash is detected
            # Crash dump is added into a temporary file
            # GDB is used to back trace the crash
            if b"Enable logging" in line.strip():
                print("Crash detected, retrieving crash info, please be patient...")
                dump_file = open(args.dump_fileout, 'wb+')

                # We are now storing the stack dump into the file
                ser.write("dump_crash_log".encode('utf-8')) # Send a newline to start the dump
                line = ser.readline()
                dumping = False
                while b"End of dump" not in line.strip():
                    sys.stdout.write(next(spinner))
                    sys.stdout.flush()
                    if b"6343" in line.strip(): # Look for the start of dump
                        dumping = True
                    if dumping:
                        dump_file.write(line)
                    line = ser.readline()
                    sys.stdout.write('\b')

                print("Crash info retrieved.\n")

                dump_file.close()
                return True
    except KeyboardInterrupt:
        ser.close()
    return False

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
    parser.add_argument('--ser-debug', action='store_true', help='enable serial debug')
    parser.add_argument('--ser-port', help='serial port to use')
    parser.add_argument('--dump-debug', action='store_true', help='generate stack trace from dump file')
    parser.add_argument('--dump-filein', help='log file to use to generate stack trace')
    parser.add_argument('--swd-debug', action='store_true', help='enable swd debug')
    parser.add_argument('--gdb-port', default=':3333', help='set gdb port')
    parser.add_argument('--dump-fileout', help='filename to dump crash dump')

    args = parser.parse_args()

    if not args.ser_debug and not args.swd_debug and not args.dump_debug:
        parser.error('Must enable either --ser-debug or --swd-debug')

    if args.ser_debug and not args.ser_port:
        parser.error('--ser-debug requires --port')

    if args.dump_debug and not args.dump_filein:
        parser.error('--dump-debug requires --dump-filein')

    #get directory of the script
    dir_path = os.path.dirname(os.path.realpath(__file__))
    crashdebug_exe = None
    if sys.platform == "linux" or sys.platform == "linux2":
        crashdebug_exe = str(os.path.join(dir_path, "../../modules/CrashDebug/bins/lin64/CrashDebug"))
    elif sys.platform == "darwin":
        crashdebug_exe = str(os.path.join(dir_path, "../../modules/CrashDebug/bins/osx/CrashDebug"))
    elif sys.platform == "win32":
        crashdebug_exe = str(os.path.join(dir_path, "../../modules/CrashDebug/bins/win32/CrashDebug"))
    def spinning_cursor():
        while True:
            for cursor in '|/-\\':
                yield cursor

    spinner = spinning_cursor()
    dump_file = None
    if args.ser_debug:
        if args.dump_fileout is None:
            args.dump_fileout = "last_crash_dump_ser.txt"
        if (serial_debug(args)):
            dump_file = args.dump_fileout
    elif args.swd_debug:
        if args.dump_fileout is None:
            args.dump_fileout = "last_crash_dump_gdb.txt"
        if (swd_debug(args)):
            dump_file = args.dump_fileout
    elif args.dump_debug:
        dump_file = args.dump_filein

    if dump_file is not None:
        print(crashdebug_exe)
        print("Processing Crash Dump.\n")
        process_cmd = "arm-none-eabi-gdb -nx --batch --quiet " + args.elf_file + "  -ex \"set target-charset ASCII\" -ex \"target remote | " + crashdebug_exe + " --elf " + args.elf_file + " --dump " + dump_file + "\" -ex \"set print pretty on\" -ex \"bt full\" -ex \"quit\""
        print(process_cmd)
        # We can call GDB and CrashDebug using the command and print the results
        process = subprocess.Popen(process_cmd, shell=True, stdout=subprocess.PIPE)
        output, error = process.communicate()

        print(output.decode("utf-8"))
        print("---------\n")
        line = b""
    else:
        print("No crash detected")
    print("\nExiting!")
