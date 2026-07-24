#!/usr/bin/env python3

'''
decode the NPTR messages in a log into symbolised backtraces

NPTR is emitted when a read or write to the reserved first 1k of memory is
trapped, which is almost always a null pointer dereference. It carries the
faulting PC, the link register and a heuristic stack backtrace, all of which
are just addresses until they are run through the firmware ELF.

The ELF must be from the same git hash as the firmware which produced the log,
and needs debug info, so build with:

  ./waf configure --board CubeOrange --debug-symbols
  ./waf plane

Tools/scripts/decode_NPTR.py log.bin build/CubeOrange/bin/arduplane

AP_FLAKE8_CLEAN
'''

import optparse
import os
import subprocess
import sys

# heuristic backtrace fields, in the order the fault handler recorded them
BT_FIELDS = ['BT0', 'BT1', 'BT2', 'BT3', 'BT4', 'BT5']


class DecodeNPTR(object):

    def __init__(self, logfile, elf, addr2line):
        self.logfile = logfile
        self.elf = elf
        self.addr2line = addr2line
        self.cache = {}

    def resolve(self, addresses):
        '''symbolise a list of addresses, returning a list of (func, loc)'''
        wanted = [a for a in addresses if a not in self.cache]
        if wanted:
            cmd = [self.addr2line, '-f', '-C', '-i', '-e', self.elf]
            cmd += ['0x%x' % a for a in wanted]
            try:
                out = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
            except FileNotFoundError:
                print("ERROR: %s not found, use --addr2line" % self.addr2line)
                sys.exit(1)
            except subprocess.CalledProcessError as e:
                print("ERROR: %s failed: %s" % (self.addr2line, e))
                sys.exit(1)
            # with -i one address can expand to several function/location
            # pairs, so we cannot simply zip the output with the input. Ask
            # for them one at a time when inlining has occurred
            lines = out.strip('\n').split('\n')
            if len(lines) == 2*len(wanted):
                for i, a in enumerate(wanted):
                    self.cache[a] = [(lines[2*i], lines[2*i+1])]
            else:
                for a in wanted:
                    cmd = [self.addr2line, '-f', '-C', '-i', '-e', self.elf, '0x%x' % a]
                    try:
                        out = subprocess.run(cmd, capture_output=True, text=True,
                                             check=True).stdout
                    except subprocess.CalledProcessError as e:
                        print("ERROR: %s failed: %s" % (self.addr2line, e))
                        sys.exit(1)
                    lines = out.strip('\n').split('\n')
                    self.cache[a] = list(zip(lines[0::2], lines[1::2]))
        return [self.cache[a] for a in addresses]

    def describe(self, addr, is_return):
        '''one line per address, marking unresolved ones rather than hiding them'''
        if addr == 0:
            return None
        # a return address points at the instruction after the call, and has
        # the thumb bit set. Step back into the calling instruction so we
        # report the call site rather than whatever follows it
        lookup = (addr & ~1) - 2 if is_return else addr
        frames = self.resolve([lookup])[0]
        out = []
        for (func, loc) in frames:
            if func == '??' and loc.startswith('??'):
                out.append('0x%08x  <no symbol>' % addr)
            else:
                out.append('0x%08x  %s at %s' % (addr, func, loc))
        return out

    def print_message(self, m):
        print('')
        print('NPTR hit %u  address=0x%x  thread=%s%s' % (
            m.Cnt, m.FA, m.TN, '  LATCHED (trapping now disabled)' if m.Ltch else ''))
        print('  time %.3fs   exc_return=0x%08x   %s' % (
            m.TimeUS*1.0e-6, m.ExcR,
            'thread mode' if m.IPSR == 0 else 'exception %u' % m.IPSR))
        print('  faulting instruction:')
        for line in self.describe(m.PC, False) or []:
            print('    %s' % line)
        lr = self.describe(m.LR, True)
        if lr:
            print('  link register:')
            for line in lr:
                print('    %s' % line)
        print('  heuristic backtrace (stack scan, may contain stale frames):')
        any_bt = False
        for f in BT_FIELDS:
            lines = self.describe(getattr(m, f), True)
            if lines is None:
                continue
            any_bt = True
            for line in lines:
                print('    %s' % line)
        if not any_bt:
            print('    (none recovered)')

    def run(self):
        from pymavlink import mavutil
        if not os.path.exists(self.elf):
            print("ERROR: ELF '%s' not found" % self.elf)
            sys.exit(1)
        mlog = mavutil.mavlink_connection(self.logfile)
        count = 0
        while True:
            m = mlog.recv_match(type='NPTR')
            if m is None:
                break
            count += 1
            self.print_message(m)
        print('')
        if count == 0:
            print('No NPTR messages found in %s' % self.logfile)
        else:
            print('%u NPTR message(s) decoded.' % count)
            print('Addresses which do not resolve are usually stale stack '
                  'words; the scan cannot tell them from real frames.')


if __name__ == '__main__':
    parser = optparse.OptionParser('decode_NPTR.py [options] LOG ELF')
    parser.add_option("--addr2line",
                      default="arm-none-eabi-addr2line",
                      help="addr2line binary to use")
    (opts, args) = parser.parse_args()
    if len(args) != 2:
        parser.print_help()
        sys.exit(1)
    DecodeNPTR(args[0], args[1], opts.addr2line).run()
