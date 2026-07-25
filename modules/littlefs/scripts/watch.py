#!/usr/bin/env python3
#
# Traditional watch command, but with higher resolution updates and a bit
# different options/output format
#
# Example:
# ./scripts/watch.py -s0.1 date
#
# Copyright (c) 2022, The littlefs authors.
# SPDX-License-Identifier: BSD-3-Clause
#

import collections as co
import errno
import fcntl
import io
import os
import pty
import re
import shutil
import struct
import subprocess as sp
import sys
import termios
import time

try:
    import inotify_simple
except ModuleNotFoundError:
    inotify_simple = None


def openio(path, mode='r', buffering=-1):
    # allow '-' for stdin/stdout
    if path == '-':
        if mode == 'r':
            return os.fdopen(os.dup(sys.stdin.fileno()), mode, buffering)
        else:
            return os.fdopen(os.dup(sys.stdout.fileno()), mode, buffering)
    else:
        return open(path, mode, buffering)

def inotifywait(paths):
    # wait for interesting events
    inotify = inotify_simple.INotify()
    flags = (inotify_simple.flags.ATTRIB
        | inotify_simple.flags.CREATE
        | inotify_simple.flags.DELETE
        | inotify_simple.flags.DELETE_SELF
        | inotify_simple.flags.MODIFY
        | inotify_simple.flags.MOVED_FROM
        | inotify_simple.flags.MOVED_TO
        | inotify_simple.flags.MOVE_SELF)

    # recurse into directories
    for path in paths:
        if os.path.isdir(path):
            for dir, _, files in os.walk(path):
                inotify.add_watch(dir, flags)
                for f in files:
                    inotify.add_watch(os.path.join(dir, f), flags)
        else:
            inotify.add_watch(path, flags)

    # wait for event
    inotify.read()

class LinesIO:
    def __init__(self, maxlen=None):
        self.maxlen = maxlen
        self.lines = co.deque(maxlen=maxlen)
        self.tail = io.StringIO()

        # trigger automatic sizing
        if maxlen == 0:
            self.resize(0)

    def write(self, s):
        # note using split here ensures the trailing string has no newline
        lines = s.split('\n')

        if len(lines) > 1 and self.tail.getvalue():
            self.tail.write(lines[0])
            lines[0] = self.tail.getvalue()
            self.tail = io.StringIO()

        self.lines.extend(lines[:-1])

        if lines[-1]:
            self.tail.write(lines[-1])

    def resize(self, maxlen):
        self.maxlen = maxlen
        if maxlen == 0:
            maxlen = shutil.get_terminal_size((80, 5))[1]
        if maxlen != self.lines.maxlen:
            self.lines = co.deque(self.lines, maxlen=maxlen)

    canvas_lines = 1
    def draw(self):
        # did terminal size change?
        if self.maxlen == 0:
            self.resize(0)

        # first thing first, give ourself a canvas
        while LinesIO.canvas_lines < len(self.lines):
            sys.stdout.write('\n')
            LinesIO.canvas_lines += 1

        # clear the bottom of the canvas if we shrink
        shrink = LinesIO.canvas_lines - len(self.lines)
        if shrink > 0:
            for i in range(shrink):
                sys.stdout.write('\r')
                if shrink-1-i > 0:
                    sys.stdout.write('\x1b[%dA' % (shrink-1-i))
                sys.stdout.write('\x1b[K')
                if shrink-1-i > 0:
                    sys.stdout.write('\x1b[%dB' % (shrink-1-i))
            sys.stdout.write('\x1b[%dA' % shrink)
            LinesIO.canvas_lines = len(self.lines)

        for i, line in enumerate(self.lines):
            # move cursor, clear line, disable/reenable line wrapping
            sys.stdout.write('\r')
            if len(self.lines)-1-i > 0:
                sys.stdout.write('\x1b[%dA' % (len(self.lines)-1-i))
            sys.stdout.write('\x1b[K')
            sys.stdout.write('\x1b[?7l')
            sys.stdout.write(line)
            sys.stdout.write('\x1b[?7h')
            if len(self.lines)-1-i > 0:
                sys.stdout.write('\x1b[%dB' % (len(self.lines)-1-i))
        sys.stdout.flush()


def main(command, *,
        lines=0,
        cat=False,
        sleep=None,
        keep_open=False,
        keep_open_paths=None,
        exit_on_error=False):
    returncode = 0
    try:
        while True:
            # reset ring each run
            if cat:
                ring = sys.stdout
            else:
                ring = LinesIO(lines)

            try:
                # run the command under a pseudoterminal 
                mpty, spty = pty.openpty()

                # forward terminal size
                w, h = shutil.get_terminal_size((80, 5))
                if lines:
                    h = lines
                fcntl.ioctl(spty, termios.TIOCSWINSZ,
                    struct.pack('HHHH', h, w, 0, 0))

                proc = sp.Popen(command,
                    stdout=spty,
                    stderr=spty,
                    close_fds=False)
                os.close(spty)
                mpty = os.fdopen(mpty, 'r', 1)

                while True:
                    try:
                        line = mpty.readline()
                    except OSError as e:
                        if e.errno != errno.EIO:
                            raise
                        break
                    if not line:
                        break

                    ring.write(line)
                    if not cat:
                        ring.draw()

                mpty.close()
                proc.wait()
                if exit_on_error and proc.returncode != 0:
                    returncode = proc.returncode
                    break
            except OSError as e:
                if e.errno != errno.ETXTBSY:
                    raise
                pass

            # try to inotifywait
            if keep_open and inotify_simple is not None:
                if keep_open_paths:
                    paths = set(keep_paths)
                else:
                    # guess inotify paths from command
                    paths = set()
                    for p in command:
                        for p in {
                                p,
                                re.sub('^-.', '', p),
                                re.sub('^--[^=]+=', '', p)}:
                            if p and os.path.exists(p):
                                paths.add(p)
                ptime = time.time()
                inotifywait(paths)
                # sleep for a minimum amount of time, this helps issues around
                # rapidly updating files
                time.sleep(max(0, (sleep or 0.1) - (time.time()-ptime)))
            else:
                time.sleep(sleep or 0.1)
    except KeyboardInterrupt:
        pass

    if not cat:
        sys.stdout.write('\n')
    sys.exit(returncode)


if __name__ == "__main__":
    import sys
    import argparse
    parser = argparse.ArgumentParser(
        description="Traditional watch command, but with higher resolution "
            "updates and a bit different options/output format.",
        allow_abbrev=False)
    parser.add_argument(
        'command',
        nargs=argparse.REMAINDER,
        help="Command to run.")
    parser.add_argument(
        '-n', '--lines',
        nargs='?',
        type=lambda x: int(x, 0),
        const=0,
        help="Show this many lines of history. 0 uses the terminal height. "
            "Defaults to 0.")
    parser.add_argument(
        '-z', '--cat',
        action='store_true',
        help="Pipe directly to stdout.")
    parser.add_argument(
        '-s', '--sleep',
        type=float,
        help="Seconds to sleep between runs. Defaults to 0.1.")
    parser.add_argument(
        '-k', '--keep-open',
        action='store_true',
        help="Try to use inotify to wait for changes.")
    parser.add_argument(
        '-K', '--keep-open-path',
        dest='keep_open_paths',
        action='append',
        help="Use this path for inotify. Defaults to guessing.")
    parser.add_argument(
        '-e', '--exit-on-error',
        action='store_true',
        help="Exit on error.")
    sys.exit(main(**{k: v
        for k, v in vars(parser.parse_args()).items()
        if v is not None}))
