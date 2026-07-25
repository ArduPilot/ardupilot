#!/usr/bin/env python3
#
# Plot CSV files in terminal.
#
# Example:
# ./scripts/plot.py bench.csv -xSIZE -ybench_read -W80 -H17
#
# Copyright (c) 2022, The littlefs authors.
# SPDX-License-Identifier: BSD-3-Clause
#

import bisect
import codecs
import collections as co
import csv
import io
import itertools as it
import math as m
import os
import shlex
import shutil
import time

try:
    import inotify_simple
except ModuleNotFoundError:
    inotify_simple = None


COLORS = [
    '1;34', # bold blue
    '1;31', # bold red
    '1;32', # bold green
    '1;35', # bold purple
    '1;33', # bold yellow
    '1;36', # bold cyan
    '34',   # blue
    '31',   # red
    '32',   # green
    '35',   # purple
    '33',   # yellow
    '36',   # cyan
]

CHARS_DOTS = " .':"
CHARS_BRAILLE = (
    '⠀⢀⡀⣀⠠⢠⡠⣠⠄⢄⡄⣄⠤⢤⡤⣤' '⠐⢐⡐⣐⠰⢰⡰⣰⠔⢔⡔⣔⠴⢴⡴⣴'
    '⠂⢂⡂⣂⠢⢢⡢⣢⠆⢆⡆⣆⠦⢦⡦⣦' '⠒⢒⡒⣒⠲⢲⡲⣲⠖⢖⡖⣖⠶⢶⡶⣶'
    '⠈⢈⡈⣈⠨⢨⡨⣨⠌⢌⡌⣌⠬⢬⡬⣬' '⠘⢘⡘⣘⠸⢸⡸⣸⠜⢜⡜⣜⠼⢼⡼⣼'
    '⠊⢊⡊⣊⠪⢪⡪⣪⠎⢎⡎⣎⠮⢮⡮⣮' '⠚⢚⡚⣚⠺⢺⡺⣺⠞⢞⡞⣞⠾⢾⡾⣾'
    '⠁⢁⡁⣁⠡⢡⡡⣡⠅⢅⡅⣅⠥⢥⡥⣥' '⠑⢑⡑⣑⠱⢱⡱⣱⠕⢕⡕⣕⠵⢵⡵⣵'
    '⠃⢃⡃⣃⠣⢣⡣⣣⠇⢇⡇⣇⠧⢧⡧⣧' '⠓⢓⡓⣓⠳⢳⡳⣳⠗⢗⡗⣗⠷⢷⡷⣷'
    '⠉⢉⡉⣉⠩⢩⡩⣩⠍⢍⡍⣍⠭⢭⡭⣭' '⠙⢙⡙⣙⠹⢹⡹⣹⠝⢝⡝⣝⠽⢽⡽⣽'
    '⠋⢋⡋⣋⠫⢫⡫⣫⠏⢏⡏⣏⠯⢯⡯⣯' '⠛⢛⡛⣛⠻⢻⡻⣻⠟⢟⡟⣟⠿⢿⡿⣿')
CHARS_POINTS_AND_LINES = 'o'

SI_PREFIXES = {
    18:  'E',
    15:  'P',
    12:  'T',
    9:   'G',
    6:   'M',
    3:   'K',
    0:   '',
    -3:  'm',
    -6:  'u',
    -9:  'n',
    -12: 'p',
    -15: 'f',
    -18: 'a',
}

SI2_PREFIXES = {
    60:  'Ei',
    50:  'Pi',
    40:  'Ti',
    30:  'Gi',
    20:  'Mi',
    10:  'Ki',
    0:   '',
    -10: 'mi',
    -20: 'ui',
    -30: 'ni',
    -40: 'pi',
    -50: 'fi',
    -60: 'ai',
}


# format a number to a strict character width using SI prefixes
def si(x, w=4):
    if x == 0:
        return '0'
    # figure out prefix and scale
    #
    # note we adjust this so that 100K = .1M, which has more info
    # per character
    p = 3*int(m.log(abs(x)*10, 10**3))
    p = min(18, max(-18, p))
    # format with enough digits
    s = '%.*f' % (w, abs(x) / (10.0**p))
    s = s.lstrip('0')
    # truncate but only digits that follow the dot
    if '.' in s:
        s = s[:max(s.find('.'), w-(2 if x < 0 else 1))]
        s = s.rstrip('0')
        s = s.rstrip('.')
    return '%s%s%s' % ('-' if x < 0 else '', s, SI_PREFIXES[p])

def si2(x, w=5):
    if x == 0:
        return '0'
    # figure out prefix and scale
    #
    # note we adjust this so that 128Ki = .1Mi, which has more info
    # per character
    p = 10*int(m.log(abs(x)*10, 2**10))
    p = min(30, max(-30, p))
    # format with enough digits
    s = '%.*f' % (w, abs(x) / (2.0**p))
    s = s.lstrip('0')
    # truncate but only digits that follow the dot
    if '.' in s:
        s = s[:max(s.find('.'), w-(3 if x < 0 else 2))]
        s = s.rstrip('0')
        s = s.rstrip('.')
    return '%s%s%s' % ('-' if x < 0 else '', s, SI2_PREFIXES[p])

# parse escape strings
def escape(s):
    return codecs.escape_decode(s.encode('utf8'))[0].decode('utf8')

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


# parse different data representations
def dat(x):
    # allow the first part of an a/b fraction
    if '/' in x:
        x, _ = x.split('/', 1)

    # first try as int
    try:
        return int(x, 0)
    except ValueError:
        pass

    # then try as float
    try:
        return float(x)
        # just don't allow infinity or nan
        if m.isinf(x) or m.isnan(x):
            raise ValueError("invalid dat %r" % x)
    except ValueError:
        pass

    # else give up
    raise ValueError("invalid dat %r" % x)


# a hack log that preserves sign, with a linear region between -1 and 1
def symlog(x):
    if x > 1:
        return m.log(x)+1
    elif x < -1:
        return -m.log(-x)-1
    else:
        return x

class Plot:
    def __init__(self, width, height, *,
            xlim=None,
            ylim=None,
            xlog=False,
            ylog=False,
            braille=False,
            dots=False):
        # scale if we're printing with dots or braille
        self.width = 2*width if braille else width
        self.height = (4*height if braille
            else 2*height if dots
            else height)

        self.xlim = xlim or (0, width)
        self.ylim = ylim or (0, height)
        self.xlog = xlog
        self.ylog = ylog
        self.braille = braille
        self.dots = dots

        self.grid = [('',False)]*(self.width*self.height)

    def scale(self, x, y):
        # scale and clamp
        try:
            if self.xlog:
                x = int(self.width * (
                    (symlog(x)-symlog(self.xlim[0]))
                    / (symlog(self.xlim[1])-symlog(self.xlim[0]))))
            else:
                x = int(self.width * (
                    (x-self.xlim[0])
                    / (self.xlim[1]-self.xlim[0])))
            if self.ylog:
                y = int(self.height * (
                    (symlog(y)-symlog(self.ylim[0]))
                    / (symlog(self.ylim[1])-symlog(self.ylim[0]))))
            else:
                y = int(self.height * (
                    (y-self.ylim[0])
                    / (self.ylim[1]-self.ylim[0])))
        except ZeroDivisionError:
            x = 0
            y = 0
        return x, y

    def point(self, x, y, *,
            color=COLORS[0],
            char=True):
        # scale
        x, y = self.scale(x, y)

        # ignore out of bounds points
        if x >= 0 and x < self.width and y >= 0 and y < self.height:
            self.grid[x + y*self.width] = (color, char)

    def line(self, x1, y1, x2, y2, *,
            color=COLORS[0],
            char=True):
        # scale
        x1, y1 = self.scale(x1, y1)
        x2, y2 = self.scale(x2, y2)

        # incremental error line algorithm
        ex = abs(x2 - x1)
        ey = -abs(y2 - y1)
        dx = +1 if x1 < x2 else -1
        dy = +1 if y1 < y2 else -1
        e = ex + ey

        while True:
            if x1 >= 0 and x1 < self.width and y1 >= 0 and y1 < self.height:
                self.grid[x1 + y1*self.width] = (color, char)
            e2 = 2*e

            if x1 == x2 and y1 == y2:
                break

            if e2 > ey:
                e += ey
                x1 += dx

            if x1 == x2 and y1 == y2:
                break

            if e2 < ex:
                e += ex
                y1 += dy

        if x2 >= 0 and x2 < self.width and y2 >= 0 and y2 < self.height:
            self.grid[x2 + y2*self.width] = (color, char)

    def plot(self, coords, *,
            color=COLORS[0],
            char=True,
            line_char=True):
        # draw lines
        if line_char:
            for (x1, y1), (x2, y2) in zip(coords, coords[1:]):
                if y1 is not None and y2 is not None:
                    self.line(x1, y1, x2, y2,
                        color=color,
                        char=line_char)

        # draw points
        if char and (not line_char or char is not True):
            for x, y in coords:
                if y is not None:
                    self.point(x, y,
                        color=color,
                        char=char)

    def draw(self, row, *,
            color=False):
        # scale if needed
        if self.braille:
            xscale, yscale = 2, 4
        elif self.dots:
            xscale, yscale = 1, 2
        else:
            xscale, yscale = 1, 1

        y = self.height//yscale-1 - row
        row_ = []
        for x in range(self.width//xscale):
            best_f = ''
            best_c = False

            # encode into a byte
            b = 0
            for i in range(xscale*yscale):
                f, c = self.grid[x*xscale+(xscale-1-(i%xscale))
                        + (y*yscale+(i//xscale))*self.width]
                if c:
                    b |= 1 << i

                if f:
                    best_f = f
                if c and c is not True:
                    best_c = c

            # use byte to lookup character
            if b:
                if best_c:
                    c = best_c
                elif self.braille:
                    c = CHARS_BRAILLE[b]
                else:
                    c = CHARS_DOTS[b]
            else:
                c = ' '

            # color?
            if b and color and best_f:
                c = '\x1b[%sm%s\x1b[m' % (best_f, c)

            # draw axis in blank spaces
            if not b:
                if x == 0 and y == 0:
                    c = '+'
                elif x == 0 and y == self.height//yscale-1:
                    c = '^'
                elif x == self.width//xscale-1 and y == 0:
                    c = '>'
                elif x == 0:
                    c = '|'
                elif y == 0:
                    c = '-'

            row_.append(c)

        return ''.join(row_)


def collect(csv_paths, renames=[]):
    # collect results from CSV files
    results = []
    for path in csv_paths:
        try:
            with openio(path) as f:
                reader = csv.DictReader(f, restval='')
                for r in reader:
                    results.append(r)
        except FileNotFoundError:
            pass

    if renames:
        for r in results:
            # make a copy so renames can overlap
            r_ = {}
            for new_k, old_k in renames:
                if old_k in r:
                    r_[new_k] = r[old_k]
            r.update(r_)

    return results

def dataset(results, x=None, y=None, define=[]):
    # organize by 'by', x, and y
    dataset = {}
    i = 0
    for r in results:
        # filter results by matching defines
        if not all(k in r and r[k] in vs for k, vs in define):
            continue

        # find xs
        if x is not None:
            if x not in r:
                continue
            try:
                x_ = dat(r[x])
            except ValueError:
                continue
        else:
            x_ = i
            i += 1

        # find ys
        if y is not None:
            if y not in r:
                continue
            try:
                y_ = dat(r[y])
            except ValueError:
                continue
        else:
            y_ = None

        if y_ is not None:
            dataset[x_] = y_ + dataset.get(x_, 0)
        else:
            dataset[x_] = y_ or dataset.get(x_, None)

    return dataset

def datasets(results, by=None, x=None, y=None, define=[]):
    # filter results by matching defines
    results_ = []
    for r in results:
        if all(k in r and r[k] in vs for k, vs in define):
            results_.append(r)
    results = results_

    # if y not specified, try to guess from data
    if y is None:
        y = co.OrderedDict()
        for r in results:
            for k, v in r.items():
                if (by is None or k not in by) and v.strip():
                    try:
                        dat(v)
                        y[k] = True
                    except ValueError:
                        y[k] = False
        y = list(k for k,v in y.items() if v)

    if by is not None:
        # find all 'by' values
        ks = set()
        for r in results:
            ks.add(tuple(r.get(k, '') for k in by))
        ks = sorted(ks)

    # collect all datasets
    datasets = co.OrderedDict()
    for ks_ in (ks if by is not None else [()]):
        for x_ in (x if x is not None else [None]):
            for y_ in y:
                # hide x/y if there is only one field
                k_x = x_ if len(x or []) > 1 else ''
                k_y = y_ if len(y or []) > 1 or (not ks_ and not k_x) else ''

                datasets[ks_ + (k_x, k_y)] = dataset(
                    results,
                    x_,
                    y_,
                    [(by_, {k_}) for by_, k_ in zip(by, ks_)]
                        if by is not None else [])

    return datasets


# some classes for organizing subplots into a grid
class Subplot:
    def __init__(self, **args):
        self.x = 0
        self.y = 0
        self.xspan = 1
        self.yspan = 1
        self.args = args

class Grid:
    def __init__(self, subplot, width=1.0, height=1.0):
        self.xweights = [width]
        self.yweights = [height]
        self.map = {(0,0): subplot}
        self.subplots = [subplot]

    def __repr__(self):
        return 'Grid(%r, %r)' % (self.xweights, self.yweights)

    @property
    def width(self):
        return len(self.xweights)

    @property
    def height(self):
        return len(self.yweights)

    def __iter__(self):
        return iter(self.subplots)

    def __getitem__(self, i):
        x, y = i
        if x < 0:
            x += len(self.xweights)
        if y < 0:
            y += len(self.yweights)

        return self.map[(x,y)]

    def merge(self, other, dir):
        if dir in ['above', 'below']:
            # first scale the two grids so they line up
            self_xweights = self.xweights
            other_xweights = other.xweights
            self_w = sum(self_xweights)
            other_w = sum(other_xweights)
            ratio = self_w / other_w
            other_xweights = [s*ratio for s in other_xweights]

            # now interleave xweights as needed
            new_xweights = []
            self_map = {}
            other_map = {}
            self_i = 0
            other_i = 0
            self_xweight = (self_xweights[self_i]
                if self_i < len(self_xweights) else m.inf)
            other_xweight = (other_xweights[other_i]
                if other_i < len(other_xweights) else m.inf)
            while self_i < len(self_xweights) and other_i < len(other_xweights):
                if other_xweight - self_xweight > 0.0000001:
                    new_xweights.append(self_xweight)
                    other_xweight -= self_xweight

                    new_i = len(new_xweights)-1
                    for j in range(len(self.yweights)):
                        self_map[(new_i, j)] = self.map[(self_i, j)]
                    for j in range(len(other.yweights)):
                        other_map[(new_i, j)] = other.map[(other_i, j)]
                    for s in other.subplots:
                        if s.x+s.xspan-1 == new_i:
                            s.xspan += 1
                        elif s.x > new_i:
                            s.x += 1

                    self_i += 1
                    self_xweight = (self_xweights[self_i]
                        if self_i < len(self_xweights) else m.inf)
                elif self_xweight - other_xweight > 0.0000001:
                    new_xweights.append(other_xweight)
                    self_xweight -= other_xweight

                    new_i = len(new_xweights)-1
                    for j in range(len(other.yweights)):
                        other_map[(new_i, j)] = other.map[(other_i, j)]
                    for j in range(len(self.yweights)):
                        self_map[(new_i, j)] = self.map[(self_i, j)]
                    for s in self.subplots:
                        if s.x+s.xspan-1 == new_i:
                            s.xspan += 1
                        elif s.x > new_i:
                            s.x += 1

                    other_i += 1
                    other_xweight = (other_xweights[other_i]
                        if other_i < len(other_xweights) else m.inf)
                else:
                    new_xweights.append(self_xweight)

                    new_i = len(new_xweights)-1
                    for j in range(len(self.yweights)):
                        self_map[(new_i, j)] = self.map[(self_i, j)]
                    for j in range(len(other.yweights)):
                        other_map[(new_i, j)] = other.map[(other_i, j)]

                    self_i += 1
                    self_xweight = (self_xweights[self_i]
                        if self_i < len(self_xweights) else m.inf)
                    other_i += 1
                    other_xweight = (other_xweights[other_i]
                        if other_i < len(other_xweights) else m.inf)

            # squish so ratios are preserved
            self_h = sum(self.yweights)
            other_h = sum(other.yweights)
            ratio = (self_h-other_h) / self_h
            self_yweights = [s*ratio for s in self.yweights]

            # finally concatenate the two grids
            if dir == 'above':
                for s in other.subplots:
                    s.y += len(self_yweights)
                self.subplots.extend(other.subplots)

                self.xweights = new_xweights
                self.yweights = self_yweights + other.yweights
                self.map = self_map | {(x, y+len(self_yweights)): s
                    for (x, y), s in other_map.items()}
            else:
                for s in self.subplots:
                    s.y += len(other.yweights)
                self.subplots.extend(other.subplots)

                self.xweights = new_xweights
                self.yweights = other.yweights + self_yweights
                self.map = other_map | {(x, y+len(other.yweights)): s
                    for (x, y), s in self_map.items()}

        if dir in ['right', 'left']:
            # first scale the two grids so they line up
            self_yweights = self.yweights
            other_yweights = other.yweights
            self_h = sum(self_yweights)
            other_h = sum(other_yweights)
            ratio = self_h / other_h
            other_yweights = [s*ratio for s in other_yweights]

            # now interleave yweights as needed
            new_yweights = []
            self_map = {}
            other_map = {}
            self_i = 0
            other_i = 0
            self_yweight = (self_yweights[self_i]
                if self_i < len(self_yweights) else m.inf)
            other_yweight = (other_yweights[other_i]
                if other_i < len(other_yweights) else m.inf)
            while self_i < len(self_yweights) and other_i < len(other_yweights):
                if other_yweight - self_yweight > 0.0000001:
                    new_yweights.append(self_yweight)
                    other_yweight -= self_yweight

                    new_i = len(new_yweights)-1
                    for j in range(len(self.xweights)):
                        self_map[(j, new_i)] = self.map[(j, self_i)]
                    for j in range(len(other.xweights)):
                        other_map[(j, new_i)] = other.map[(j, other_i)]
                    for s in other.subplots:
                        if s.y+s.yspan-1 == new_i:
                            s.yspan += 1
                        elif s.y > new_i:
                            s.y += 1

                    self_i += 1
                    self_yweight = (self_yweights[self_i]
                        if self_i < len(self_yweights) else m.inf)
                elif self_yweight - other_yweight > 0.0000001:
                    new_yweights.append(other_yweight)
                    self_yweight -= other_yweight

                    new_i = len(new_yweights)-1
                    for j in range(len(other.xweights)):
                        other_map[(j, new_i)] = other.map[(j, other_i)]
                    for j in range(len(self.xweights)):
                        self_map[(j, new_i)] = self.map[(j, self_i)]
                    for s in self.subplots:
                        if s.y+s.yspan-1 == new_i:
                            s.yspan += 1
                        elif s.y > new_i:
                            s.y += 1

                    other_i += 1
                    other_yweight = (other_yweights[other_i]
                        if other_i < len(other_yweights) else m.inf)
                else:
                    new_yweights.append(self_yweight)

                    new_i = len(new_yweights)-1
                    for j in range(len(self.xweights)):
                        self_map[(j, new_i)] = self.map[(j, self_i)]
                    for j in range(len(other.xweights)):
                        other_map[(j, new_i)] = other.map[(j, other_i)]

                    self_i += 1
                    self_yweight = (self_yweights[self_i]
                        if self_i < len(self_yweights) else m.inf)
                    other_i += 1
                    other_yweight = (other_yweights[other_i]
                        if other_i < len(other_yweights) else m.inf)

            # squish so ratios are preserved
            self_w = sum(self.xweights)
            other_w = sum(other.xweights)
            ratio = (self_w-other_w) / self_w
            self_xweights = [s*ratio for s in self.xweights]

            # finally concatenate the two grids
            if dir == 'right':
                for s in other.subplots:
                    s.x += len(self_xweights)
                self.subplots.extend(other.subplots)

                self.xweights = self_xweights + other.xweights
                self.yweights = new_yweights
                self.map = self_map | {(x+len(self_xweights), y): s
                    for (x, y), s in other_map.items()}
            else:
                for s in self.subplots:
                    s.x += len(other.xweights)
                self.subplots.extend(other.subplots)

                self.xweights = other.xweights + self_xweights
                self.yweights = new_yweights
                self.map = other_map | {(x+len(other.xweights), y): s
                    for (x, y), s in self_map.items()}
                    

    def scale(self, width, height):
        self.xweights = [s*width for s in self.xweights]
        self.yweights = [s*height for s in self.yweights]

    @classmethod
    def fromargs(cls, width=1.0, height=1.0, *,
            subplots=[],
            **args):
        grid = cls(Subplot(**args))

        for dir, subargs in subplots:
            subgrid = cls.fromargs(
                width=subargs.pop('width',
                    0.5 if dir in ['right', 'left'] else width),
                height=subargs.pop('height',
                    0.5 if dir in ['above', 'below'] else height),
                **subargs)
            grid.merge(subgrid, dir)

        grid.scale(width, height)
        return grid
    

def main(csv_paths, *,
        by=None,
        x=None,
        y=None,
        define=[],
        color=False,
        braille=False,
        colors=None,
        chars=None,
        line_chars=None,
        points=False,
        points_and_lines=False,
        width=None,
        height=None,
        xlim=(None,None),
        ylim=(None,None),
        xlog=False,
        ylog=False,
        x2=False,
        y2=False,
        xunits='',
        yunits='',
        xlabel=None,
        ylabel=None,
        xticklabels=None,
        yticklabels=None,
        title=None,
        legend_right=False,
        legend_above=False,
        legend_below=False,
        subplot={},
        subplots=[],
        cat=False,
        keep_open=False,
        sleep=None,
        **args):
    # figure out what color should be
    if color == 'auto':
        color = sys.stdout.isatty()
    elif color == 'always':
        color = True
    else:
        color = False

    # what colors to use?
    if colors is not None:
        colors_ = colors
    else:
        colors_ = COLORS

    if chars is not None:
        chars_ = chars
    elif points_and_lines:
        chars_ = CHARS_POINTS_AND_LINES
    else:
        chars_ = [True]

    if line_chars is not None:
        line_chars_ = line_chars
    elif points_and_lines or not points:
        line_chars_ = [True]
    else:
        line_chars_ = [False]

    # allow escape codes in labels/titles
    title = escape(title).splitlines() if title is not None else []
    xlabel = escape(xlabel).splitlines() if xlabel is not None else []
    ylabel = escape(ylabel).splitlines() if ylabel is not None else []

    # separate out renames
    renames = list(it.chain.from_iterable(
        ((k, v) for v in vs)
        for k, vs in it.chain(by or [], x or [], y or [])))
    if by is not None:
        by = [k for k, _ in by]
    if x is not None:
        x = [k for k, _ in x]
    if y is not None:
        y = [k for k, _ in y]

    # create a grid of subplots
    grid = Grid.fromargs(
        subplots=subplots + subplot.pop('subplots', []),
        **subplot)

    for s in grid:
        # allow subplot params to override global params
        x2_ = s.args.get('x2', False) or x2
        y2_ = s.args.get('y2', False) or y2
        xunits_ = s.args.get('xunits', xunits)
        yunits_ = s.args.get('yunits', yunits)
        xticklabels_ = s.args.get('xticklabels', xticklabels)
        yticklabels_ = s.args.get('yticklabels', yticklabels)

        # label/titles are handled a bit differently in subplots
        subtitle = s.args.get('title')
        xsublabel = s.args.get('xlabel')
        ysublabel = s.args.get('ylabel')

        # allow escape codes in sublabels/subtitles
        subtitle = (escape(subtitle).splitlines()
            if subtitle is not None else [])
        xsublabel = (escape(xsublabel).splitlines()
            if xsublabel is not None else [])
        ysublabel = (escape(ysublabel).splitlines()
            if ysublabel is not None else [])

        # don't allow >2 ticklabels and render single ticklabels only once
        if xticklabels_ is not None:
            if len(xticklabels_) == 1:
                xticklabels_ = ["", xticklabels_[0]]
            elif len(xticklabels_) > 2:
                xticklabels_ = [xticklabels_[0], xticklabels_[-1]]
        if yticklabels_ is not None:
            if len(yticklabels_) == 1:
                yticklabels_ = ["", yticklabels_[0]]
            elif len(yticklabels_) > 2:
                yticklabels_ = [yticklabels_[0], yticklabels_[-1]]

        s.x2 = x2_
        s.y2 = y2_
        s.xunits = xunits_
        s.yunits = yunits_
        s.xticklabels = xticklabels_
        s.yticklabels = yticklabels_
        s.title = subtitle
        s.xlabel = xsublabel
        s.ylabel = ysublabel

    # preprocess margins so they can be shared
    for s in grid:
        s.xmargin = (
            len(s.ylabel) + (1 if s.ylabel else 0) # fit ysublabel
                + (1 if s.x > 0 else 0),           # space between
            ((5 if s.y2 else 4) + len(s.yunits)    # fit yticklabels
                if s.yticklabels is None
                else max((len(t) for t in s.yticklabels), default=0))
                + (1 if s.yticklabels != [] else 0),
        )
        s.ymargin = (
            len(s.xlabel),                   # fit xsublabel
            1 if s.xticklabels != [] else 0, # fit xticklabels
            len(s.title),                    # fit subtitle
        )

    for s in grid:
        # share margins so everything aligns nicely
        s.xmargin = (
            max(s_.xmargin[0] for s_ in grid if s_.x == s.x),
            max(s_.xmargin[1] for s_ in grid if s_.x == s.x),
        )
        s.ymargin = (
            max(s_.ymargin[0] for s_ in grid if s_.y == s.y),
            max(s_.ymargin[1] for s_ in grid if s_.y == s.y),
            max(s_.ymargin[-1] for s_ in grid if s_.y+s_.yspan == s.y+s.yspan),
        )


    def draw(f):
        def writeln(s=''):
            f.write(s)
            f.write('\n')
        f.writeln = writeln

        # first collect results from CSV files
        results = collect(csv_paths, renames)

        # then extract the requested datasets
        datasets_ = datasets(results, by, x, y, define)

        # figure out colors/chars here so that subplot defines
        # don't change them later, that'd be bad
        datacolors_ = {
            name: colors_[i % len(colors_)]
            for i, name in enumerate(datasets_.keys())}
        datachars_ = {
            name: chars_[i % len(chars_)]
            for i, name in enumerate(datasets_.keys())}
        dataline_chars_ = {
            name: line_chars_[i % len(line_chars_)]
            for i, name in enumerate(datasets_.keys())}

        # build legend?
        legend_width = 0
        if legend_right or legend_above or legend_below:
            legend_ = []
            for i, k in enumerate(datasets_.keys()):
                label = '%s%s' % (
                    '%s ' % chars_[i % len(chars_)]
                        if chars is not None
                        else '%s ' % line_chars_[i % len(line_chars_)]
                        if line_chars is not None
                        else '',
                    ','.join(k_ for k_ in k if k_))

                if label:
                    legend_.append(label)
                    legend_width = max(legend_width, len(label)+1)

        # figure out our canvas size
        if width is None:
            width_ = min(80, shutil.get_terminal_size((80, None))[0])
        elif width:
            width_ = width
        else:
            width_ = shutil.get_terminal_size((80, None))[0]

        if height is None:
            height_ = 17 + len(title) + len(xlabel)
        elif height:
            height_ = height
        else:
            height_ = shutil.get_terminal_size((None,
                17 + len(title) + len(xlabel)))[1]
            # make space for shell prompt
            if not keep_open:
                height_ -= 1

        # carve out space for the xlabel
        height_ -= len(xlabel)
        # carve out space for the ylabel
        width_ -= len(ylabel) + (1 if ylabel else 0)
        # carve out space for title
        height_ -= len(title)

        # carve out space for the legend
        if legend_right and legend_:
            width_ -= legend_width
        if legend_above and legend_:
            legend_cols = len(legend_)
            while True:
                legend_widths = [
                    max(len(l) for l in legend_[i::legend_cols])
                    for i in range(legend_cols)]
                if (legend_cols <= 1
                        or sum(legend_widths)+2*(legend_cols-1)
                            + max(sum(s.xmargin[:2]) for s in grid if s.x == 0)
                            <= width_):
                    break
                legend_cols -= 1
            height_ -= (len(legend_)+legend_cols-1) // legend_cols
        if legend_below and legend_:
            legend_cols = len(legend_)
            while True:
                legend_widths = [
                    max(len(l) for l in legend_[i::legend_cols])
                    for i in range(legend_cols)]
                if (legend_cols <= 1
                        or sum(legend_widths)+2*(legend_cols-1)
                            + max(sum(s.xmargin[:2]) for s in grid if s.x == 0)
                            <= width_):
                    break
                legend_cols -= 1
            height_ -= (len(legend_)+legend_cols-1) // legend_cols

        # figure out the grid dimensions
        #
        # note we floor to give the dimension tweaks the best chance of not
        # exceeding the requested dimensions, this means we usually are less
        # than the requested dimensions by quite a bit when we have many
        # subplots, but it's a tradeoff for a relatively simple implementation
        widths = [m.floor(w*width_) for w in grid.xweights]
        heights = [m.floor(w*height_) for w in grid.yweights]

        # tweak dimensions to allow all plots to have a minimum width,
        # this may force the plot to be larger than the requested dimensions,
        # but that's the best we can do
        for s in grid:
            # fit xunits
            minwidth = sum(s.xmargin) + max(2,
                2*((5 if s.x2 else 4)+len(s.xunits))
                    if s.xticklabels is None
                    else sum(len(t) for t in s.xticklabels))
            # fit yunits
            minheight = sum(s.ymargin) + 2

            i = 0
            while minwidth > sum(widths[s.x:s.x+s.xspan]):
                widths[s.x+i] += 1
                i = (i + 1) % s.xspan

            i = 0
            while minheight > sum(heights[s.y:s.y+s.yspan]):
                heights[s.y+i] += 1
                i = (i + 1) % s.yspan

        width_ = sum(widths)
        height_ = sum(heights)

        # create a plot for each subplot
        for s in grid:
            # allow subplot params to override global params
            define_ = define + s.args.get('define', [])
            xlim_ = s.args.get('xlim', xlim)
            ylim_ = s.args.get('ylim', ylim)
            xlog_ = s.args.get('xlog', False) or xlog
            ylog_ = s.args.get('ylog', False) or ylog

            # allow shortened ranges
            if len(xlim_) == 1:
                xlim_ = (0, xlim_[0])
            if len(ylim_) == 1:
                ylim_ = (0, ylim_[0])

            # data can be constrained by subplot-specific defines,
            # so re-extract for each plot
            subdatasets = datasets(results, by, x, y, define_)

            # find actual xlim/ylim
            xlim_ = (
                xlim_[0] if xlim_[0] is not None
                    else min(it.chain([0], (k
                        for r in subdatasets.values()
                        for k, v in r.items()
                        if v is not None))),
                xlim_[1] if xlim_[1] is not None
                    else max(it.chain([0], (k
                        for r in subdatasets.values()
                        for k, v in r.items()
                        if v is not None))))

            ylim_ = (
                ylim_[0] if ylim_[0] is not None
                    else min(it.chain([0], (v
                        for r in subdatasets.values()
                        for _, v in r.items()
                        if v is not None))),
                ylim_[1] if ylim_[1] is not None
                    else max(it.chain([0], (v
                        for r in subdatasets.values()
                        for _, v in r.items()
                        if v is not None))))

            # find actual width/height
            subwidth = sum(widths[s.x:s.x+s.xspan]) - sum(s.xmargin)
            subheight = sum(heights[s.y:s.y+s.yspan]) - sum(s.ymargin)

            # plot!
            plot = Plot(
                subwidth,
                subheight,
                xlim=xlim_,
                ylim=ylim_,
                xlog=xlog_,
                ylog=ylog_,
                braille=line_chars is None and braille,
                dots=line_chars is None and not braille)

            for name, dataset in subdatasets.items():
                plot.plot(
                    sorted((x,y) for x,y in dataset.items()),
                    color=datacolors_[name],
                    char=datachars_[name],
                    line_char=dataline_chars_[name])

            s.plot = plot
            s.width = subwidth
            s.height = subheight
            s.xlim = xlim_
            s.ylim = ylim_


        # now that everything's plotted, let's render things to the terminal

        # figure out margin
        xmargin = (
            len(ylabel) + (1 if ylabel else 0),
            sum(grid[0,0].xmargin[:2]),
        )
        ymargin = (
            sum(grid[0,0].ymargin[:2]),
            grid[-1,-1].ymargin[-1],
        )

        # draw title?
        for line in title:
            f.writeln('%*s%s' % (
                sum(xmargin[:2]), '',
                line.center(width_-xmargin[1])))

        # draw legend_above?
        if legend_above and legend_:
            for i in range(0, len(legend_), legend_cols):
                f.writeln('%*s%s' % (
                    max(sum(xmargin[:2])
                        + (width_-xmargin[1]
                            - (sum(legend_widths)+2*(legend_cols-1)))
                            // 2,
                        0), '',
                    '  '.join('%s%s%s' % (
                        '\x1b[%sm' % colors_[(i+j) % len(colors_)]
                            if color else '',
                        '%-*s' % (legend_widths[j], legend_[i+j]),
                        '\x1b[m'
                            if color else '')
                        for j in range(min(legend_cols, len(legend_)-i)))))

        for row in range(height_):
            # draw ylabel?
            f.write(
                '%s ' % ''.join(
                    ('%*s%s%*s' % (
                        ymargin[-1], '',
                        line.center(height_-sum(ymargin)),
                        ymargin[0], ''))[row]
                    for line in ylabel)
                if ylabel else '')

            for x_ in range(grid.width):
                # figure out the grid x/y position
                subrow = row
                y_ = len(heights)-1
                while subrow >= heights[y_]:
                    subrow -= heights[y_]
                    y_ -= 1

                s = grid[x_, y_]
                subrow = row - sum(heights[s.y+s.yspan:])

                # header
                if subrow < s.ymargin[-1]:
                    # draw subtitle?
                    if subrow < len(s.title):
                        f.write('%*s%s' % (
                            sum(s.xmargin[:2]), '',
                            s.title[subrow].center(s.width)))
                    else:
                        f.write('%*s%*s' % (
                            sum(s.xmargin[:2]), '',
                            s.width, ''))
                # draw plot?
                elif subrow-s.ymargin[-1] < s.height:
                    subrow = subrow-s.ymargin[-1]

                    # draw ysublabel?
                    f.write('%-*s' % (
                        s.xmargin[0],
                        '%s ' % ''.join(
                                line.center(s.height)[subrow]
                                for line in s.ylabel)
                            if s.ylabel else ''))

                    # draw yunits?
                    if subrow == 0 and s.yticklabels != []:
                        f.write('%*s' % (
                            s.xmargin[1],
                            ((si2 if s.y2 else si)(s.ylim[1]) + s.yunits
                                if s.yticklabels is None
                                else s.yticklabels[1])
                                + ' '))
                    elif subrow == s.height-1 and s.yticklabels != []:
                        f.write('%*s' % (
                            s.xmargin[1],
                            ((si2 if s.y2 else si)(s.ylim[0]) + s.yunits
                                if s.yticklabels is None
                                else s.yticklabels[0])
                                + ' '))
                    else:
                        f.write('%*s' % (
                            s.xmargin[1], ''))

                    # draw plot!
                    f.write(s.plot.draw(subrow, color=color))

                # footer
                else:
                    subrow = subrow-s.ymargin[-1]-s.height

                    # draw xunits?
                    if subrow < (1 if s.xticklabels != [] else 0):
                        f.write('%*s%-*s%*s%*s' % (
                            sum(s.xmargin[:2]), '',
                            (5 if s.x2 else 4) + len(s.xunits)
                                if s.xticklabels is None
                                else len(s.xticklabels[0]),
                            (si2 if s.x2 else si)(s.xlim[0]) + s.xunits
                                if s.xticklabels is None
                                else s.xticklabels[0],
                            s.width - (2*((5 if s.x2 else 4)+len(s.xunits))
                                if s.xticklabels is None
                                else sum(len(t) for t in s.xticklabels)), '',
                            (5 if s.x2 else 4) + len(s.xunits)
                                if s.xticklabels is None
                                else len(s.xticklabels[1]),
                            (si2 if s.x2 else si)(s.xlim[1]) + s.xunits
                                if s.xticklabels is None
                                else s.xticklabels[1]))
                    # draw xsublabel?
                    elif (subrow < s.ymargin[1]
                            or subrow-s.ymargin[1] >= len(s.xlabel)):
                        f.write('%*s%*s' % (
                            sum(s.xmargin[:2]), '',
                            s.width, ''))
                    else:
                        f.write('%*s%s' % (
                            sum(s.xmargin[:2]), '',
                            s.xlabel[subrow-s.ymargin[1]].center(s.width)))

            # draw legend_right?
            if (legend_right and legend_
                    and row >= ymargin[-1]
                    and row-ymargin[-1] < len(legend_)):
                j = row-ymargin[-1]
                f.write(' %s%s%s' % (
                    '\x1b[%sm' % colors_[j % len(colors_)] if color else '',
                    legend_[j],
                    '\x1b[m' if color else ''))

            f.writeln()

        # draw xlabel?
        for line in xlabel:
            f.writeln('%*s%s' % (
                sum(xmargin[:2]), '',
                line.center(width_-xmargin[1])))

        # draw legend below?
        if legend_below and legend_:
            for i in range(0, len(legend_), legend_cols):
                f.writeln('%*s%s' % (
                    max(sum(xmargin[:2])
                        + (width_-xmargin[1]
                            - (sum(legend_widths)+2*(legend_cols-1)))
                            // 2,
                        0), '',
                    '  '.join('%s%s%s' % (
                        '\x1b[%sm' % colors_[(i+j) % len(colors_)]
                            if color else '',
                        '%-*s' % (legend_widths[j], legend_[i+j]),
                        '\x1b[m'
                            if color else '')
                        for j in range(min(legend_cols, len(legend_)-i)))))


    if keep_open:
        try:
            while True:
                if cat:
                    draw(sys.stdout)
                else:
                    ring = LinesIO()
                    draw(ring)
                    ring.draw()

                # try to inotifywait
                if inotify_simple is not None:
                    ptime = time.time()
                    inotifywait(csv_paths)
                    # sleep for a minimum amount of time, this helps issues
                    # around rapidly updating files
                    time.sleep(max(0, (sleep or 0.01) - (time.time()-ptime)))
                else:
                    time.sleep(sleep or 0.1)
        except KeyboardInterrupt:
            pass

        if cat:
            draw(sys.stdout)
        else:
            ring = LinesIO()
            draw(ring)
            ring.draw()
        sys.stdout.write('\n')
    else:
        draw(sys.stdout)


if __name__ == "__main__":
    import sys
    import argparse
    parser = argparse.ArgumentParser(
        description="Plot CSV files in terminal.",
        allow_abbrev=False)
    parser.add_argument(
        'csv_paths',
        nargs='*',
        help="Input *.csv files.")
    parser.add_argument(
        '-b', '--by',
        action='append',
        type=lambda x: (
            lambda k,v=None: (k, v.split(',') if v is not None else ())
            )(*x.split('=', 1)),
        help="Group by this field. Can rename fields with new_name=old_name.")
    parser.add_argument(
        '-x',
        action='append',
        type=lambda x: (
            lambda k,v=None: (k, v.split(',') if v is not None else ())
            )(*x.split('=', 1)),
        help="Field to use for the x-axis. Can rename fields with "
            "new_name=old_name.")
    parser.add_argument(
        '-y',
        action='append',
        type=lambda x: (
            lambda k,v=None: (k, v.split(',') if v is not None else ())
            )(*x.split('=', 1)),
        help="Field to use for the y-axis. Can rename fields with "
            "new_name=old_name.")
    parser.add_argument(
        '-D', '--define',
        type=lambda x: (lambda k,v: (k, set(v.split(','))))(*x.split('=', 1)),
        action='append',
        help="Only include results where this field is this value. May include "
            "comma-separated options.")
    parser.add_argument(
        '--color',
        choices=['never', 'always', 'auto'],
        default='auto',
        help="When to use terminal colors. Defaults to 'auto'.")
    parser.add_argument(
        '-⣿', '--braille',
        action='store_true',
        help="Use 2x4 unicode braille characters. Note that braille characters "
            "sometimes suffer from inconsistent widths.")
    parser.add_argument(
        '-.', '--points',
        action='store_true',
        help="Only draw data points.")
    parser.add_argument(
        '-!', '--points-and-lines',
        action='store_true',
        help="Draw data points and lines.")
    parser.add_argument(
        '--colors',
        type=lambda x: [x.strip() for x in x.split(',')],
        help="Comma-separated colors to use.")
    parser.add_argument(
        '--chars',
        help="Characters to use for points.")
    parser.add_argument(
        '--line-chars',
        help="Characters to use for lines.")
    parser.add_argument(
        '-W', '--width',
        nargs='?',
        type=lambda x: int(x, 0),
        const=0,
        help="Width in columns. 0 uses the terminal width. Defaults to "
            "min(terminal, 80).")
    parser.add_argument(
        '-H', '--height',
        nargs='?',
        type=lambda x: int(x, 0),
        const=0,
        help="Height in rows. 0 uses the terminal height. Defaults to 17.")
    parser.add_argument(
        '-X', '--xlim',
        type=lambda x: tuple(
            dat(x) if x.strip() else None
            for x in x.split(',')),
        help="Range for the x-axis.")
    parser.add_argument(
        '-Y', '--ylim',
        type=lambda x: tuple(
            dat(x) if x.strip() else None
            for x in x.split(',')),
        help="Range for the y-axis.")
    parser.add_argument(
        '--xlog',
        action='store_true',
        help="Use a logarithmic x-axis.")
    parser.add_argument(
        '--ylog',
        action='store_true',
        help="Use a logarithmic y-axis.")
    parser.add_argument(
        '--x2',
        action='store_true',
        help="Use base-2 prefixes for the x-axis.")
    parser.add_argument(
        '--y2',
        action='store_true',
        help="Use base-2 prefixes for the y-axis.")
    parser.add_argument(
        '--xunits',
        help="Units for the x-axis.")
    parser.add_argument(
        '--yunits',
        help="Units for the y-axis.")
    parser.add_argument(
        '--xlabel',
        help="Add a label to the x-axis.")
    parser.add_argument(
        '--ylabel',
        help="Add a label to the y-axis.")
    parser.add_argument(
        '--xticklabels',
        type=lambda x:
            [x.strip() for x in x.split(',')]
            if x.strip() else [],
        help="Comma separated xticklabels.")
    parser.add_argument(
        '--yticklabels',
        type=lambda x:
            [x.strip() for x in x.split(',')]
            if x.strip() else [],
        help="Comma separated yticklabels.")
    parser.add_argument(
        '-t', '--title',
        help="Add a title.")
    parser.add_argument(
        '-l', '--legend-right',
        action='store_true',
        help="Place a legend to the right.")
    parser.add_argument(
        '--legend-above',
        action='store_true',
        help="Place a legend above.")
    parser.add_argument(
        '--legend-below',
        action='store_true',
        help="Place a legend below.")
    class AppendSubplot(argparse.Action):
        @staticmethod
        def parse(value):
            import copy
            subparser = copy.deepcopy(parser)
            next(a for a in subparser._actions
                if '--width' in a.option_strings).type = float
            next(a for a in subparser._actions
                if '--height' in a.option_strings).type = float
            return subparser.parse_intermixed_args(shlex.split(value or ""))
        def __call__(self, parser, namespace, value, option):
            if not hasattr(namespace, 'subplots'):
                namespace.subplots = []
            namespace.subplots.append((
                option.split('-')[-1],
                self.__class__.parse(value)))
    parser.add_argument(
        '--subplot-above',
        action=AppendSubplot,
        help="Add subplot above with the same dataset. Takes an arg string to "
            "control the subplot which supports most (but not all) of the "
            "parameters listed here. The relative dimensions of the subplot "
            "can be controlled with -W/-H which now take a percentage.")
    parser.add_argument(
        '--subplot-below',
        action=AppendSubplot,
        help="Add subplot below with the same dataset.")
    parser.add_argument(
        '--subplot-left',
        action=AppendSubplot,
        help="Add subplot left with the same dataset.")
    parser.add_argument(
        '--subplot-right',
        action=AppendSubplot,
        help="Add subplot right with the same dataset.")
    parser.add_argument(
        '--subplot',
        type=AppendSubplot.parse,
        help="Add subplot-specific arguments to the main plot.")
    parser.add_argument(
        '-z', '--cat',
        action='store_true',
        help="Pipe directly to stdout.")
    parser.add_argument(
        '-k', '--keep-open',
        action='store_true',
        help="Continue to open and redraw the CSV files in a loop.")
    parser.add_argument(
        '-s', '--sleep',
        type=float,
        help="Time in seconds to sleep between redraws when running with -k. "
            "Defaults to 0.01.")

    def dictify(ns):
        if hasattr(ns, 'subplots'):
            ns.subplots = [(dir, dictify(subplot_ns))
                for dir, subplot_ns in ns.subplots]
        if ns.subplot is not None:
            ns.subplot = dictify(ns.subplot)
        return {k: v
            for k, v in vars(ns).items()
            if v is not None}

    sys.exit(main(**dictify(parser.parse_intermixed_args())))
