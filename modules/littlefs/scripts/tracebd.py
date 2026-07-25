#!/usr/bin/env python3
#
# Display operations on block devices based on trace output
#
# Example:
# ./scripts/tracebd.py trace
#
# Copyright (c) 2022, The littlefs authors.
# SPDX-License-Identifier: BSD-3-Clause
#

import collections as co
import functools as ft
import io
import itertools as it
import math as m
import os
import re
import shutil
import threading as th
import time


CHARS = 'rpe.'
COLORS = ['42', '45', '44', '']

WEAR_CHARS = '0123456789'
WEAR_CHARS_SUBSCRIPTS = '.₁₂₃₄₅₆789'
WEAR_COLORS = ['', '', '', '', '', '', '', '35', '35', '1;31']

CHARS_DOTS = " .':"
COLORS_DOTS = ['32', '35', '34', '']
CHARS_BRAILLE = (
    '⠀⢀⡀⣀⠠⢠⡠⣠⠄⢄⡄⣄⠤⢤⡤⣤' '⠐⢐⡐⣐⠰⢰⡰⣰⠔⢔⡔⣔⠴⢴⡴⣴'
    '⠂⢂⡂⣂⠢⢢⡢⣢⠆⢆⡆⣆⠦⢦⡦⣦' '⠒⢒⡒⣒⠲⢲⡲⣲⠖⢖⡖⣖⠶⢶⡶⣶'
    '⠈⢈⡈⣈⠨⢨⡨⣨⠌⢌⡌⣌⠬⢬⡬⣬' '⠘⢘⡘⣘⠸⢸⡸⣸⠜⢜⡜⣜⠼⢼⡼⣼'
    '⠊⢊⡊⣊⠪⢪⡪⣪⠎⢎⡎⣎⠮⢮⡮⣮' '⠚⢚⡚⣚⠺⢺⡺⣺⠞⢞⡞⣞⠾⢾⡾⣾'
    '⠁⢁⡁⣁⠡⢡⡡⣡⠅⢅⡅⣅⠥⢥⡥⣥' '⠑⢑⡑⣑⠱⢱⡱⣱⠕⢕⡕⣕⠵⢵⡵⣵'
    '⠃⢃⡃⣃⠣⢣⡣⣣⠇⢇⡇⣇⠧⢧⡧⣧' '⠓⢓⡓⣓⠳⢳⡳⣳⠗⢗⡗⣗⠷⢷⡷⣷'
    '⠉⢉⡉⣉⠩⢩⡩⣩⠍⢍⡍⣍⠭⢭⡭⣭' '⠙⢙⡙⣙⠹⢹⡹⣹⠝⢝⡝⣝⠽⢽⡽⣽'
    '⠋⢋⡋⣋⠫⢫⡫⣫⠏⢏⡏⣏⠯⢯⡯⣯' '⠛⢛⡛⣛⠻⢻⡻⣻⠟⢟⡟⣟⠿⢿⡿⣿')


def openio(path, mode='r', buffering=-1):
    # allow '-' for stdin/stdout
    if path == '-':
        if mode == 'r':
            return os.fdopen(os.dup(sys.stdin.fileno()), mode, buffering)
        else:
            return os.fdopen(os.dup(sys.stdout.fileno()), mode, buffering)
    else:
        return open(path, mode, buffering)

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


# space filling Hilbert-curve
#
# note we memoize the last curve since this is a bit expensive
#
@ft.lru_cache(1)
def hilbert_curve(width, height):
    # based on generalized Hilbert curves:
    # https://github.com/jakubcerveny/gilbert
    #
    def hilbert_(x, y, a_x, a_y, b_x, b_y):
        w = abs(a_x+a_y)
        h = abs(b_x+b_y)
        a_dx = -1 if a_x < 0 else +1 if a_x > 0 else 0
        a_dy = -1 if a_y < 0 else +1 if a_y > 0 else 0
        b_dx = -1 if b_x < 0 else +1 if b_x > 0 else 0
        b_dy = -1 if b_y < 0 else +1 if b_y > 0 else 0

        # trivial row
        if h == 1:
            for _ in range(w):
                yield (x,y)
                x, y = x+a_dx, y+a_dy
            return

        # trivial column
        if w == 1:
            for _ in range(h):
                yield (x,y)
                x, y = x+b_dx, y+b_dy
            return

        a_x_, a_y_ = a_x//2, a_y//2
        b_x_, b_y_ = b_x//2, b_y//2
        w_ = abs(a_x_+a_y_)
        h_ = abs(b_x_+b_y_)

        if 2*w > 3*h:
            # prefer even steps
            if w_ % 2 != 0 and w > 2:
                a_x_, a_y_ = a_x_+a_dx, a_y_+a_dy

            # split in two
            yield from hilbert_(x, y, a_x_, a_y_, b_x, b_y)
            yield from hilbert_(x+a_x_, y+a_y_, a_x-a_x_, a_y-a_y_, b_x, b_y)
        else:
            # prefer even steps
            if h_ % 2 != 0 and h > 2:
                b_x_, b_y_ = b_x_+b_dx, b_y_+b_dy

            # split in three
            yield from hilbert_(x, y, b_x_, b_y_, a_x_, a_y_)
            yield from hilbert_(x+b_x_, y+b_y_, a_x, a_y, b_x-b_x_, b_y-b_y_)
            yield from hilbert_(
                x+(a_x-a_dx)+(b_x_-b_dx), y+(a_y-a_dy)+(b_y_-b_dy),
                -b_x_, -b_y_, -(a_x-a_x_), -(a_y-a_y_))

    if width >= height:
        curve = hilbert_(0, 0, +width, 0, 0, +height)
    else:
        curve = hilbert_(0, 0, 0, +height, +width, 0)

    return list(curve)

# space filling Z-curve/Lebesgue-curve
#
# note we memoize the last curve since this is a bit expensive
#
@ft.lru_cache(1)
def lebesgue_curve(width, height):
    # we create a truncated Z-curve by simply filtering out the points
    # that are outside our region
    curve = []
    for i in range(2**(2*m.ceil(m.log2(max(width, height))))):
        # we just operate on binary strings here because it's easier
        b = '{:0{}b}'.format(i, 2*m.ceil(m.log2(i+1)/2))
        x = int(b[1::2], 2) if b[1::2] else 0
        y = int(b[0::2], 2) if b[0::2] else 0
        if x < width and y < height:
            curve.append((x, y))

    return curve


class Block(int):
    __slots__ = ()
    def __new__(cls, state=0, *,
            wear=0,
            readed=False,
            proged=False,
            erased=False):
        return super().__new__(cls,
            state
            | (wear << 3)
            | (1 if readed else 0)
            | (2 if proged else 0)
            | (4 if erased else 0))

    @property
    def wear(self):
        return self >> 3

    @property
    def readed(self):
        return (self & 1) != 0

    @property
    def proged(self):
        return (self & 2) != 0

    @property
    def erased(self):
        return (self & 4) != 0

    def read(self):
        return Block(int(self) | 1)

    def prog(self):
        return Block(int(self) | 2)

    def erase(self):
        return Block((int(self) | 4) + 8)

    def clear(self):
        return Block(int(self) & ~7)

    def __or__(self, other):
        return Block(
            (int(self) | int(other)) & 7,
            wear=max(self.wear, other.wear))

    def worn(self, max_wear, *,
            block_cycles=None,
            wear_chars=None,
            **_):
        if wear_chars is None:
            wear_chars = WEAR_CHARS

        if block_cycles:
            return self.wear / block_cycles
        else:
            return self.wear / max(max_wear, len(wear_chars))

    def draw(self, max_wear, char=None, *,
            read=True,
            prog=True,
            erase=True,
            wear=False,
            block_cycles=None,
            color=True,
            subscripts=False,
            dots=False,
            braille=False,
            chars=None,
            wear_chars=None,
            colors=None,
            wear_colors=None,
            **_):
        # fallback to default chars/colors
        if chars is None:
            chars = CHARS
        if len(chars) < len(CHARS):
            chars = chars + CHARS[len(chars):]

        if colors is None:
            if braille or dots:
                colors = COLORS_DOTS
            else:
                colors = COLORS
        if len(colors) < len(COLORS):
            colors = colors + COLORS[len(colors):]

        if wear_chars is None:
            if subscripts:
                wear_chars = WEAR_CHARS_SUBSCRIPTS
            else:
                wear_chars = WEAR_CHARS

        if wear_colors is None:
            wear_colors = WEAR_COLORS

        # compute char/color
        c = chars[3]
        f = [colors[3]]

        if wear:
            w = min(
                self.worn(
                    max_wear,
                    block_cycles=block_cycles,
                    wear_chars=wear_chars),
                1)

            c = wear_chars[int(w * (len(wear_chars)-1))]
            f.append(wear_colors[int(w * (len(wear_colors)-1))])

        if erase and self.erased:
            c = chars[2]
            f.append(colors[2])
        elif prog and self.proged:
            c = chars[1]
            f.append(colors[1])
        elif read and self.readed:
            c = chars[0]
            f.append(colors[0])

        # override char?
        if char:
            c = char

        # apply colors
        if f and color:
            c = '%s%s\x1b[m' % (
                ''.join('\x1b[%sm' % f_ for f_ in f),
                c)

        return c


class Bd:
    def __init__(self, *,
            size=1,
            count=1,
            width=None,
            height=1,
            blocks=None):
        if width is None:
            width = count

        if blocks is None:
            self.blocks = [Block() for _ in range(width*height)]
        else:
            self.blocks = blocks
        self.size = size
        self.count = count
        self.width = width
        self.height = height

    def _op(self, f, block=None, off=None, size=None):
        if block is None:
            range_ = range(len(self.blocks))
        else:
            if off is None:
                off, size = 0, self.size
            elif size is None:
                off, size = 0, off

            # update our geometry? this will do nothing if we haven't changed
            self.resize(
                size=max(self.size, off+size),
                count=max(self.count, block+1))

            # map to our block space
            start = (block*self.size + off) / (self.size*self.count)
            stop = (block*self.size + off+size) / (self.size*self.count)

            range_ = range(
                m.floor(start*len(self.blocks)),
                m.ceil(stop*len(self.blocks)))

        # apply the op
        for i in range_:
            self.blocks[i] = f(self.blocks[i])

    def read(self, block=None, off=None, size=None):
        self._op(Block.read, block, off, size)

    def prog(self, block=None, off=None, size=None):
        self._op(Block.prog, block, off, size)

    def erase(self, block=None, off=None, size=None):
        self._op(Block.erase, block, off, size)

    def clear(self, block=None, off=None, size=None):
        self._op(Block.clear, block, off, size)

    def copy(self):
        return Bd(
            blocks=self.blocks.copy(),
            size=self.size,
            count=self.count,
            width=self.width,
            height=self.height)

    def resize(self, *,
            size=None,
            count=None,
            width=None,
            height=None):
        size = size if size is not None else self.size
        count = count if count is not None else self.count
        width = width if width is not None else self.width
        height = height if height is not None else self.height

        if (size == self.size
                and count == self.count
                and width == self.width
                and height == self.height):
            return

        # transform our blocks
        blocks = []
        for x in range(width*height):
            # map from new bd space
            start = m.floor(x * (size*count)/(width*height))
            stop = m.ceil((x+1) * (size*count)/(width*height))
            start_block = start // size
            start_off = start % size
            stop_block = stop // size
            stop_off = stop % size
            # map to old bd space
            start = start_block*self.size + start_off
            stop = stop_block*self.size + stop_off
            start = m.floor(start * len(self.blocks)/(self.size*self.count))
            stop = m.ceil(stop * len(self.blocks)/(self.size*self.count))

            # aggregate state
            blocks.append(ft.reduce(
                Block.__or__,
                self.blocks[start:stop],
                Block()))
            
        self.size = size
        self.count = count
        self.width = width
        self.height = height
        self.blocks = blocks

    def draw(self, row, *,
            read=False,
            prog=False,
            erase=False,
            wear=False,
            hilbert=False,
            lebesgue=False,
            dots=False,
            braille=False,
            **args):
        # find max wear?
        max_wear = None
        if wear:
            max_wear = max(b.wear for b in self.blocks)

        # fold via a curve?
        if hilbert:
            grid = [None]*(self.width*self.height)
            for (x,y), b in zip(
                    hilbert_curve(self.width, self.height),
                    self.blocks):
                grid[x + y*self.width] = b
        elif lebesgue:
            grid = [None]*(self.width*self.height)
            for (x,y), b in zip(
                    lebesgue_curve(self.width, self.height),
                    self.blocks):
                grid[x + y*self.width] = b
        else:
            grid = self.blocks

        # need to wait for more trace output before rendering
        #
        # this is sort of a hack that knows the output is going to a terminal
        if (braille and self.height < 4) or (dots and self.height < 2):
            needed_height = 4 if braille else 2

            self.history = getattr(self, 'history', [])
            self.history.append(grid)

            if len(self.history)*self.height < needed_height:
                # skip for now
                return None

            grid = list(it.chain.from_iterable(
                # did we resize?
                it.islice(it.chain(h, it.repeat(Block())),
                    self.width*self.height)
                for h in self.history))
            self.history = []

        line = []
        if braille:
            # encode into a byte
            for x in range(0, self.width, 2):
                byte_b = 0
                best_b = Block()
                for i in range(2*4):
                    b = grid[x+(2-1-(i%2)) + ((row*4)+(4-1-(i//2)))*self.width]
                    best_b |= b
                    if ((read and b.readed)
                            or (prog and b.proged)
                            or (erase and b.erased)
                            or (not read and not prog and not erase
                                and wear and b.worn(max_wear, **args) >= 0.7)):
                        byte_b |= 1 << i

                line.append(best_b.draw(
                    max_wear,
                    CHARS_BRAILLE[byte_b],
                    braille=True,
                    read=read,
                    prog=prog,
                    erase=erase,
                    wear=wear,
                    **args))
        elif dots:
            # encode into a byte
            for x in range(self.width):
                byte_b = 0
                best_b = Block()
                for i in range(2):
                    b = grid[x + ((row*2)+(2-1-i))*self.width]
                    best_b |= b
                    if ((read and b.readed)
                            or (prog and b.proged)
                            or (erase and b.erased)
                            or (not read and not prog and not erase
                                and wear and b.worn(max_wear, **args) >= 0.7)):
                        byte_b |= 1 << i

                line.append(best_b.draw(
                    max_wear,
                    CHARS_DOTS[byte_b],
                    dots=True,
                    read=read,
                    prog=prog,
                    erase=erase,
                    wear=wear,
                    **args))
        else:
            for x in range(self.width):
                line.append(grid[x + row*self.width].draw(
                    max_wear,
                    read=read,
                    prog=prog,
                    erase=erase,
                    wear=wear,
                    **args))

        return ''.join(line)



def main(path='-', *,
        read=False,
        prog=False,
        erase=False,
        wear=False,
        block=(None,None),
        off=(None,None),
        block_size=None,
        block_count=None,
        block_cycles=None,
        reset=False,
        color='auto',
        dots=False,
        braille=False,
        width=None,
        height=None,
        lines=None,
        cat=False,
        hilbert=False,
        lebesgue=False,
        coalesce=None,
        sleep=None,
        keep_open=False,
        **args):
    # figure out what color should be
    if color == 'auto':
        color = sys.stdout.isatty()
    elif color == 'always':
        color = True
    else:
        color = False

    # exclusive wear or read/prog/erase by default
    if not read and not prog and not erase and not wear:
        read = True
        prog = True
        erase = True

    # assume a reasonable lines/height if not specified
    #
    # note that we let height = None if neither hilbert or lebesgue
    # are specified, this is a bit special as the default may be less
    # than one character in height.
    if height is None and (hilbert or lebesgue):
        if lines is not None:
            height = lines
        else:
            height = 5

    if lines is None:
        if height is not None:
            lines = height
        else:
            lines = 5

    # allow ranges for blocks/offs
    block_start = block[0]
    block_stop = block[1] if len(block) > 1 else block[0]+1
    off_start = off[0]
    off_stop = off[1] if len(off) > 1 else off[0]+1

    if block_start is None:
        block_start = 0
    if block_stop is None and block_count is not None:
        block_stop = block_count
    if off_start is None:
        off_start = 0
    if off_stop is None and block_size is not None:
        off_stop = block_size

    # create a block device representation
    bd = Bd()

    def resize(*, size=None, count=None):
        nonlocal bd

        # size may be overriden by cli args
        if block_size is not None:
            size = block_size
        elif off_stop is not None:
            size = off_stop-off_start

        if block_count is not None:
            count = block_count
        elif block_stop is not None:
            count = block_stop-block_start

        # figure out best width/height
        if width is None:
            width_ = min(80, shutil.get_terminal_size((80, 5))[0])
        elif width:
            width_ = width
        else:
            width_ = shutil.get_terminal_size((80, 5))[0]

        if height is None:
            height_ = 0
        elif height:
            height_ = height
        else:
            height_ = shutil.get_terminal_size((80, 5))[1]

        bd.resize(
            size=size,
            count=count,
            # scale if we're printing with dots or braille
            width=2*width_ if braille else width_,
            height=max(1,
                4*height_ if braille
                else 2*height_ if dots
                else height_))
    resize()

    # parse a line of trace output
    pattern = re.compile(
        '^(?P<file>[^:]*):(?P<line>[0-9]+):trace:.*?bd_(?:'
            '(?P<create>create\w*)\('
                '(?:'
                    'block_size=(?P<block_size>\w+)'
                    '|' 'block_count=(?P<block_count>\w+)'
                    '|' '.*?' ')*' '\)'
            '|' '(?P<read>read)\('
                '\s*(?P<read_ctx>\w+)' '\s*,'
                '\s*(?P<read_block>\w+)' '\s*,'
                '\s*(?P<read_off>\w+)' '\s*,'
                '\s*(?P<read_buffer>\w+)' '\s*,'
                '\s*(?P<read_size>\w+)' '\s*\)'
            '|' '(?P<prog>prog)\('
                '\s*(?P<prog_ctx>\w+)' '\s*,'
                '\s*(?P<prog_block>\w+)' '\s*,'
                '\s*(?P<prog_off>\w+)' '\s*,'
                '\s*(?P<prog_buffer>\w+)' '\s*,'
                '\s*(?P<prog_size>\w+)' '\s*\)'
            '|' '(?P<erase>erase)\('
                '\s*(?P<erase_ctx>\w+)' '\s*,'
                '\s*(?P<erase_block>\w+)'
                '\s*\(\s*(?P<erase_size>\w+)\s*\)' '\s*\)'
            '|' '(?P<sync>sync)\('
                '\s*(?P<sync_ctx>\w+)' '\s*\)' ')\s*$')
    def parse(line):
        nonlocal bd

        # string searching is much faster than the regex here, and this
        # actually has a big impact given how much trace output comes
        # through here
        if 'trace' not in line or 'bd' not in line:
            return False
        m = pattern.match(line)
        if not m:
            return False

        if m.group('create'):
            # update our block size/count
            size = int(m.group('block_size'), 0)
            count = int(m.group('block_count'), 0)

            resize(size=size, count=count)
            if reset:
                bd = Bd(
                    size=bd.size,
                    count=bd.count,
                    width=bd.width,
                    height=bd.height)
            return True

        elif m.group('read') and read:
            block = int(m.group('read_block'), 0)
            off = int(m.group('read_off'), 0)
            size = int(m.group('read_size'), 0)

            if block_stop is not None and block >= block_stop:
                return False
            block -= block_start
            if off_stop is not None:
                if off >= off_stop:
                    return False
                size = min(size, off_stop-off)
            off -= off_start

            bd.read(block, off, size)
            return True

        elif m.group('prog') and prog:
            block = int(m.group('prog_block'), 0)
            off = int(m.group('prog_off'), 0)
            size = int(m.group('prog_size'), 0)

            if block_stop is not None and block >= block_stop:
                return False
            block -= block_start
            if off_stop is not None:
                if off >= off_stop:
                    return False
                size = min(size, off_stop-off)
            off -= off_start

            bd.prog(block, off, size)
            return True

        elif m.group('erase') and (erase or wear):
            block = int(m.group('erase_block'), 0)
            size = int(m.group('erase_size'), 0)

            if block_stop is not None and block >= block_stop:
                return False
            block -= block_start
            if off_stop is not None:
                size = min(size, off_stop)
            off = -off_start

            bd.erase(block, off, size)
            return True

        else:
            return False

    # print trace output
    def draw(f):
        def writeln(s=''):
            f.write(s)
            f.write('\n')
        f.writeln = writeln

        # don't forget we've scaled this for braille/dots!
        for row in range(
                m.ceil(bd.height/4) if braille
                else m.ceil(bd.height/2) if dots
                else bd.height):
            line = bd.draw(row,
                read=read,
                prog=prog,
                erase=erase,
                wear=wear,
                block_cycles=block_cycles,
                color=color,
                dots=dots,
                braille=braille,
                hilbert=hilbert,
                lebesgue=lebesgue,
                **args)
            if line:
                f.writeln(line)

        bd.clear()
        resize()


    # read/parse/coalesce operations
    if cat:
        ring = sys.stdout
    else:
        ring = LinesIO(lines)

    # if sleep print in background thread to avoid getting stuck in a read call
    event = th.Event()
    lock = th.Lock()
    if sleep:
        done = False
        def background():
            while not done:
                event.wait()
                event.clear()
                with lock:
                    draw(ring)
                    if not cat:
                        ring.draw()
                time.sleep(sleep or 0.01)
        th.Thread(target=background, daemon=True).start()

    try:
        while True:
            with openio(path) as f:
                changed = 0
                for line in f:
                    with lock:
                        changed += parse(line)

                        # need to redraw?
                        if changed and (not coalesce or changed >= coalesce):
                            if sleep:
                                event.set()
                            else:
                                draw(ring)
                                if not cat:
                                    ring.draw()
                            changed = 0

            if not keep_open:
                break
            # don't just flood open calls
            time.sleep(sleep or 0.1)
    except FileNotFoundError as e:
        print("error: file not found %r" % path)
        sys.exit(-1)
    except KeyboardInterrupt:
        pass

    if sleep:
        done = True
        lock.acquire() # avoids https://bugs.python.org/issue42717
    if not cat:
        sys.stdout.write('\n')


if __name__ == "__main__":
    import sys
    import argparse
    parser = argparse.ArgumentParser(
        description="Display operations on block devices based on "
            "trace output.",
        allow_abbrev=False)
    parser.add_argument(
        'path',
        nargs='?',
        help="Path to read from.")
    parser.add_argument(
        '-r', '--read',
        action='store_true',
        help="Render reads.")
    parser.add_argument(
        '-p', '--prog',
        action='store_true',
        help="Render progs.")
    parser.add_argument(
        '-e', '--erase',
        action='store_true',
        help="Render erases.")
    parser.add_argument(
        '-w', '--wear',
        action='store_true',
        help="Render wear.")
    parser.add_argument(
        '-b', '--block',
        type=lambda x: tuple(
            int(x, 0) if x.strip() else None
            for x in x.split(',')),
        help="Show a specific block or range of blocks.")
    parser.add_argument(
        '-i', '--off',
        type=lambda x: tuple(
            int(x, 0) if x.strip() else None
            for x in x.split(',')),
        help="Show a specific offset or range of offsets.")
    parser.add_argument(
        '-B', '--block-size',
        type=lambda x: int(x, 0),
        help="Assume a specific block size.")
    parser.add_argument(
        '--block-count',
        type=lambda x: int(x, 0),
        help="Assume a specific block count.")
    parser.add_argument(
        '-C', '--block-cycles',
        type=lambda x: int(x, 0),
        help="Assumed maximum number of erase cycles when measuring wear.")
    parser.add_argument(
        '-R', '--reset',
        action='store_true',
        help="Reset wear on block device initialization.")
    parser.add_argument(
        '--color',
        choices=['never', 'always', 'auto'],
        default='auto',
        help="When to use terminal colors. Defaults to 'auto'.")
    parser.add_argument(
        '--subscripts',
        action='store_true',
        help="Use unicode subscripts for showing wear.")
    parser.add_argument(
        '-:', '--dots',
        action='store_true',
        help="Use 1x2 ascii dot characters.")
    parser.add_argument(
        '-⣿', '--braille',
        action='store_true',
        help="Use 2x4 unicode braille characters. Note that braille characters "
            "sometimes suffer from inconsistent widths.")
    parser.add_argument(
        '--chars',
        help="Characters to use for read, prog, erase, noop operations.")
    parser.add_argument(
        '--wear-chars',
        help="Characters to use for showing wear.")
    parser.add_argument(
        '--colors',
        type=lambda x: [x.strip() for x in x.split(',')],
        help="Colors to use for read, prog, erase, noop operations.")
    parser.add_argument(
        '--wear-colors',
        type=lambda x: [x.strip() for x in x.split(',')],
        help="Colors to use for showing wear.")
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
        help="Height in rows. 0 uses the terminal height. Defaults to 1.")
    parser.add_argument(
        '-n', '--lines',
        nargs='?',
        type=lambda x: int(x, 0),
        const=0,
        help="Show this many lines of history. 0 uses the terminal height. "
            "Defaults to 5.")
    parser.add_argument(
        '-z', '--cat',
        action='store_true',
        help="Pipe directly to stdout.")
    parser.add_argument(
        '-U', '--hilbert',
        action='store_true',
        help="Render as a space-filling Hilbert curve.")
    parser.add_argument(
        '-Z', '--lebesgue',
        action='store_true',
        help="Render as a space-filling Z-curve.")
    parser.add_argument(
        '-c', '--coalesce',
        type=lambda x: int(x, 0),
        help="Number of operations to coalesce together.")
    parser.add_argument(
        '-s', '--sleep',
        type=float,
        help="Time in seconds to sleep between reads, coalescing operations.")
    parser.add_argument(
        '-k', '--keep-open',
        action='store_true',
        help="Reopen the pipe on EOF, useful when multiple "
            "processes are writing.")
    sys.exit(main(**{k: v
        for k, v in vars(parser.parse_intermixed_args()).items()
        if v is not None}))
