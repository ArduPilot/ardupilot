#!/usr/bin/env python3
#
# Plot CSV files with matplotlib.
#
# Example:
# ./scripts/plotmpl.py bench.csv -xSIZE -ybench_read -obench.svg
#
# Copyright (c) 2022, The littlefs authors.
# SPDX-License-Identifier: BSD-3-Clause
#

import codecs
import collections as co
import csv
import io
import itertools as it
import logging
import math as m
import numpy as np
import os
import shlex
import shutil
import time

import matplotlib as mpl
import matplotlib.pyplot as plt

# some nicer colors borrowed from Seaborn
# note these include a non-opaque alpha
COLORS = [
    '#4c72b0bf', # blue
    '#dd8452bf', # orange
    '#55a868bf', # green
    '#c44e52bf', # red
    '#8172b3bf', # purple
    '#937860bf', # brown
    '#da8bc3bf', # pink
    '#8c8c8cbf', # gray
    '#ccb974bf', # yellow
    '#64b5cdbf', # cyan
]
COLORS_DARK = [
    '#a1c9f4bf', # blue
    '#ffb482bf', # orange
    '#8de5a1bf', # green
    '#ff9f9bbf', # red
    '#d0bbffbf', # purple
    '#debb9bbf', # brown
    '#fab0e4bf', # pink
    '#cfcfcfbf', # gray
    '#fffea3bf', # yellow
    '#b9f2f0bf', # cyan
]
ALPHAS = [0.75]
FORMATS = ['-']
FORMATS_POINTS = ['.']
FORMATS_POINTS_AND_LINES = ['.-']

WIDTH = 750
HEIGHT = 350
FONT_SIZE = 11

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


# formatter for matplotlib
def si(x):
    if x == 0:
        return '0'
    # figure out prefix and scale
    p = 3*int(m.log(abs(x), 10**3))
    p = min(18, max(-18, p))
    # format with 3 digits of precision
    s = '%.3f' % (abs(x) / (10.0**p))
    s = s[:3+1]
    # truncate but only digits that follow the dot
    if '.' in s:
        s = s.rstrip('0')
        s = s.rstrip('.')
    return '%s%s%s' % ('-' if x < 0 else '', s, SI_PREFIXES[p])

# formatter for matplotlib
def si2(x):
    if x == 0:
        return '0'
    # figure out prefix and scale
    p = 10*int(m.log(abs(x), 2**10))
    p = min(30, max(-30, p))
    # format with 3 digits of precision
    s = '%.3f' % (abs(x) / (2.0**p))
    s = s[:3+1]
    # truncate but only digits that follow the dot
    if '.' in s:
        s = s.rstrip('0')
        s = s.rstrip('.')
    return '%s%s%s' % ('-' if x < 0 else '', s, SI2_PREFIXES[p])

# parse escape strings
def escape(s):
    return codecs.escape_decode(s.encode('utf8'))[0].decode('utf8')

# we want to use MaxNLocator, but since MaxNLocator forces multiples of 10
# to be an option, we can't really...
class AutoMultipleLocator(mpl.ticker.MultipleLocator):
    def __init__(self, base, nbins=None):
        # note base needs to be floats to avoid integer pow issues
        self.base = float(base)
        self.nbins = nbins
        super().__init__(self.base)

    def __call__(self):
        # find best tick count, conveniently matplotlib has a function for this
        vmin, vmax = self.axis.get_view_interval()
        vmin, vmax = mpl.transforms.nonsingular(vmin, vmax, 1e-12, 1e-13)
        if self.nbins is not None:
            nbins = self.nbins
        else:
            nbins = np.clip(self.axis.get_tick_space(), 1, 9)

        # find the best power, use this as our locator's actual base
        scale = self.base ** (m.ceil(m.log((vmax-vmin) / (nbins+1), self.base)))
        self.set_params(scale)

        return super().__call__()


def openio(path, mode='r', buffering=-1):
    # allow '-' for stdin/stdout
    if path == '-':
        if mode == 'r':
            return os.fdopen(os.dup(sys.stdin.fileno()), mode, buffering)
        else:
            return os.fdopen(os.dup(sys.stdout.fileno()), mode, buffering)
    else:
        return open(path, mode, buffering)


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


def main(csv_paths, output, *,
        svg=False,
        png=False,
        quiet=False,
        by=None,
        x=None,
        y=None,
        define=[],
        points=False,
        points_and_lines=False,
        colors=None,
        formats=None,
        width=WIDTH,
        height=HEIGHT,
        xlim=(None,None),
        ylim=(None,None),
        xlog=False,
        ylog=False,
        x2=False,
        y2=False,
        xticks=None,
        yticks=None,
        xunits=None,
        yunits=None,
        xlabel=None,
        ylabel=None,
        xticklabels=None,
        yticklabels=None,
        title=None,
        legend_right=False,
        legend_above=False,
        legend_below=False,
        dark=False,
        ggplot=False,
        xkcd=False,
        github=False,
        font=None,
        font_size=FONT_SIZE,
        font_color=None,
        foreground=None,
        background=None,
        subplot={},
        subplots=[],
        **args):
    # guess the output format
    if not png and not svg:
        if output.endswith('.png'):
            png = True
        else:
            svg = True

    # some shortcuts for color schemes
    if github:
        ggplot = True
        if font_color is None:
            if dark:
                font_color = '#c9d1d9'
            else:
                font_color = '#24292f'
        if foreground is None:
            if dark:
                foreground = '#343942'
            else:
                foreground = '#eff1f3'
        if background is None:
            if dark:
                background = '#0d1117'
            else:
                background = '#ffffff'

    # what colors/alphas/formats to use?
    if colors is not None:
        colors_ = colors
    elif dark:
        colors_ = COLORS_DARK
    else:
        colors_ = COLORS

    if formats is not None:
        formats_ = formats
    elif points_and_lines:
        formats_ = FORMATS_POINTS_AND_LINES
    elif points:
        formats_ = FORMATS_POINTS
    else:
        formats_ = FORMATS

    if font_color is not None:
        font_color_ = font_color
    elif dark:
        font_color_ = '#ffffff'
    else:
        font_color_ = '#000000'

    if foreground is not None:
        foreground_ = foreground
    elif dark:
        foreground_ = '#333333'
    else:
        foreground_ = '#e5e5e5'

    if background is not None:
        background_ = background
    elif dark:
        background_ = '#000000'
    else:
        background_ = '#ffffff'

    # configure some matplotlib settings
    if xkcd:
        # the font search here prints a bunch of unhelpful warnings
        logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
        plt.xkcd()
        # turn off the white outline, this breaks some things
        plt.rc('path', effects=[])
    if ggplot:
        plt.style.use('ggplot')
        plt.rc('patch', linewidth=0)
        plt.rc('axes', facecolor=foreground_, edgecolor=background_)
        plt.rc('grid', color=background_)
        # fix the the gridlines when ggplot+xkcd
        if xkcd:
            plt.rc('grid', linewidth=1)
            plt.rc('axes.spines', bottom=False, left=False)
    if dark:
        plt.style.use('dark_background')
        plt.rc('savefig', facecolor='auto', edgecolor='auto')
        # fix ggplot when dark
        if ggplot:
            plt.rc('axes',
                facecolor=foreground_,
                edgecolor=background_)
            plt.rc('grid', color=background_)

    if font is not None:
        plt.rc('font', family=font)
    plt.rc('font', size=font_size)
    plt.rc('text', color=font_color_)
    plt.rc('figure',
        titlesize='medium',
        labelsize='small')
    plt.rc('axes',
        titlesize='small',
        labelsize='small',
        labelcolor=font_color_)
    if not ggplot:
        plt.rc('axes', edgecolor=font_color_)
    plt.rc('xtick', labelsize='small', color=font_color_)
    plt.rc('ytick', labelsize='small', color=font_color_)
    plt.rc('legend',
        fontsize='small',
        fancybox=False,
        framealpha=None,
        edgecolor=foreground_,
        borderaxespad=0)
    plt.rc('axes.spines', top=False, right=False)

    plt.rc('figure', facecolor=background_, edgecolor=background_)
    if not ggplot:
        plt.rc('axes', facecolor='#00000000')

    # I think the svg backend just ignores DPI, but seems to use something
    # equivalent to 96, maybe this is the default for SVG rendering?
    plt.rc('figure', dpi=96)

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

    # first collect results from CSV files
    results = collect(csv_paths, renames)

    # then extract the requested datasets
    datasets_ = datasets(results, by, x, y, define)

    # figure out formats/colors here so that subplot defines
    # don't change them later, that'd be bad
    dataformats_ = {
        name: formats_[i % len(formats_)]
        for i, name in enumerate(datasets_.keys())}
    datacolors_ = {
        name: colors_[i % len(colors_)]
        for i, name in enumerate(datasets_.keys())}

    # create a grid of subplots
    grid = Grid.fromargs(
        subplots=subplots + subplot.pop('subplots', []),
        **subplot)

    # create a matplotlib plot
    fig = plt.figure(figsize=(
        width/plt.rcParams['figure.dpi'],
        height/plt.rcParams['figure.dpi']),
        layout='constrained',
        # we need a linewidth to keep xkcd mode happy
        linewidth=8 if xkcd else 0)

    gs = fig.add_gridspec(
        grid.height
            + (1 if legend_above else 0)
            + (1 if legend_below else 0),
        grid.width
            + (1 if legend_right else 0),
        height_ratios=([0.001] if legend_above else [])
            + [max(s, 0.01) for s in reversed(grid.yweights)]
            + ([0.001] if legend_below else []),
        width_ratios=[max(s, 0.01) for s in grid.xweights]
            + ([0.001] if legend_right else []))

    # first create axes so that plots can interact with each other
    for s in grid:
        s.ax = fig.add_subplot(gs[
            grid.height-(s.y+s.yspan) + (1 if legend_above else 0)
                : grid.height-s.y + (1 if legend_above else 0),
            s.x
                : s.x+s.xspan])

    # now plot each subplot
    for s in grid:
        # allow subplot params to override global params
        define_ = define + s.args.get('define', [])
        xlim_ = s.args.get('xlim', xlim)
        ylim_ = s.args.get('ylim', ylim)
        xlog_ = s.args.get('xlog', False) or xlog
        ylog_ = s.args.get('ylog', False) or ylog
        x2_ = s.args.get('x2', False) or x2
        y2_ = s.args.get('y2', False) or y2
        xticks_ = s.args.get('xticks', xticks)
        yticks_ = s.args.get('yticks', yticks)
        xunits_ = s.args.get('xunits', xunits)
        yunits_ = s.args.get('yunits', yunits)
        xticklabels_ = s.args.get('xticklabels', xticklabels)
        yticklabels_ = s.args.get('yticklabels', yticklabels)

        # label/titles are handled a bit differently in subplots
        subtitle = s.args.get('title')
        xsublabel = s.args.get('xlabel')
        ysublabel = s.args.get('ylabel')

        # allow shortened ranges
        if len(xlim_) == 1:
            xlim_ = (0, xlim_[0])
        if len(ylim_) == 1:
            ylim_ = (0, ylim_[0])

        # data can be constrained by subplot-specific defines,
        # so re-extract for each plot
        subdatasets = datasets(results, by, x, y, define_)

        # plot!
        ax = s.ax
        for name, dataset in subdatasets.items():
            dats = sorted((x,y) for x,y in dataset.items())
            ax.plot([x for x,_ in dats], [y for _,y in dats],
                dataformats_[name],
                color=datacolors_[name],
                label=','.join(k for k in name if k))

        # axes scaling
        if xlog_:
            ax.set_xscale('symlog')
            ax.xaxis.set_minor_locator(mpl.ticker.NullLocator())
        if ylog_:
            ax.set_yscale('symlog')
            ax.yaxis.set_minor_locator(mpl.ticker.NullLocator())
        # axes limits
        ax.set_xlim(
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
        ax.set_ylim(
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
        # axes ticks
        if x2_:
            ax.xaxis.set_major_formatter(lambda x, pos:
                si2(x)+(xunits_ if xunits_ else ''))
            if xticklabels_ is not None:
                ax.xaxis.set_ticklabels(xticklabels_)
            if xticks_ is None:
                ax.xaxis.set_major_locator(AutoMultipleLocator(2))
            elif isinstance(xticks_, list):
                ax.xaxis.set_major_locator(mpl.ticker.FixedLocator(xticks_))
            elif xticks_ != 0:
                ax.xaxis.set_major_locator(AutoMultipleLocator(2, xticks_-1))
            else:
                ax.xaxis.set_major_locator(mpl.ticker.NullLocator())
        else:
            ax.xaxis.set_major_formatter(lambda x, pos:
                si(x)+(xunits_ if xunits_ else ''))
            if xticklabels_ is not None:
                ax.xaxis.set_ticklabels(xticklabels_)
            if xticks_ is None:
                ax.xaxis.set_major_locator(mpl.ticker.AutoLocator())
            elif isinstance(xticks_, list):
                ax.xaxis.set_major_locator(mpl.ticker.FixedLocator(xticks_))
            elif xticks_ != 0:
                ax.xaxis.set_major_locator(mpl.ticker.MaxNLocator(xticks_-1))
            else:
                ax.xaxis.set_major_locator(mpl.ticker.NullLocator())
        if y2_:
            ax.yaxis.set_major_formatter(lambda x, pos:
                si2(x)+(yunits_ if yunits_ else ''))
            if yticklabels_ is not None:
                ax.yaxis.set_ticklabels(yticklabels_)
            if yticks_ is None:
                ax.yaxis.set_major_locator(AutoMultipleLocator(2))
            elif isinstance(yticks_, list):
                ax.yaxis.set_major_locator(mpl.ticker.FixedLocator(yticks_))
            elif yticks_ != 0:
                ax.yaxis.set_major_locator(AutoMultipleLocator(2, yticks_-1))
            else:
                ax.yaxis.set_major_locator(mpl.ticker.NullLocator())
        else:
            ax.yaxis.set_major_formatter(lambda x, pos:
                si(x)+(yunits_ if yunits_ else ''))
            if yticklabels_ is not None:
                ax.yaxis.set_ticklabels(yticklabels_)
            if yticks_ is None:
                ax.yaxis.set_major_locator(mpl.ticker.AutoLocator())
            elif isinstance(yticks_, list):
                ax.yaxis.set_major_locator(mpl.ticker.FixedLocator(yticks_))
            elif yticks_ != 0:
                ax.yaxis.set_major_locator(mpl.ticker.MaxNLocator(yticks_-1))
            else:
                ax.yaxis.set_major_locator(mpl.ticker.NullLocator())
        if ggplot:
            ax.grid(sketch_params=None)

        # axes subplot labels
        if xsublabel is not None:
            ax.set_xlabel(escape(xsublabel))
        if ysublabel is not None:
            ax.set_ylabel(escape(ysublabel))
        if subtitle is not None:
            ax.set_title(escape(subtitle))

    # add a legend? a bit tricky with matplotlib
    #
    # the best solution I've found is a dedicated, invisible axes for the
    # legend, hacky, but it works.
    #
    # note this was written before constrained_layout supported legend
    # collisions, hopefully this is added in the future
    labels = co.OrderedDict()
    for s in grid:
        for h, l in zip(*s.ax.get_legend_handles_labels()):
            labels[l] = h

    if legend_right:
        ax = fig.add_subplot(gs[(1 if legend_above else 0):,-1])
        ax.set_axis_off()
        ax.legend(
            labels.values(),
            labels.keys(),
            loc='upper left',
            fancybox=False,
            borderaxespad=0)

    if legend_above:
        ax = fig.add_subplot(gs[0, :grid.width])
        ax.set_axis_off()

        # try different column counts until we fit in the axes
        for ncol in reversed(range(1, len(labels)+1)):
            legend_ = ax.legend(
                labels.values(),
                labels.keys(),
                loc='upper center',
                ncol=ncol,
                fancybox=False,
                borderaxespad=0)

            if (legend_.get_window_extent().width
                    <= ax.get_window_extent().width):
                break

    if legend_below:
        ax = fig.add_subplot(gs[-1, :grid.width])
        ax.set_axis_off()

        # big hack to get xlabel above the legend! but hey this
        # works really well actually
        if xlabel:
            ax.set_title(escape(xlabel),
                size=plt.rcParams['axes.labelsize'],
                weight=plt.rcParams['axes.labelweight'])

        # try different column counts until we fit in the axes
        for ncol in reversed(range(1, len(labels)+1)):
            legend_ = ax.legend(
                labels.values(),
                labels.keys(),
                loc='upper center',
                ncol=ncol,
                fancybox=False,
                borderaxespad=0)

            if (legend_.get_window_extent().width
                    <= ax.get_window_extent().width):
                break


    # axes labels, NOTE we reposition these below
    if xlabel is not None and not legend_below:
        fig.supxlabel(escape(xlabel))
    if ylabel is not None:
        fig.supylabel(escape(ylabel))
    if title is not None:
        fig.suptitle(escape(title))

    # precompute constrained layout and find midpoints to adjust things
    # that should be centered so they are actually centered
    fig.canvas.draw()
    xmid = (grid[0,0].ax.get_position().x0 + grid[-1,0].ax.get_position().x1)/2
    ymid = (grid[0,0].ax.get_position().y0 + grid[0,-1].ax.get_position().y1)/2

    if xlabel is not None and not legend_below:
        fig.supxlabel(escape(xlabel), x=xmid)
    if ylabel is not None:
        fig.supylabel(escape(ylabel), y=ymid)
    if title is not None:
        fig.suptitle(escape(title), x=xmid)


    # write the figure!
    plt.savefig(output, format='png' if png else 'svg')

    # some stats
    if not quiet:
        print('updated %s, %s datasets, %s points' % (
            output,
            len(datasets_),
            sum(len(dataset) for dataset in datasets_.values())))


if __name__ == "__main__":
    import sys
    import argparse
    parser = argparse.ArgumentParser(
        description="Plot CSV files with matplotlib.",
        allow_abbrev=False)
    parser.add_argument(
        'csv_paths',
        nargs='*',
        help="Input *.csv files.")
    output_rule = parser.add_argument(
        '-o', '--output',
        required=True,
        help="Output *.svg/*.png file.")
    parser.add_argument(
        '--svg',
        action='store_true',
        help="Output an svg file. By default this is infered.")
    parser.add_argument(
        '--png',
        action='store_true',
        help="Output a png file. By default this is infered.")
    parser.add_argument(
        '-q', '--quiet',
        action='store_true',
        help="Don't print info.")
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
        help="Comma-separated hex colors to use.")
    parser.add_argument(
        '--formats',
        type=lambda x: [x.strip().replace('0',',') for x in x.split(',')],
        help="Comma-separated matplotlib formats to use. Allows '0' as an "
            "alternative for ','.")
    parser.add_argument(
        '-W', '--width',
        type=lambda x: int(x, 0),
        help="Width in pixels. Defaults to %r." % WIDTH)
    parser.add_argument(
        '-H', '--height',
        type=lambda x: int(x, 0),
        help="Height in pixels. Defaults to %r." % HEIGHT)
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
        '--xticks',
        type=lambda x: int(x, 0) if ',' not in x
            else [dat(x) for x in x.split(',')],
        help="Ticks for the x-axis. This can be explicit comma-separated "
            "ticks, the number of ticks, or 0 to disable.")
    parser.add_argument(
        '--yticks',
        type=lambda x: int(x, 0) if ',' not in x
            else [dat(x) for x in x.split(',')],
        help="Ticks for the y-axis. This can be explicit comma-separated "
            "ticks, the number of ticks, or 0 to disable.")
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
    parser.add_argument(
        '--dark',
        action='store_true',
        help="Use the dark style.")
    parser.add_argument(
        '--ggplot',
        action='store_true',
        help="Use the ggplot style.")
    parser.add_argument(
        '--xkcd',
        action='store_true',
        help="Use the xkcd style.")
    parser.add_argument(
        '--github',
        action='store_true',
        help="Use the ggplot style with GitHub colors.")
    parser.add_argument(
        '--font',
        type=lambda x: [x.strip() for x in x.split(',')],
        help="Font family for matplotlib.")
    parser.add_argument(
        '--font-size',
        help="Font size for matplotlib. Defaults to %r." % FONT_SIZE)
    parser.add_argument(
        '--font-color',
        help="Color for the font and other line elements.")
    parser.add_argument(
        '--foreground',
        help="Foreground color to use.")
    parser.add_argument(
        '--background',
        help="Background color to use.")
    class AppendSubplot(argparse.Action):
        @staticmethod
        def parse(value):
            import copy
            subparser = copy.deepcopy(parser)
            next(a for a in subparser._actions
                if '--output' in a.option_strings).required = False
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
