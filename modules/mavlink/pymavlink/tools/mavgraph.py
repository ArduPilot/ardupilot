#!/usr/bin/env python3
'''
graph a MAVLink log file
Andrew Tridgell August 2011
'''

import datetime
import matplotlib
import os
import re
import sys
import time
from math import *

try:
    from pymavlink.mavextra import *
except:
    print("WARNING: Numpy missing, mathematical notation will not be supported.")

text_types = frozenset([str,])

# cope with rename of raw_input in python3
try:
    input = raw_input
except NameError:
    pass

colourmap = {
    'ardupilot' : {
        'MANUAL'    : (1.0,   0,   0),
        'AUTO'      : (  0, 1.0,   0),
        'LOITER'    : (  0,   0, 1.0),
        'FBWA'      : (1.0, 0.5,   0),
        'RTL'       : (  1,   0, 0.5),
        'STABILIZE' : (0.5, 1.0,   0),
        'LAND'      : (  0, 1.0, 0.5),
        'STEERING'  : (0.5,   0, 1.0),
        'HOLD'      : (  0, 0.5, 1.0),
        'ALT_HOLD'  : (1.0, 0.5, 0.5),
        'CIRCLE'    : (0.5, 1.0, 0.5),
        'POSITION'  : (1.0, 0.0, 1.0),
        'GUIDED'    : (0.5, 0.5, 1.0),
        'ACRO'      : (1.0, 1.0,   0),
        'CRUISE'    : (  0, 1.0, 1.0)
        },
    'px4' : {
        'MANUAL'    : (1.0,   0,   0),
        'SEATBELT'  : (  0.5, 0.5,   0),
        'EASY'      : (  0, 1.0,   0),
        'AUTO'    : (  0,   0, 1.0),
        'UNKNOWN'    : (  1.0,   1.0, 1.0)
        }
    }
colourmap["apm"] = colourmap["ardupilot"]

edge_colour = (0.1, 0.1, 0.1)

lowest_x = None
highest_x = None

def plotit(x, y, fields, colors=[]):
    '''plot a set of graphs using date for x axis'''
    global lowest_x, highest_x
    pylab.ion()
    fig = pylab.figure(num=1, figsize=(12,6))
    ax1 = fig.gca()
    ax2 = None
    xrange = 0.0
    for i in range(0, len(fields)):
        if len(x[i]) == 0: continue
        if lowest_x is None or x[i][0] < lowest_x:
            lowest_x = x[i][0]
        if highest_x is None or x[i][-1] > highest_x:
            highest_x = x[i][-1]
    if highest_x is None or lowest_x is None:
        return
    xrange = highest_x - lowest_x
    xrange *= 24 * 60 * 60
    formatter = matplotlib.dates.DateFormatter('%H:%M:%S')
    interval = 1
    intervals = [ 1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600,
                  900, 1800, 3600, 7200, 5*3600, 10*3600, 24*3600 ]
    for interval in intervals:
        if xrange / interval < 15:
            break
    locator = matplotlib.dates.SecondLocator(interval=interval)
    if not args.xaxis:
        ax1.xaxis.set_major_locator(locator)
        ax1.xaxis.set_major_formatter(formatter)
    empty = True
    ax1_labels = []
    ax2_labels = []
    for i in range(0, len(fields)):
        if len(x[i]) == 0:
            print("Failed to find any values for field %s" % fields[i])
            continue
        if i < len(colors):
            color = colors[i]
        else:
            color = 'red'
        (tz, tzdst) = time.tzname
        if axes[i] == 2:
            if ax2 is None:
                ax2 = ax1.twinx()
            ax = ax2
            if not args.xaxis:
                ax2.xaxis.set_major_locator(locator)
                ax2.xaxis.set_major_formatter(formatter)
            label = fields[i]
            if label.endswith(":2"):
                label = label[:-2]
            ax2_labels.append(label)
        else:
            ax1_labels.append(fields[i])
            ax = ax1
        if args.xaxis:
            if args.marker is not None:
                marker = args.marker
            else:
                marker = '+'
            if args.linestyle is not None:
                linestyle = args.linestyle
            else:
                linestyle = 'None'
            ax.plot(x[i], y[i], color=color, label=fields[i],
                    linestyle=linestyle, marker=marker)
        else:
            if args.marker is not None:
                marker = args.marker
            else:
                marker = 'None'
            if args.linestyle is not None:
                linestyle = args.linestyle
            else:
                linestyle = '-'
            if len(y[i]) > 0 and type(y[i][0]) in text_types:
                # assume this is a piece of text to be rendered at a point in time
                last_text_time = -1
                last_text = None
                for n in range(0, len(x[i])):
                    if last_text is None:
                        last_text = "[" + y[i][n] + "]"
                        last_text_time = x[i][n]
                    elif x[i][n] == last_text_time:
                        last_text += "[" + y[i][n] + "]"
                    else:
                        ax.text(x[i][n], 10, last_text,
                                rotation=90,
                                alpha=0.3,
                                verticalalignment='baseline')
                        last_text = None
                        last_label_time = x[i][n]
                if last_text is not None:
                    ax.text(x[i][n], 10, last_text,
                            rotation=90,
                            alpha=0.3,
                            verticalalignment='baseline')
            else:
                ax.plot_date(x[i], y[i], color=color, label=fields[i],
                             linestyle=linestyle, marker=marker, tz=None)
        empty = False
    if args.flightmode is not None:
        for i in range(len(modes)-1):
            c = colourmap[args.flightmode].get(modes[i][1], edge_colour)
            ax1.axvspan(modes[i][0], modes[i+1][0], fc=c, ec=edge_colour, alpha=0.1)
        c = colourmap[args.flightmode].get(modes[-1][1], edge_colour)
        ax1.axvspan(modes[-1][0], ax1.get_xlim()[1], fc=c, ec=edge_colour, alpha=0.1)
    if ax1_labels != []:
        ax1.legend(ax1_labels,loc=args.legend)
    if ax2_labels != []:
        ax2.legend(ax2_labels,loc=args.legend2)
    if empty:
        print("No data to graph")
        return
    return fig


from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--planner", action='store_true', help="use planner file format")
parser.add_argument("--condition", default=None, help="select packets by a condition")
parser.add_argument("--labels", default=None, help="comma separated field labels")
parser.add_argument("--legend", default='upper left', help="default legend position")
parser.add_argument("--legend2", default='upper right', help="default legend2 position")
parser.add_argument("--marker", default=None, help="point marker")
parser.add_argument("--linestyle", default=None, help="line style")
parser.add_argument("--xaxis", default=None, help="X axis expression")
parser.add_argument("--multi", action='store_true', help="multiple files with same colours")
parser.add_argument("--zero-time-base", action='store_true', help="use Z time base for DF logs")
parser.add_argument("--flightmode", default=None,
                    help="Choose the plot background according to the active flight mode of the specified type, e.g. --flightmode=apm for ArduPilot or --flightmode=px4 for PX4 stack logs.  Cannot be specified with --xaxis.")
parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
parser.add_argument("--output", default=None, help="provide an output format")
parser.add_argument("--timeshift", type=float, default=0, help="shift time on first graph in seconds")
parser.add_argument("logs_fields", metavar="<LOG or FIELD>", nargs="+")
args = parser.parse_args()

from pymavlink import mavutil

if args.flightmode is not None and args.xaxis:
    print("Cannot request flightmode backgrounds with an x-axis expression")
    sys.exit(1)

if args.flightmode is not None and args.flightmode not in colourmap:
    print("Unknown flight controller '%s' in specification of --flightmode (choose from %s)" % (args.flightmode, ",".join(colourmap.keys())))
    sys.exit(1)


if args.output is not None:
    matplotlib.use('Agg')

import pylab

filenames = []
fields = []
for f in args.logs_fields:
    if os.path.exists(f):
        filenames.append(f)
    else:
        fields.append(f)
msg_types = set()
multiplier = []
field_types = []

colors = [ 'red', 'green', 'blue', 'orange', 'olive', 'black', 'grey', 'yellow', 'brown', 'darkcyan', 'cornflowerblue', 'darkmagenta', 'deeppink', 'darkred']

# work out msg types we are interested in
x = []
y = []
modes = []
axes = []
first_only = []
re_caps = re.compile('[A-Z_][A-Z0-9_]+')
for f in fields:
    caps = set(re.findall(re_caps, f))
    msg_types = msg_types.union(caps)
    field_types.append(caps)
    y.append([])
    x.append([])
    axes.append(1)
    first_only.append(False)

def add_data(t, msg, vars, flightmode):
    '''add some data'''
    mtype = msg.get_type()
    if args.flightmode is not None and (len(modes) == 0 or modes[-1][1] != flightmode):
        modes.append((t, flightmode))
    if mtype not in msg_types:
        return
    for i in range(0, len(fields)):
        if mtype not in field_types[i]:
            continue
        f = fields[i]
        if f.endswith(":2"):
            axes[i] = 2
            f = f[:-2]
        if f.endswith(":1"):
            first_only[i] = True
            f = f[:-2]
        v = mavutil.evaluate_expression(f, vars)
        if v is None:
            continue
        if args.xaxis is None:
            xv = t
        else:
            xv = mavutil.evaluate_expression(args.xaxis, vars)
            if xv is None:
                continue
        y[i].append(v)
        x[i].append(xv)

def process_file(filename, timeshift):
    '''process one file'''
    print("Processing %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps, zero_time_base=args.zero_time_base, dialect=args.dialect)
    vars = {}
    all_messages = {}

    while True:
        msg = mlog.recv_match(args.condition)
        if msg is None: break
        try:
            tdays = matplotlib.dates.date2num(datetime.datetime.fromtimestamp(msg._timestamp+timeshift))
        except ValueError:
            # this can happen if the log is corrupt
            # ValueError: year is out of range
            break
        all_messages[msg.get_type()] = msg
        add_data(tdays, msg, all_messages, mlog.flightmode)

if len(filenames) == 0:
    print("No files to process")
    sys.exit(1)

if args.labels is not None:
    labels = args.labels.split(',')
    if len(labels) != len(fields)*len(filenames):
        print("Number of labels (%u) must match number of fields (%u)" % (
            len(labels), len(fields)*len(filenames)))
        sys.exit(1)
else:
    labels = None

timeshift = args.timeshift

for fi in range(0, len(filenames)):
    f = filenames[fi]
    process_file(f, timeshift)
    timeshift = 0
    for i in range(0, len(x)):
        if first_only[i] and fi != 0:
            x[i] = []
            y[i] = []
    if labels:
        lab = labels[fi*len(fields):(fi+1)*len(fields)]
    else:
        lab = fields[:]
    if args.multi:
        col = colors[:]
    else:
        col = colors[fi*len(fields):]
    fig = plotit(x, y, lab, colors=col)
    for i in range(0, len(x)):
        x[i] = []
        y[i] = []
if args.output is None:
    pylab.show()
    pylab.draw()
    input('press enter to exit....')
else:
    fname, fext = os.path.splitext(args.output)
    if fext == '.html':
        import mpld3
        html = mpld3.fig_to_html(fig)
        f_out = open(args.output, 'w')
        f_out.write(html)
        f_out.close()
    else:
        pylab.legend(loc=2,prop={'size':8})
        pylab.savefig(args.output, bbox_inches='tight', dpi=200)
