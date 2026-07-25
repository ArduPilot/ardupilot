#!/usr/bin/env python3

'''
example program that dumps a Mavlink log file. The log file is
assumed to be in the format that qgroundcontrol uses, which consists
of a series of MAVLink packets, each with a 64 bit timestamp
header. The timestamp is in microseconds since 1970 (unix epoch)
'''
import array
import fnmatch
import json
import os
import struct
import sys
import time

try:
    from pymavlink.mavextra import *
except:
    print("WARNING: Numpy missing, mathematical notation will not be supported..")

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--planner", action='store_true', help="use planner file format")
parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("-f", "--follow", action='store_true', help="keep waiting for more data at end of file (not implemented for .bin, .log, .csv")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("-q", "--quiet", action='store_true', help="don't display packets")
parser.add_argument("-o", "--output", default=None, help="output matching packets to give file")
parser.add_argument("-p", "--parms", action='store_true', help="preserve parameters in output with -o")
parser.add_argument("--format", default=None, help="Change the output format between 'standard', 'json', 'csv' and 'mat'. For the CSV output, you must supply types that you want. For MAT output, specify output file with --mat_file")
parser.add_argument("--csv_sep", dest="csv_sep", default=",", help="Select the delimiter between columns for the output CSV file. Use 'tab' to specify tabs. Only applies when --format=csv")
parser.add_argument("--types", default=None, help="types of messages (comma separated with wildcard)")
parser.add_argument("--nottypes", default=None, help="types of messages not to include (comma separated with wildcard)")
parser.add_argument("--mat_file", dest="mat_file", help="Output file path for MATLAB file output. Only applies when --format=mat")
parser.add_argument("-c", "--compress", action='store_true', help="Compress .mat file data")
parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
parser.add_argument("--zero-time-base", action='store_true', help="use Z time base for DF logs")
parser.add_argument("--no-bad-data", action='store_true', help="Don't output corrupted messages")
parser.add_argument("--show-source", action='store_true', help="Show source system ID and component ID")
parser.add_argument("--show-seq", action='store_true', help="Show sequence numbers")
parser.add_argument("--show-types", action='store_true', help="Shows all message types available on opened log")
parser.add_argument("--show-loss", action='store_true', help="Shows changes in lost messages")
parser.add_argument("--source-system", type=int, default=None, help="filter by source system ID")
parser.add_argument("--source-component", type=int, default=None, help="filter by source component ID")
parser.add_argument("--link", type=int, default=None, help="filter by comms link ID")
parser.add_argument("--verbose", action='store_true', help="Dump messages in a much more verbose (but non-parseable) format")
parser.add_argument("--mav10", action='store_true', help="parse as MAVLink1")
parser.add_argument("--reduce", type=int, default=0, help="reduce streaming messages")
parser.add_argument("--reduce-rate", type=float, default=0, help="reduce messages to maximum rate in Hz")
parser.add_argument("log", metavar="LOG")
parser.add_argument("--profile", action='store_true', help="run the Yappi python profiler")
parser.add_argument("--meta", action='store_true', help="output meta-data msgs even if not matching condition")

args = parser.parse_args()

if not args.mav10:
    os.environ['MAVLINK20'] = '1'

import inspect

from pymavlink import mavutil


if args.profile:
    import yappi    # We do the import here so that we won't barf if run normally and yappi not available
    yappi.start()

if args.format == 'mat':
    # Check that the mat_file argument has been specified
    if args.mat_file is None:
        print("mat_file argument must be specified when mat format is selected")
        sys.exit(1)
    # Load these modules here, as they're only needed for MAT file creation
    import scipy.io
    import numpy as np

filename = args.log
mlog = mavutil.mavlink_connection(filename, planner_format=args.planner,
                                  notimestamps=args.notimestamps,
                                  robust_parsing=args.robust,
                                  dialect=args.dialect,
                                  zero_time_base=args.zero_time_base)

output = None
if args.output:
    output = open(args.output, mode='wb')

types = args.types
if types is not None:
    types = types.split(',')

nottypes = args.nottypes
if nottypes is not None:
    nottypes = nottypes.split(',')

ext = os.path.splitext(filename)[1]
isbin = ext in ['.bin', '.BIN', '.px4log']
islog = ext in ['.log', '.LOG'] # NOTE: "islog" does not mean a tlog
istlog = ext in ['.tlog', '.TLOG']

# list of msgs to reduce in rate when --reduce is used
reduction_msgs = ['NKF*', 'XKF*', 'IMU*', 'AHR2', 'BAR*', 'ATT', 'BAT*', 'CTUN', 'NTUN', 'GP*', 'IMT*', 'MAG*', 'PL', 'POS', 'POW*', 'RATE', 'RC*', 'RFND', 'UBX*', 'VIBE', 'NKQ*', 'MOT*', 'CTRL', 'FTS*', 'DSF', 'CST*', 'LOS*', 'UWB*']
reduction_yes = set()
reduction_no = set()
reduction_count = {}

def reduce_msg(mtype, reduction_ratio):
    '''return True if this msg should be discarded by reduction'''
    if mtype in reduction_no:
        return False
    if not mtype in reduction_yes:
        for m in reduction_msgs:
            if fnmatch.fnmatch(mtype, m):
                reduction_yes.add(mtype)
                reduction_count[mtype] = 0
                break
        if not mtype in reduction_yes:
            reduction_no.add(mtype)
            return False
    reduction_count[mtype] += 1
    if reduction_count[mtype] == reduction_ratio:
        reduction_count[mtype] = 0
        return False
    return True

last_msg_rate_t = {}

def reduce_rate_msg(m, reduction_rate):
    '''return True if this msg should be discarded by reduction'''
    mtype = m.get_type()
    if mtype in ['PARM','MSG','FMT','FMTU','MULT','MODE','EVT','UNIT', 'VER']:
        return False
    t = getattr(m,'_timestamp',None)
    if t is None:
        return False
    if not mtype in last_msg_rate_t:
        last_msg_rate_t[mtype] = t
    dt = t - last_msg_rate_t[mtype]

    if dt < 0 or dt >= 1.0/reduction_rate:
        last_msg_rate_t[mtype] = t
        return False
    return True

if args.csv_sep == "tab":
    args.csv_sep = ","

# swiped from DFReader.py
def to_string(s):
    '''desperate attempt to convert a string regardless of what garbage we get'''
    if isinstance(s, str):
        return s
    return s.decode(errors="backslashreplace")

def match_type(mtype, patterns):
    '''return True if mtype matches pattern'''
    for p in patterns:
        if fnmatch.fnmatch(mtype, p):
            return True
    return False

# Write out a header row as we're outputting in CSV format.
fields = ['timestamp']
offsets = {}
if istlog and args.format == 'csv': # we know our fields from the get-go
    try:
        currentOffset = 1 # Store how many fields in we are for each message.
        for type in types:
            try:
                typeClass = "MAVLink_{0}_message".format(type.lower())
                fields += [type + '.' + x for x in inspect.getfullargspec(getattr(mavutil.mavlink, typeClass).__init__).args[1:]]
                offsets[type] = currentOffset
                currentOffset += len(fields)
            except IndexError:
                sys.exit(1)
            except AttributeError:
                print("Message type '%s' not found" % (type))
                sys.exit(1)
    except TypeError:
        print("You must specify a list of message types if outputting CSV format via the --types argument.")
        sys.exit(1)

    # The first line output are names for all columns
    print(args.csv_sep.join(fields))

if (isbin or islog) and args.format == 'csv': # need to accumulate columns from message
    if types is None or len(types) != 1:
        print("Need exactly one type when dumping CSV from bin file")
        sys.exit(1)

# Track types found
available_types = set()

# for DF logs pre-calculate types list
match_types=None
if types is not None and hasattr(mlog, 'name_to_id'):
    for k in mlog.name_to_id.keys():
        if match_type(k, types):
            if nottypes is not None and match_type(k, nottypes):
                continue
            if match_types is None:
                match_types = []
            match_types.append(k)

if (isbin or islog) and args.format == 'csv':
    # Make sure the specified type was found
    if match_types is None:
        print("Specified type '%s' not found in log file" % (types[0]))
        sys.exit(1)
    # we need FMT messages for column headings
    match_types.append("FMT")

last_loss = 0

# Keep track of data from the current timestep. If the following timestep has the same data, it's stored in here as well. Output should therefore have entirely unique timesteps.
MAT = {}    # Dictionary to hold output data for 'mat' format option
while True:
    m = mlog.recv_match(blocking=args.follow, type=match_types)
    if m is None:
        break
    m_type = m.get_type()
    available_types.add(m_type)
    if (isbin or islog) and m_type == "FMT" and args.format == 'csv':
        if m.Name == types[0]:
            fields += m.Columns.split(',')
            print(args.csv_sep.join(fields))

    if args.reduce and reduce_msg(m_type, args.reduce):
        continue

    if args.reduce_rate > 0 and reduce_rate_msg(m, args.reduce_rate):
        continue

    if output is not None:
        if (isbin or islog) and m_type == "FMT":
            output.write(m.get_msgbuf())
            continue
        if (isbin or islog) and m_type in ["FMTU", "MULT", "UNIT"]:
            output.write(m.get_msgbuf())
            continue
        if (isbin or islog) and (m_type == "PARM" and args.parms):
            output.write(m.get_msgbuf())
            continue
        if m_type == 'PARAM_VALUE' and args.parms:
            timestamp = getattr(m, '_timestamp', None)
            output.write(struct.pack('>Q', int(timestamp*1.0e6)) + m.get_msgbuf())
            continue

    if not mavutil.evaluate_condition(args.condition, mlog.messages) and (
            not (m_type in ['FMT', 'FMTU', 'MULT', 'PARM', 'MODE', 'UNIT', 'VER','CMD','MAVC','MSG','EV'] and args.meta)):
        continue
    if args.source_system is not None and args.source_system != m.get_srcSystem():
        continue
    if args.source_component is not None and args.source_component != m.get_srcComponent():
        continue
    if args.link is not None and args.link != m._link:
        continue

    if types is not None and m_type != 'BAD_DATA' and not match_type(m_type, types):
        continue

    if nottypes is not None and match_type(m_type, nottypes):
        continue

    # Ignore BAD_DATA messages is the user requested or if they're because of a bad prefix. The
    # latter case is normally because of a mismatched MAVLink version.
    if m_type == 'BAD_DATA' and (args.no_bad_data is True or m.reason == "Bad prefix"):
        continue

    # Grab the timestamp.
    timestamp = getattr(m, '_timestamp', 0.0)

    # If we're just logging, pack in the timestamp and data into the output file.
    if output:
        if not (isbin or islog):
            output.write(struct.pack('>Q', int(timestamp*1.0e6)))
        try:
            output.write(m.get_msgbuf())
        except Exception as ex:
            print("Failed to write msg %s: %s" % (m_type, str(ex)))

    # If quiet is specified, don't display output to the terminal.
    if args.quiet:
        continue

    # If JSON was ordered, serve it up. Split it nicely into metadata and data.
    if args.format == 'json':
        # Format our message as a Python dict, which gets us almost to proper JSON format
        data = m.to_dict()

        # Remove the mavpackettype value as we specify that later.
        del data['mavpackettype']

        # Also, if it's a BAD_DATA message, make it JSON-compatible by removing array objects
        if 'data' in data and type(data['data']) is not dict:
            data['data'] = list(data['data'])

        # Prepare the message as a single object with 'meta' and 'data' keys holding
        # the message's metadata and actual data respectively.
        meta = {"type": m_type, "timestamp": timestamp}
        if args.show_source:
            meta["srcSystem"] = m.get_srcSystem()
            meta["srcComponent"] = m.get_srcComponent()

        # convert any array.array (e.g. packed-16-bit fft readings) into lists:
        for key in data.keys():
            if type(data[key]) == array.array:
                data[key] = list(data[key])
        # convert any byte-strings into utf-8 strings.  Don't die trying.
        for key in data.keys():
            if type(data[key]) == bytes:
                data[key] = to_string(data[key])
        outMsg = {"meta": meta, "data": data}

        # Now print out this object with stringified properly.
        print(json.dumps(outMsg))

    # CSV format outputs columnar data with a user-specified delimiter
    elif args.format == 'csv':
        data = m.to_dict()
        if isbin or islog:
            csv_out = [str(data[y]) if y != "timestamp" else "" for y in fields]
        else:
            csv_out = [str(data[y.split('.')[-1]]) if y.split('.')[0] == m_type and y.split('.')[-1] in data else "" for y in fields]
        csv_out[0] = "{:.8f}".format(timestamp)
        print(args.csv_sep.join(csv_out))

    # MAT format outputs data to a .mat file specified through the
    # --mat_file option
    elif args.format == 'mat':
        # If this packet contains data (i.e. is not a FMT
        # packet), append the data in this packet to the
        # corresponding list
        if m_type != 'FMT':

            # If this packet type has not yet been
            # seen, add a new entry to the big dict
            if m_type not in MAT:
                MAT[m_type] = {}

            md = m.to_dict()
            del md['mavpackettype']
            cols = md.keys()
            for col in cols:
                # If this column hasn't had data entered,
                # make a new key and list
                if col in MAT[m_type]:
                    MAT[m_type][col].append(md[col])
                else:
                    MAT[m_type][col] = [md[col]]
    elif args.show_types:
        # do nothing
        pass
    elif args.verbose and istlog:
        mavutil.dump_message_verbose(sys.stdout, m)
        print("")
    elif args.verbose and hasattr(m,"dump_verbose"):
        m.dump_verbose(sys.stdout)
        print("")
    else:
        # Otherwise we output in a standard Python dict-style format
        s = "%s.%02u: %s" % (time.strftime("%Y-%m-%d %H:%M:%S",
                                           time.localtime(timestamp)),
                             int(timestamp*100.0)%100, m)
        if args.show_source:
            s += " srcSystem=%u srcComponent=%u" % (m.get_srcSystem(), m.get_srcComponent())
        if args.show_seq:
            s += " seq=%u" % m.get_seq()
        print(s)
    if args.show_loss:
        if last_loss != mlog.mav_loss:
            print("lost %d messages" % (mlog.mav_loss - last_loss))
            last_loss = mlog.mav_loss

# Export the .mat file
if args.format == 'mat':
    scipy.io.savemat(args.mat_file, MAT, do_compression=args.compress)

if args.show_types:
    for msgType in available_types:
        print(msgType)

if args.profile:
    yappi.get_func_stats().print_all()
    yappi.get_thread_stats().print_all()
