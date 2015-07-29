#!/bin/python
from __future__ import print_function

import re
import sys
import os

from collections import OrderedDict
from math import sqrt

filenames = set()
tasks_names = {}
added_tasks = None
removed_tasks = set()
tasks_regex_filters = []
use_histogram = False
use_table = True
histogram_interval = 100

def usage():
    print("Usage: " + os.path.basename(__file__) + " [OPTIONS] FILE [FILE...]")
    print("""
This script outputs stats from the output of ardupilot's AP_Scheduler when it's
DEBUG parameter is set to ShowAll.

The options are listed bellow:

    -h, --help  Shows this help message.

    {+-}t<task_id>
    {+-}t<regex>
        This option can be used multiple times. When used with '+', the task
        numbered by task_id or the tasks that match with the regular expression
        regex are added to list of tasks to be analized. If there's no
        occurence of '+', then the list of tasks is initially considered all
        tasks in the log files. If there's at least one occurence of '+', then
        the list of tasks is initially considered empty and the options
        prefixed by '+' add the tasks to the list.

        Occurences with '-' remove the pointed tasks from the list of tasks.

        The order these options are passed matters.

    {+-}table
        Tells whether the table with stats should be printed ('+') or not
        ('-'). The table is printed by default.

    {+-}histogram[<interval>]
        Tells if a histogram output should be added ('+') or not ('-').  The
        value of <interval> is measured in units of us. The default value for
        <interval> is 100.

        The histogram is not added by default.
""")

def parse_args():
    global added_tasks, use_histogram, histogram_interval, use_table

    args = sys.argv[1:]

    if len(args) == 0:
        usage()
        exit(1)

    for arg in args:
        if arg in ('-h', '--help'):
            usage()
            exit(0)

        if arg[0] in ('+', '-'):
            operation = arg[0]
            arg = arg[1:]
            if arg == "table":
                use_table = True if operation == '+' else False
            elif arg.find("t") == 0:
                t_arg = arg[1:]
                if t_arg == "":
                    raise Exception("option t needs an argument")

                try:
                    task_id = int(t_arg)
                    if task_id < 0:
                        raise Exception("task_id can't be negative")

                    if operation == '+':
                        if added_tasks is None: added_tasks = set()
                        added_tasks.add(task_id)
                        if task_id in removed_tasks: removed_tasks.remove(task_id)
                    else:
                        removed_tasks.add(task_id)
                        if added_tasks and task_id in added_tasks: added_tasks.remove(task_id)
                except:
                    try:
                        task_regex = re.compile(t_arg)
                    except:
                        raise Exception("invalid regular expression %s" % t_arg)

                    tasks_regex_filters.append((operation, task_regex))

            elif arg.find("histogram") == 0:
                use_histogram = True if operation == '+' else False
                interval = arg[len("histogram"):]
                if len(interval) > 0:
                    try:
                        histogram_interval = int(interval)
                    except:
                        raise Exception("Invalid histogram interval " + interval)
                    if histogram_interval <= 0:
                        raise Exception("Histogram interval must be greater than zero")
            else:
                raise Exception("Invalid operation " + operation + arg)
        else:
            filenames.add(arg)

def task_is_considered(task_id):
    global added_tasks
    if added_tasks is not None:
        if task_id in added_tasks: return True
    else:
        if task_id in removed_tasks: return False

    task_name = tasks_names[task_id]

    for op, regex in reversed(tasks_regex_filters):
        if regex.search(task_name) is not None:
            if op == '+':
                if added_tasks is None: added_tasks = set()
                added_tasks.add(task_id)
            else:
                removed_tasks.add(task_id)
            break

    if added_tasks is not None and task_id not in added_tasks: return False
    if task_id in removed_tasks: return False
    return True

def print_table(data, task_id):
    table_rows = OrderedDict((
            ('time_limits', 'Time limit'),
            ('means', 'Average'),
            ('variances', 'Variance'),
            ('std_deviations', 'Standard Deviation'),
            ('samples', 'Number of samples'),
            ('overruns', 'Number of overruns'),
            ('overrun_rates', 'Overrun rate')
    ))

    row_label_max_length = max(len(table_rows[k]) for k in table_rows)

    print("    " + (" " * row_label_max_length), end="")
    for filename in filenames:
        print("%12s" % filename, end=" ")
    print()

    for row_key in table_rows:
        print("    " + ("%" + str(row_label_max_length) + "s") % table_rows[row_key], end=" ")
        for filename in filenames:
            local_data = data[filename]
            if task_id in local_data['values']:
                print("%12.3f" % local_data[row_key][task_id], end=" ")
            else:
                print(" " * 6 + "-" + " " * 5, end=" ")
        print()

    print()


def print_histogram(data, task_id):
    print(" "*8 + "Histogram:     ", end=" ")
    for filename in filenames:
        print("%24s" % filename, end=" ")
    print()
    print(" "*8 + "Intervals      ", end=" ")
    for filename in filenames:
        print("%24s" % "Count Frequency", end=" ")
    print()

    histogram_max_len = max(len(data[filename]['histograms'][task_id]) for filename in filenames)
    for i in range(histogram_max_len):
        line_buffer = "[%5d : %5d)  " % (i * histogram_interval, (i + 1) * histogram_interval)
        for filename in filenames:
            histogram = data[filename]['histograms'][task_id]
            if i < len(histogram):
                count = histogram[i]
            else:
                count = 0
            local_line_buffer = " "
            local_line_buffer += "%8d" % count
            local_line_buffer += " "
            local_line_buffer += "%9.3f" % (count / data[filename]['samples'][task_id])
            local_line_buffer += " "
            local_line_buffer = ("%24s" % local_line_buffer) + " "
            line_buffer += local_line_buffer
        print(" "*8 + line_buffer)
    print()


def run():
    line_regex = re.compile(r'Scheduler task\[(\d+)-(.+)\] \((\d+)\/(\d+)\)')

    data = {}
    max_task_id = -1

    for filename in filenames:
        f = open(filename, "rb")
        data[filename] = {} # Key: task_id
        expected_times = data[filename]['time_limits'] = {}
        values = data[filename]['values'] = {}
        overruns = data[filename]['overruns'] = {}
        histograms = data[filename]['histograms'] = {}

        for line in f:
            line = line.decode('utf_8', 'ignore')
            match = line_regex.search(line)
            if match is not None:
                task_id = int(match.group(1))

                task_name = match.group(2)
                if task_id not in tasks_names: tasks_names[task_id] = task_name

                if not task_is_considered(task_id): continue
                if task_id > max_task_id: max_task_id = task_id

                task_time_taken = int(match.group(3))

                if task_id not in values:
                    expected_times[task_id] = int(match.group(4))
                    values[task_id] = []
                    overruns[task_id] = 0
                    histograms[task_id] = []

                values[task_id].append(task_time_taken)

                if task_time_taken > expected_times[task_id]:
                    overruns[task_id] += 1

                if use_histogram:
                    histogram = histograms[task_id]
                    histogram_index = int(task_time_taken / histogram_interval)
                    while histogram_index >= len(histogram): histogram.append(0)
                    histogram[histogram_index] += 1

        means = data[filename]['means'] = {
                task_id: float(sum(values[task_id])) / len(values[task_id]) for task_id in values}
        variances = data[filename]['variances'] = {
                task_id: sum((v - means[task_id])**2 for v in values[task_id]) / len(values[task_id])
                for task_id in values}
        data[filename]['std_deviations'] = {
                task_id: sqrt(variances[task_id]) for task_id in variances}
        data[filename]['samples'] = {
                task_id: len(values[task_id]) for task_id in values}
        data[filename]['overrun_rates'] = {
                task_id: float(overruns[task_id]) / len(values[task_id]) for task_id in values}
        f.close()

    #Ouput
    for task_id in range(max_task_id + 1):
        if not task_is_considered(task_id): continue

        print("Task %2d - %s:" % (task_id, tasks_names[task_id]))

        if use_table: print_table(data, task_id)

        if use_histogram: print_histogram(data, task_id)




try:
    parse_args()
except Exception as e:
    print("argument error: " + str(e), file=sys.stderr)
    exit(1)

try:
    run()
except Exception as e:
    print("runtime error: " + str(e), file=sys.stderr)
    exit(1)

exit(0)
