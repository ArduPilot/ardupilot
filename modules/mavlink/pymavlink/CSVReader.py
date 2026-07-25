#!/usr/bin/env python3
'''
CSV log file reader

Copyright Peter Barker 2020
Released under GNU GPL version 3 or later

Timestamp must be in first column, microseconds

First row must be column headings.

e.g.

pbarker@bluebottle:~/rc/pymavlink(pr/csvreader)$ head -2 $LOG
WEEK_NB;TOW;ACCL_X;ACCL_Y;ACCL_Z;GYRO_X;GYRO_Y;GYRO_Z;TEMP;STATUS;COUNTER;VALID_CHCKSM
2100.0;371149.08302956074;-0.25742456316947937;-0.017161637544631958;9.684066772460938;0.003054326167330146;-0.00578140327706933;0.0006544984644278884;34.599998474121094;0.0;54637.0;0.0
pbarker@bluebottle:~/rc/pymavlink(pr/csvreader)$ MAVExplorer.py "csv:$LOG:separator=;:timestamp_expression=gps_time_to_epoch(CSV.WEEK_NB,CSV.TOW*1000.0)"
MAV> graph CSV.GYRO_X

in this case the GPS time was in seconds-since-week-start, so a conversion to ms is required

'''

import csv
import struct
import os

from . import mavutil
from . import mavextra
from . import mavexpression

class CSVMessage(object):
    def __init__(self, message_type, fmt, line):
        self.fmt = fmt
        self.message_type = message_type

        self.line = []
        for entry in line:
            try:
                self.line.append(float(entry))
            except ValueError:
                self.line.append(entry)

    def get_type(self):
        return self.message_type

    def __str__(self):
        ret = "%s {" % self.message_type
        for (c, val) in zip(self.fmt.headings, self.line):
            ret += "%s : %s, " % (c, str(val))
        return ret + '}'

    # methods for MAVExplorer:
    def get_fieldnames(self):
        return self.fmt.headings
    # end methods for MAVExplorer

    def __getattr__(self, field):
        '''override field getter'''
        if field == '_timestamp':
            if self.fmt.timestamp_expression is not None:
                return mavexpression.evaluate_expression(self.fmt.timestamp_expression, self.fmt.messages)
            return int(self.line[0])
        return self.line[self.fmt.field_offset[field]]

class CSVFormat(object):
    def __init__(self, headings, messages, timestamp_expression=None):
        self.headings = headings
        self.messages = messages
        self.timestamp_expression = timestamp_expression

        # map from a field name to an offset in the line:
        self.field_offset = {}
        count = 0
        for heading in self.headings:
            self.field_offset[heading] = count
            count += 1

class CSVReader(object):
    '''parse a CSV file'''
    def __init__(self,
                 filename,
                 zero_time_base=False,
                 progress_callback=None,
                 separator=';',
                 message_type='CSV',
                 timestamp_expression=None,
                 ):

        separator = os.environ.get("CSV_SEPARATOR", separator)

        self.messages = { 'MAV' : self }
        self.filename = filename
        self.separator = separator
        self.message_type = message_type
        self.progress_callback = progress_callback
        self.timestamp_expression = timestamp_expression

        self.timestamp = 0
        self.verbose = False
        self.f = None
        self.linecount = None
        self.params = {}

        self._rewind()  # opens files etc etc
        self.recv_msg()  # populate self.messages
        self._rewind()  # opens files etc etc

        # start attributes for MAVExplorer
        self._flightmodes = []
        # end attributes for MAVExplorer

    # start methods for MAVExplorer:
    def flightmode_list(self):
        return self._flightmodes

    @property
    def _count(self):
        # implicitly rewinds
        if self.linecount is None:
            self.linecount = self.count_lines()
        return self.linecount

    def count_lines(self):
        self._rewind()
        count = 0
        while True:
            try:
                if count == 0:
                    if self._parse_next() == None:
                        raise StopIteration
                else:
                    next(self.reader)
                count += 1
            except StopIteration:
                self._rewind()
                return count

    def rewind(self):
        self._rewind()

    def name_to_id(self, name):
        return 1  # FIXME

    # end methods for MAVExplorer

    def _rewind(self):
        '''reset state on rewind'''
        self.percent = 0

        if self.f is not None:
            self.f.close()

        self.f = open(self.filename, mode='r')

        self.reader = csv.reader(self.f, delimiter=self.separator)

        self.fmt = CSVFormat(next(self.reader),
                             self.messages,
                             timestamp_expression=self.timestamp_expression)

    def recv_msg(self):
        return self._parse_next()

    def recv_match(self, condition=None, type=None, blocking=False):
#        print("recv_match called (condition=%s type=%s blocking=%s" % (str(condition), str(type), str(blocking)))
        '''recv the next message that matches the given condition
        type can be a string or a list of strings'''
        if type is not None and not isinstance(type, list) and not isinstance(type, set):
            type = [type]
        while True:
            m = self.recv_msg()
            if m is None:
                return None
            if type is not None and not m.get_type() in type:
                continue
            if not mavutil.evaluate_condition(condition, self.messages):
                continue
            return m

    def check_condition(self, condition):
        '''check if a condition is true'''
        return mavutil.evaluate_condition(condition, self.messages)

    def _parse_next(self):
        '''read one message, returning it as an object'''

        try:
            line = next(self.reader)
        except StopIteration:
            return None

        m = CSVMessage(self.message_type, self.fmt, line)

        self._add_msg(m)

        return m

    def _add_msg(self, m):
        '''add a new message'''
        self.messages[self.message_type] = m

if __name__ == "__main__":
    print("FIXME")
