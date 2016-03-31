#!/usr/bin/env python

import re
from param import *
from emit import Emit
import cgi

# Emit docs in a RST format
class RSTEmit(Emit):

    def blurb(self):
        return '''This is a complete list of the parameters which can be set (e.g. via the MAVLink protocol) to control vehicle behaviour. They are stored in persistent storage on the vehicle.

This list is automatically generated from the latest ardupilot source code, and so may contain parameters which are not yet in the stable released versions of the code.
'''
    def toolname(self):
        return "Tools/autotest/param_metadata/param_parse.py"

    def __init__(self):
        output_fname = 'Parameters.rst'
        self.f = open(output_fname, mode='w')
        self.spacer = re.compile("^", re.MULTILINE)
        self.rstescape = re.compile("([^a-zA-Z0-9\n 	])")
        self.preamble = '''.. Dynamically generated list of documented parameters
.. This page was generated using {toolname}

.. DO NOT EDIT


.. _parameters:

Complete Parameter List
=======================

{blurb}

'''.format(blurb=self.escape(self.blurb()),
           toolname=self.escape(self.toolname()))
        self.t = ''

    def escape(self, s):
        ret = re.sub(self.rstescape, "\\\\\g<1>", s)
        return ret

    def close(self):
        self.f.write(self.preamble)
        self.f.write(self.t)
        self.f.close()


    def start_libraries(self):
        pass

    def tablify_row(self, row, widths):
        ret = ""
        i=0
        for item in row:
            ret += "| " + item + " "*(widths[i]-len(item)-1)
            i += 1
        ret += "|"
        return ret

    def tablify(self, rows, headings=None):
        if headings is None:
            print("headings must be supplied ATM")
            sys.exit(1)

        # determine column widths
        widths = [0] * len(headings)
        i=0
        for heading in headings:
            widths[i] = len(str(heading)) + 2 # +2 for leading/trailing ws
            i += 1
        for row in rows:
            i=0
            for value in row:
                width = len(str(value)) + 2 # +2 for leading/trailing ws
                if width > widths[i]:
                    widths[i] = width
                i += 1
        # create dividing lines
        bar = ""
        heading_bar = ""
        for width in widths:
            bar += "+"
            heading_bar += "+"
            bar += "-" * width
            heading_bar += "=" * width
        bar += "+"
        heading_bar += "+"

        # create table
        ret = bar + "\n"
        ret += self.tablify_row(headings, widths) + "\n"
        ret += heading_bar + "\n"
        for row in rows:
            ret += self.tablify_row(row, widths) + "\n"
            ret += bar + "\n"

        return ret

    def emit(self, g, f):
        tag = '%s Parameters' % self.escape(g.name)
        reference = "parameters_" + g.name
        ret = '''

.. _{reference}:

{tag}
{underline}
'''.format(tag=tag,underline="-" * len(tag),
           reference=reference)

        for param in g.params:
            if not hasattr(param, 'DisplayName') or not hasattr(param, 'Description'):
                continue
            d = param.__dict__
            if self.annotate_with_vehicle:
                name = param.name
            else:
                name = param.name.split(':')[-1]
            tag = '%s: %s' % (self.escape(name), self.escape(param.DisplayName), )
            tag = tag.strip()
            reference = param.name
            # remove e.g. "ArduPlane:" from start of parameter name:
            if self.annotate_with_vehicle:
                reference = g.name + "_" + reference.split(":")[-1]
            else:
                reference = reference.split(":")[-1]

            ret += '''

.. _{reference}:

{tag}
{tag_underline}
'''.format(tag=tag, tag_underline='~'*len(tag), reference=reference)

            if d.get('User',None) == 'Advanced':
                ret += '\n| *Note: This parameter is for advanced users*'
            ret += "\n\n%s\n" % self.escape(param.Description)

            headings = []
            rows = []
            for field in param.__dict__.keys():
                if field not in ['name', 'DisplayName', 'Description', 'User'] and field in known_param_fields:
                    if field == 'Values' and Emit.prog_values_field.match(param.__dict__[field]):
                        values = (param.__dict__[field]).split(',')
                        rows = []
                        for value in values:
                            v = [ x.strip() for x in value.split(':') ]
                            rows.append(v)
                        ret += "\n" + self.tablify(rows, headings=['Value','Meaning']) + "\n\n"
                    else:
                        ret += "%s: %s\n" % (field, cgi.escape(param.__dict__[field]))

        self.t += ret
