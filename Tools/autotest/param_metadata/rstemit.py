#!/usr/bin/env python

import re
from param import known_param_fields
from emit import Emit
import cgi


# Emit docs in a RST format
class RSTEmit(Emit):
    def blurb(self):
        return """This is a complete list of the parameters which can be set (e.g. via the MAVLink protocol) to control vehicle behaviour. They are stored in persistent storage on the vehicle.

This list is automatically generated from the latest ardupilot source code, and so may contain parameters which are not yet in the stable released versions of the code.
"""

    def toolname(self):
        return "Tools/autotest/param_metadata/param_parse.py"

    def __init__(self):
        Emit.__init__(self)
        output_fname = 'Parameters.rst'
        self.f = open(output_fname, mode='w')
        self.spacer = re.compile("^", re.MULTILINE)
        self.rstescape = re.compile("([^a-zA-Z0-9\n 	])")
        self.preamble = """.. Dynamically generated list of documented parameters
.. This page was generated using {toolname}

.. DO NOT EDIT


.. _parameters:

Complete Parameter List
=======================

{blurb}

""".format(blurb=self.escape(self.blurb()),
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

    def tablify_row(self, rowheading, row, widths, height):
        ret = ""
        joiner = "|"

        row_lines = [x.split("\n") for x in row]
        for row_line in row_lines:
            row_line.extend([""] * (height - len(row_line)))
        if rowheading is not None:
            rowheading_lines = rowheading.split("\n")
            rowheading_lines.extend([""] * (height - len(rowheading_lines)))

        out_lines = []
        for i in range(0, height):
            out_line = ""
            if rowheading is not None:
                rowheading_line = rowheading_lines[i]
                out_line += joiner + " " + rowheading_line + " " * (widths[0] - len(rowheading_line) - 1)
                joiner = "#"
            j = 0
            for item in row_lines:
                widthnum = j
                if rowheading is not None:
                    widthnum += 1
                line = item[i]
                out_line += joiner + " " + line + " " * (widths[widthnum] - len(line) - 1)
                joiner = "|"
                j += 1
            out_line += "|"
            out_lines.append(out_line)
        return "\n".join(out_lines)

    def tablify_longest_row_length(self, rows, rowheadings, headings):
        check_width_rows = rows[:]
        if headings is not None:
            check_width_rows.append(headings)
        longest_row_length = 0
        for row in check_width_rows:
            if len(row) > longest_row_length:
                longest_row_length = len(row)
        if rowheadings is not None:
            longest_row_length += 1
        return longest_row_length

    def longest_line_in_string(self, string):
        longest = 0
        for line in string.split("\n"):
            if len(line) > longest:
                longest = len(line)
        return longest

    def tablify_calc_row_widths_heights(self, rows, rowheadings, headings):
        rows_to_check = []
        if headings is not None:
            rows_to_check.append(headings)
        rows_to_check.extend(rows[:])

        heights = [0] * len(rows_to_check)

        longest_row_length = self.tablify_longest_row_length(rows, rowheadings, headings)
        widths = [0] * longest_row_length

        all_rowheadings = []
        if rowheadings is not None:
            if headings is not None:
                all_rowheadings.append("")
            all_rowheadings.extend(rowheadings)

        for rownum in range(0, len(rows_to_check)):
            row = rows_to_check[rownum]
            values_to_check = []
            if rowheadings is not None:
                values_to_check.append(all_rowheadings[rownum])
            values_to_check.extend(row[:])
            colnum = 0
            for value in values_to_check:
                height = len(value.split("\n"))
                if height > heights[rownum]:
                    heights[rownum] = height
                longest_line = self.longest_line_in_string(value)
                width = longest_line + 2  # +2 for leading/trailing ws
                if width > widths[colnum]:
                    widths[colnum] = width
                colnum += 1
        return (widths, heights)

    def tablify(self, rows, headings=None, rowheadings=None):

        (widths, heights) = self.tablify_calc_row_widths_heights(rows, rowheadings, headings)

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
        if headings is not None:
            rowheading = None
            if rowheadings is not None:
                rowheading = ""
            ret += self.tablify_row(rowheading, headings, widths, heights[0]) + "\n"
            ret += heading_bar + "\n"
        for i in range(0, len(rows)):
            rowheading = None
            height = i
            if rowheadings is not None:
                rowheading = rowheadings[i]
            if headings is not None:
                height += 1
            ret += self.tablify_row(rowheading, rows[i], widths, heights[height]) + "\n"
            ret += bar + "\n"

        return ret

    def render_prog_values_field(self, render_info, param, field):
        values = (param.__dict__[field]).split(',')
        rows = []
        for value in values:
            v = [x.strip() for x in value.split(':')]
            rows.append(v)
        return self.tablify(rows, headings=render_info["headings"])

    def emit(self, g, f):
        tag = '%s Parameters' % self.escape(g.name)
        reference = "parameters_" + g.name

        field_table_info = {
            "Values": {
                "headings": ['Value', 'Meaning'],
            },
            "Bitmask": {
                "headings": ['Bit', 'Meaning'],
            },
        }

        ret = """

.. _{reference}:

{tag}
{underline}
""".format(tag=tag, underline="-" * len(tag),
           reference=reference)

        for param in g.params:
            if not hasattr(param, 'DisplayName') or not hasattr(param, 'Description'):
                continue
            d = param.__dict__
            if self.annotate_with_vehicle:
                name = param.name
            else:
                name = param.name.split(':')[-1]
            tag = '%s: %s' % (self.escape(name), self.escape(param.DisplayName),)
            tag = tag.strip()
            reference = param.name
            # remove e.g. "ArduPlane:" from start of parameter name:
            if self.annotate_with_vehicle:
                reference = g.name + "_" + reference.split(":")[-1]
            else:
                reference = reference.split(":")[-1]

            ret += """

.. _{reference}:

{tag}
{tag_underline}
""".format(tag=tag, tag_underline='~' * len(tag), reference=reference)

            if d.get('User', None) == 'Advanced':
                ret += '\n| *Note: This parameter is for advanced users*'
            ret += "\n\n%s\n" % self.escape(param.Description)

            headings = []
            row = []
            for field in param.__dict__.keys():
                if field not in ['name', 'DisplayName', 'Description', 'User'] and field in known_param_fields:
                    headings.append(field)
                    if field in field_table_info and Emit.prog_values_field.match(param.__dict__[field]):
                        row.append(self.render_prog_values_field(field_table_info[field], param, field))
                    elif field == "Range":
                        (param_min, param_max) = (param.__dict__[field]).split(' ')
                        row.append("%s - %s" % (param_min, param_max,))
                    else:
                        row.append(cgi.escape(param.__dict__[field]))
            if len(row):
                ret += "\n\n" + self.tablify([row], headings=headings) + "\n\n"
        self.t += ret + "\n"


def table_test():
    e = RSTEmit()
    print("Test 1")
    print(e.tablify([["A", "B"], ["C", "D"]]))

    print("Test 2")
    print e.tablify([["A", "B"], ["CD\nE", "FG"]])

    print("Test 3")
    print(e.tablify([["A", "B"], ["CD\nEF", "GH"]], rowheadings=["r1", "row2"]))

    print("Test 4")
    print(e.tablify([["A", "B"], ["CD\nEF", "GH"]], headings=["c1", "col2"]))

    print("Test 5")
    print(e.tablify([["A", "B"], ["CD\nEF", "GH"]], headings=["c1", "col2"], rowheadings=["r1", "row2"]))


if __name__ == '__main__':
    table_test()
