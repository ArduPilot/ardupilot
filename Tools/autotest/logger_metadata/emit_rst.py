from __future__ import print_function

import emitter

class RSTEmitter(emitter.Emitter):
    def preface(self):
        return """.. Dynamically generated list of Logger Messages
.. This page was generated using Tools/autotest/logger_metdata/parse.py

.. DO NOT EDIT

.. _logmessages:

Onboard Message Log Messages
============================

This is a list of log messages which may be present in logs produced and stored onboard ArduPilot vehicles.

"""
    def postface(self):
        return ""

    def start(self):
        self.fh = open("LogMessages.rst", mode='w')
        print(self.preface(), file=self.fh)

    def emit(self, doccos, enumerations):
        self.start()
        for docco in doccos:
            print('.. _%s:' % docco.name, file=self.fh)
            print("", file=self.fh)
            desc = docco.description
            print(docco.name, file=self.fh)
            print("~" * len(docco.name), file=self.fh)
            if desc is not None:
                print("\n%s\n" % desc, file=self.fh)

            rows = []
            for f in docco.fields_order:
                if "description" in docco.fields[f]:
                    fdesc = docco.fields[f]["description"]
                else:
                    fdesc = ""
                fieldnamething = None
                if "bitmaskenum" in docco.fields[f]:
                    fieldnamething = "bitmaskenum"
                    table_label = "Bitmask values"
                elif "valueenum" in docco.fields[f]:
                    fieldnamething = "valueenum"
                    table_label = "Values"
                if fieldnamething is not None:
                    enum_name = docco.fields[f][fieldnamething]
                    if enum_name not in enumerations:
                        raise Exception("Unknown enum (%s) (have %s)" %
                                        (enum_name, "\n".join(sorted(enumerations.keys()))))
                    enumeration = enumerations[enum_name]
                    bitmaskrows = []
                    for enumentry in enumeration.entries:
#                        print("enumentry: %s" % str(enumentry))
                        comment = enumentry.comment
                        if comment is None:
                            comment = ""
                        bitmaskrows.append([enumentry.name, str(enumentry.value), comment])
                    fdesc += "\n%s:\n\n%s" % (table_label, self.tablify(bitmaskrows))
                rows.append([f, fdesc])

            print(self.tablify(rows), file=self.fh)

            print("", file=self.fh)
        self.stop()

    def stop(self):
        print(self.postface(), file=self.fh)
        self.fh.close()


    # tablify swiped from rstemit.py

    def tablify_row(self, rowheading, row, widths, height):
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

