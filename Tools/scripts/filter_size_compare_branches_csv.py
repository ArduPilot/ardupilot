#!/usr/bin/env python3

# flake8: noqa

import argparse
import csv
import sys
import tabulate

class FilterSizeCompareBranchesCSV(object):
    def __init__(self, filename, show_binary_identical=True, ):
        self.filename = filename
        self.show_binary_identical = show_binary_identical

    def pretty_print_data(self, data):
        print(tabulate.tabulate(data))

    def prune_empty_columns(self, data):
        if len(data) == 0:
            return data

        # populate an array indicating whether here are any non-empty
        # values in each column:
        line_length = len(data[0])
        column_has_content = [False] * line_length
        for line in data[1:]:  # ASSUMPTION we have a header row!
            for i in range(line_length):
                if line[i] != "":
                    column_has_content[i] = True

        # if all columns are full then we don't need to any more:
        all_empty = False
        if not any(column_has_content):
            return data

        new_data = []
        for line in data:
            new_line = []
            for i in range(line_length):
                if not column_has_content[i]:
                    continue
                new_line.append(line[i])
            new_data.append(new_line)

        return new_data

    def run(self):
        csvt = csv.reader(open(self.filename,'r'))
        data = list(csvt)

        if not self.show_binary_identical:
            new_data = []
            for line in data:
                prune = True
                for delta in line[1:]:
                    if delta != "" and delta != "*":
                        prune = False
                        break
                if not prune:
                    new_data.append(line)
            data = new_data

        data = self.prune_empty_columns(data)

        self.pretty_print_data(data)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Parse csv produced by size_compare_branches.py, filter it',
    )
    parser.add_argument("csv")
    parser.add_argument("--hide-binary-identical", action="store_true", default=False)
    args = parser.parse_args()

    x = FilterSizeCompareBranchesCSV(
        args.csv,
        show_binary_identical=not args.hide_binary_identical,
    )
    x.run()
