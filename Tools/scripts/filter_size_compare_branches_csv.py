#!/usr/bin/python

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
