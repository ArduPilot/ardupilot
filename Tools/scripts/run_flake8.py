#!/usr/bin/env python

"""
Runs flake8 over Python files which contain a marker indicating
they are clean, ensures that they actually are

 AP_FLAKE8_CLEAN
"""

import os
import subprocess
import sys

import argparse

os.environ['PYTHONUNBUFFERED'] = '1'


class Flake8Checker(object):
    def __init__(self):
        self.retcode = 0

    def progress(self, string):
        print("****** %s" % (string,))

    def check(self, filepath):
        self.progress("Checking (%s)" % filepath)
        retcode = subprocess.call(["flake8", filepath])
        if retcode != 0:
            self.progress("File (%s) failed with retcode (%s)" %
                          (filepath, retcode))
            self.retcode = 1

    def run(self):
        for (dirpath, dirnames, filenames) in os.walk("Tools"):
            for filename in filenames:
                if os.path.splitext(filename)[1] != ".py":
                    continue
                filepath = os.path.join(dirpath, filename)
                content = open(filepath).read()
                if "AP_FLAKE8_CLEAN" not in content:
                    continue
                self.check(filepath)
        return self.retcode


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check all Python files for flake8 cleanliness')
    # parser.add_argument('--build', action='store_true', default=False, help='build as well as configure')
    args = parser.parse_args()

    checker = Flake8Checker()
    sys.exit(checker.run())
