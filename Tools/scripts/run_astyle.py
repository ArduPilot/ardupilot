#!/usr/bin/env python3

"""
Runs astyle over directory sub-trees known to be "astyle-clean"

 AP_FLAKE8_CLEAN
"""

import os
import pathlib
import subprocess
import sys

import argparse

os.environ['PYTHONUNBUFFERED'] = '1'


class AStyleChecker(object):
    def __init__(self):
        self.retcode = 0
        self.directories_to_check = [
            'libraries/AP_DDS',
        ]
        self.files_to_check = []

    def progress(self, string):
        print("****** %s" % (string,))

    def check(self):
        '''run astyle on all files in self.files_to_check'''
        # for path in self.files_to_check:
        #     self.progress("Checking (%s)" % path)
        astyle_command = ["astyle", "--dry-run"]
        astyle_command.extend(self.files_to_check)
        ret = subprocess.run(
            astyle_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )
        if ret.returncode != 0:
            self.progress("astyle check failed: (%s)" % (ret.stdout))
            self.retcode = 1
        if "Formatted" in ret.stdout:
            self.progress("Files needing formatting found")
            print(ret.stdout)
            self.retcode = 1

    def run(self):
        for d in self.directories_to_check:
            self.files_to_check.extend(list(pathlib.Path(d).glob("*")))
        self.files_to_check = list(filter(lambda x : x.suffix in [".c", ".h", ".cpp"], self.files_to_check))
        self.check()
        return self.retcode


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check all Python files for astyle cleanliness')
    # parser.add_argument('--build', action='store_true', default=False, help='build as well as configure')
    args = parser.parse_args()

    checker = AStyleChecker()
    sys.exit(checker.run())
