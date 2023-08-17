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
DRY_RUN_DEFAULT = False


class AStyleChecker(object):
    def __init__(self, *, dry_run=DRY_RUN_DEFAULT):
        self.retcode = 0
        self.directories_to_check = [
            'libraries/AP_DDS',
            'libraries/AP_ExternalControl'
        ]
        self.files_to_check = []
        self.dry_run = dry_run

    def progress(self, string):
        print("****** %s" % (string,))

    def check(self):
        '''run astyle on all files in self.files_to_check'''
        # for path in self.files_to_check:
        #     self.progress("Checking (%s)" % path)
        astyle_command = ["astyle"]
        if self.dry_run:
            astyle_command.append("--dry-run")

        astyle_command.append("--options=Tools/CodeStyle/astylerc")
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
    parser.add_argument('--dry-run',
                        action='store_true',
                        default=DRY_RUN_DEFAULT,
                        help='Perform a trial run with no changes made to check for formatting')
    args = parser.parse_args()

    checker = AStyleChecker(dry_run=args.dry_run)
    sys.exit(checker.run())
