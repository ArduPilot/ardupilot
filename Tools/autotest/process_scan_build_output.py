#!/usr/bin/env python3

'''
Tool to process stdout from clang_scan_build, and manipulate files for archiving

AP_FLAKE8_CLEAN
'''

import os
import pathlib
import re
import sys


class ProcessScanBuildOutput():
    def __init__(self, stdout_filepath):
        self.stdout_filepath = stdout_filepath

    def progress(self, msg):
        print(f"psbo: {msg}")

    def run(self):
        t = pathlib.Path(self.stdout_filepath).read_text()
        m_scan_build_dir = None
        m_bug_count = None
        for line in t.split("\n"):
            # extract scan-build-dir
            m = re.match(r".*Run 'scan-view (.+)'", line)
            if m is not None:
                m_scan_build_dir = m
            # extract count of bugs
            m = re.match(r".*?(\d+) bugs found", line)
            if m is not None:
                m_bug_count = m
        if m_scan_build_dir is None:
            raise ValueError("Did not find scan-build output directory in output")
        if m_bug_count is None:
            raise ValueError("Did not find bug count in output")
        scan_build_dir = m_scan_build_dir.group(1)
        bug_count = int(m_bug_count.group(1))

        dest = "/__w/ardupilot/ardupilot/scan-build"
        self.progress(f"Renaming {scan_build_dir} to {dest}")
        os.rename(scan_build_dir, dest)

        new_stdout_filepath = os.path.join(dest, self.stdout_filepath)
        self.progress(f"Renaming {self.stdout_filepath} to {new_stdout_filepath}")
        os.rename(self.stdout_filepath, new_stdout_filepath)

        self.progress(f"Extracted Bug count is {bug_count}")

        if bug_count != self.expected_bug_count():
            self.progress(f"Bug count incorrect (want={self.expected_bug_count()} got={bug_count})")
            if bug_count < self.expected_bug_count():
                self.progress("You have fixed bugs, please modify Tools/autotest/process_scan_build_output.py to reduce expected count")  # noqa: E501
            else:
                self.progress("clang-scan-build has found problems being introduced; see artifacts")
            sys.exit(1)

    def expected_bug_count(self):
        return 77


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='tool to post-process output from clang-scan-build in CI')
    parser.add_argument('clang_scan_build_stdout', default=None, help='file containing stdout from clang-scan-build process')

    args = parser.parse_args()

    p = ProcessScanBuildOutput(args.clang_scan_build_stdout)
    p.run()
