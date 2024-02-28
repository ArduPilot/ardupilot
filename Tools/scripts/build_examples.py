#!/usr/bin/env python

# useful script to test the build of all example code
# This helps when doing large merges
# Peter Barker, June 2016
# based on build_examples.sh,  Andrew Tridgell, November 2012

# AP_FLAKE8_CLEAN

import os
import sys
import optparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../autotest/pysim'))

import util  # NOQA


class BuildExamples():
    def __init__(self, targets=[], clean=False):
        print("init")
        self.targets = targets
        self.clean = clean

    def run(self):
        for target in self.targets:
            util.build_examples(target, clean=self.clean)


if __name__ == '__main__':

    parser = optparse.OptionParser("build_examples.py")
    parser.add_option(
        "--target",
        type='string',
        default=['navio', 'Pixhawk1'],
        help='list of targets for which to build examples', action='append'
    )
    parser.add_option("--clean", action='store_true', default=False, help='clean build')
    opts, args = parser.parse_args()

    buildexamples = BuildExamples(targets=opts.target, clean=opts.clean)
    buildexamples.run()
