#!/usr/bin/env python

'''
A script suitable for use as a git pre-commit hook to ensure your
files are flake8-compliant before committing them.

Use this by copying it to a file called $ARDUPILOT_ROOT/.git/hooks/pre-commit

 AP_FLAKE8_CLEAN
'''

import os
import re
import sys
import subprocess


class PreCommitFlake8(object):

    def __init__(self):
        pass

    def progress(self, message):
        print("***** %s" % (message, ))

    def check_file(self, filepath):
        content = open(filepath).read()
        if "AP_FLAKE8_CLEAN" not in content:
            return True
        self.progress("Checking (%s)" % filepath)
        retcode = subprocess.call(["flake8", filepath])
        if retcode != 0:
            self.progress("File (%s) failed with retcode (%s)" %
                          (filepath, retcode))
            return False
        return True

    def split_git_diff_output(self, output):
        '''split output from git-diff into a list of (status, filepath) tuples'''
        ret = []
        if type(output) == bytes:
            output = output.decode('utf-8')
        for line in output.split("\n"):
            if len(line) == 0:
                continue
            ret.append(re.split(r"\s+", line))
        return ret

    def run(self):
        # generate a list of files which have changes not marked for commit
        output = subprocess.check_output([
            "git", "diff", "--name-status"])
        dirty_list = self.split_git_diff_output(output)
        dirty = set()
        for (status, dirty_filepath) in dirty_list:
            dirty.add(dirty_filepath)

        # check files marked for commit:
        output = subprocess.check_output([
            "git", "diff", "--cached", "--name-status"])
        output_tuples = self.split_git_diff_output(output)
        self.retcode = 0
        for output_tuple in output_tuples:
            if len(output_tuple) > 2:
                if output_tuple[0].startswith('R'):
                    # rename, check destination
                    (status, filepath) = (output_tuple[0], output_tuple[2])
                else:
                    raise ValueError("Unknown status %s" % str(output_tuple[0]))
            else:
                (status, filepath) = output_tuple
            if filepath in dirty:
                self.progress("WARNING: (%s) has unstaged changes" % filepath)
            if status == 'D':
                # don't check deleted files
                continue
            (base, extension) = os.path.splitext(filepath)
            if extension != ".py":
                continue
            if not self.check_file(filepath):
                self.retcode = 1
        return self.retcode


if __name__ == '__main__':
    precommit = PreCommitFlake8()
    sys.exit(precommit.run())
