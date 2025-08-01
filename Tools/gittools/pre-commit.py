#!/usr/bin/env python3

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


class AP_PreCommit(object):

    @staticmethod
    def progress(message):
        print(f"***** {message}")

    @staticmethod
    def has_flake8_tag(filepath):
        with open(filepath) as fp:
            return "AP_FLAKE8_CLEAN" in fp.read()

    def files_are_flake8_clean(self, files_to_check):
        if files_to_check:
            for path in files_to_check:
                self.progress("Checking (%s)" % path)
            try:
                subprocess.check_output(["flake8"] + files_to_check, stderr=subprocess.STDOUT)
            except subprocess.CalledProcessError as e:
                self.progress(f"Flake8 check failed: ({e.output})")
                return False
        return True

    @staticmethod
    def split_git_diff_output(output):
        '''split output from git-diff into a list of (status, filepath) tuples'''
        ret = []
        if isinstance(output, bytes):
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
        files_to_check_flake8 = []
        for output_tuple in output_tuples:
            if len(output_tuple) > 2:
                if output_tuple[0].startswith('R'):
                    # rename, check destination
                    (status, filepath) = (output_tuple[0], output_tuple[2])
                else:
                    raise ValueError(f"Unknown status {output_tuple[0]}")
            else:
                (status, filepath) = output_tuple
            if filepath in dirty:
                self.progress("WARNING: (%s) has unstaged changes" % filepath)
            if status == 'D':
                # don't check deleted files
                continue
            (base, extension) = os.path.splitext(filepath)
            if extension == ".py" and self.has_flake8_tag(filepath):
                files_to_check_flake8.append(filepath)
        if not self.files_are_flake8_clean(files_to_check_flake8):
            return 1
        return 0


if __name__ == '__main__':
    precommit = AP_PreCommit()
    sys.exit(precommit.run())
