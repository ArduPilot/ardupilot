#!/usr/bin/env python

'''
Create a replay log using master branch.
Check out a specified branch, compile and run Replay against replay log
Run check_replay.py over the produced log
'''

import git
import os
import subprocess
import sys

import check_replay

class CheckReplayBranch(object):
    def __init__(self, master='remotes/origin/master'):
        self.master = master

    def find_topdir(self):
        here = os.getcwd()
        bits = here.split(os.path.sep)
        while len(bits):
#            print("bits: %s" % str(bits))
            tmp = bits[:]
            tmp.extend([".git"])  # can we look for something more specific?
            flagfile = os.path.sep.join(tmp)
            if os.path.exists(flagfile):
#                print("Found in (%s)" % str(flagfile))
                return os.path.sep.join(bits)
            bits = bits[:-2]
        raise FileNotFoundError()

    def find_repo(self):
        return git.Repo(self.topdir)

    def assert_tree_clean(self):
        if self.repo.is_dirty():
            raise ValueError("Tree is dirty")

    def run(self):
        self.topdir = self.find_topdir()
        self.repo = self.find_repo()
        self.assert_tree_clean()

        # remember where we were:
        old_branch = self.repo.active_branch

        # check out the master branch:
        self.repo.head.reference = self.master
        self.repo.head.reset(index=True, working_tree=True)

        # generate a log:
        os.chdir(self.topdir)
        print("chdir (%s)" % str(self.topdir))
        subprocess.check_call(["Tools/autotest/autotest.py", "--debug", "build.Copter", "test.Copter.Replay"])

        logfile_name = "00000004.BIN"  # FIXME; parse output of subprocess?
        logfile_path = os.path.join("logs", logfile_name)

        # check out the original branch:
        self.repo.head.reference = old_branch
        self.repo.head.reset(index=True, working_tree=True)

        # run check_replay across Replay log
        return check_replay.check_log(logfile_path)

if __name__ == '__main__':
    import sys
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--master", default='remotes/origin/master', help="branch to consider master branch")

    args = parser.parse_args()

    s = CheckReplayBranch(master=args.master)
    if not s.run():
        sys.exit(1)

    sys.exit(0)
