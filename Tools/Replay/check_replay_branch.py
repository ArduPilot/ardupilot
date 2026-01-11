#!/usr/bin/env python3

# flake8: noqa

'''
Create a replay log using master branch.
Check out a specified branch, compile and run Replay against replay log
Run check_replay.py over the produced log
'''

import git # https://pypi.org/project/GitPython/
import glob
import os
import subprocess
import sys
import re

from pymavlink import DFReader

import check_replay

class CheckReplayBranch(object):
    def __init__(self, master='master', no_clean=False, no_debug=False):
        self.master = master
        self.no_clean = no_clean
        self.no_debug = no_debug

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

    def is_replayable_log(self, logfile_path):
        '''returns true if an XKF1 or NKF1 message appears in first 5000
        messages'''
        dfreader =  DFReader.DFReader_binary(logfile_path, zero_time_base=True);
        seen_log_replay = False
        seen_log_disarmed = False
        while True:
            m = dfreader.recv_match(type='PARM')
            if m is None:
                if not seen_log_replay:
                    return False
                if not seen_log_disarmed:
                    return False
                break
            if m.Name == "LOG_REPLAY":
                if seen_log_replay:
                    return False
                if m.Value != 1:
                    return False
                seen_log_replay = True
            if m.Name == "LOG_DISARMED":
                if seen_log_disarmed:
                    return False
                seen_log_disarmed = True
                if m.Value != 1:
                    return False
        return False

    def is_replay_log(self, logfile_path):
        '''returns true if an XKF1 or NKF1 message appears in first 5000
        messages'''
        dfreader =  DFReader.DFReader_binary(logfile_path, zero_time_base=True);
        count = 0
        while True:
            m = dfreader.recv_match()
            if m is None:
                break
            if m.get_type() == 'XKF1' and m.C >= 100:
                return True
            if m.get_type() == 'NKF1' and m.C >= 100:
                return True
            count += 1
            if count > 5000:
                return False
        return False

    def progress(self, message):
        print("CRB: %s" % message)

    def build_replay(self):
        subprocess.check_call(["./waf", "replay"])

    def run_replay_on_log(self, logfile_path):
        subprocess.check_call(["./build/sitl/tool/Replay", logfile_path])

    def get_logs(self):
        return sorted(glob.glob("logs/*.BIN"))

    def run_autotest_replay_on_master(self):
        # remember where we were:
        old_branch = self.repo.active_branch

        # check out the master branch:
        self.repo.head.reference = self.master
        self.repo.head.reset(index=True, working_tree=True)

        # generate logs:
        args = ["Tools/autotest/autotest.py"]

        if self.no_debug:
            args.append("--no-debug")
        else:
            args.append("--debug")

        if self.no_clean:
            args.append("--no-clean")

        args.extend(["build.Copter", "test.Copter.Replay"])

        subprocess.check_call(args) # actually run the test

        # check out the original branch:
        self.repo.head.reference = old_branch
        self.repo.head.reset(index=True, working_tree=True)

    def find_replayed_logs(self):
        '''find logs which were replayed in the autotest'''
        replayed_logs = set()
        for logfile_path in self.get_logs():
            self.progress("  Checking %s" % logfile_path)
            dfreader =  DFReader.DFReader_binary(logfile_path, zero_time_base=True);
            while True:
                m = dfreader.recv_match(type='MSG')
                if m is None:
                    break
                match = re.match(r".*Running replay on \(([^)]+)\).*", m.Message)
                if match is None:
                    continue
                replayed_logs.add(match.group(1))
        return sorted(list(replayed_logs))

    def run(self):
        self.topdir = self.find_topdir()
        self.repo = self.find_repo()
        self.assert_tree_clean()

        os.chdir(self.topdir)
        self.progress("chdir (%s)" % str(self.topdir))

        self.progress("Running autotest Replay on %s" % self.master)
        self.run_autotest_replay_on_master()

        self.progress("Building Replay")
        self.build_replay()
        self.progress("Build of Replay done")

        # check all replayable logs
        self.progress("Finding replayed logs")
        replay_logs = self.find_replayed_logs()
        success = True
        if len(replay_logs) == 0:
            raise ValueError("Found no Replay logs")
        for log in replay_logs:
            self.progress("Running Replay on (%s)" % log)
            old_logs = self.get_logs()
            self.run_replay_on_log(log)
            new_logs = self.get_logs()
            delta = [x for x in new_logs if x not in old_logs]
            if len(delta) != 1:
                raise ValueError("Expected a single new log")
            new_log = delta[0]
            self.progress("Running check_replay.py on Replay output log: %s" % new_log)

            # run check_replay across Replay log
            if check_replay.check_log(new_log, verbose=True):
                self.progress("check_replay.py of (%s): OK" % new_log)
            else:
                self.progress("check_replay.py of (%s): FAILED" % new_log)
                success = False
        if success:
            self.progress("All OK")
        else:
            self.progress("Failed")

        return success

if __name__ == '__main__':
    import sys
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--master", default='master', help="branch to consider master branch")
    parser.add_argument("--no-clean", action="store_true", help="do not clean SITL before building")
    parser.add_argument("--no-debug", action="store_true", help="do not make built SITL binaries debug binaries")

    args = parser.parse_args()

    s = CheckReplayBranch(master=args.master, no_clean=args.no_clean, no_debug=args.no_debug)
    if not s.run():
        sys.exit(1)

    sys.exit(0)
