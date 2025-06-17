#!/usr/bin/env python3

'''
Look at the difference between this and its merge-base with master

for each modified (or new) hwdef file, build the board
'''

import os
import optparse
import pathlib
import string
import subprocess
import time

from typing import Set

import board_list


class TestModifiedBoards():
    def __init__(self, master_branch : str = "master") -> None:
        self.master_branch : str = master_branch

    def progress(self, string : str) -> None:
        '''pretty-print progress'''
        print("TMB: %s" % string)

    def run_waf(self, args, compiler=None, show_output=True, source_dir=None):
        # try to modify the environment so we can consistent builds:
        consistent_build_envs = {
            "CHIBIOS_GIT_VERSION": "12345678",
            "GIT_VERSION": "abcdef",
            "GIT_VERSION_INT": "15",
        }
        for (n, v) in consistent_build_envs.items():
            os.environ[n] = v

        if os.path.exists("waf"):
            waf = "./waf"
        else:
            waf = os.path.join(".", "modules", "waf", "waf-light")
        cmd_list = [waf]
        cmd_list.extend(args)
        env = None
        if compiler is not None:
            # default to $HOME/arm-gcc, but allow for any path with AP_GCC_HOME environment variable
            gcc_home = os.environ.get("AP_GCC_HOME", os.path.join(os.environ["HOME"], "arm-gcc"))
            gcc_path = os.path.join(gcc_home, compiler, "bin")
            if os.path.exists(gcc_path):
                # setup PATH to point at the right compiler, and setup to use ccache
                env = os.environ.copy()
                env["PATH"] = gcc_path + ":" + env["PATH"]
                env["CC"] = "ccache arm-none-eabi-gcc"
                env["CXX"] = "ccache arm-none-eabi-g++"
            else:
                raise Exception("TMB-WAF: Missing compiler %s" % gcc_path)
        self.run_program("TMB-WAF", cmd_list, env=env, show_output=show_output, cwd=source_dir)

    def run_program(self, prefix, cmd_list, show_output=True, env=None, show_output_on_error=True, show_command=None, cwd="."):
        if show_command is None:
            show_command = True

        cmd = " ".join(cmd_list)
        if cwd is None:
            cwd = "."
        command_debug = f"Running ({cmd}) in ({cwd})"
        process_failure_content = command_debug + "\n"
        if show_command:
            self.progress(command_debug)

        p = subprocess.Popen(
            cmd_list,
            stdin=None,
            stdout=subprocess.PIPE,
            close_fds=True,
            stderr=subprocess.STDOUT,
            cwd=cwd,
            env=env)
        output = ""
        while True:
            x = p.stdout.readline()
            if len(x) == 0:
                returncode = os.waitpid(p.pid, 0)
                if returncode:
                    break
                    # select not available on Windows... probably...
                time.sleep(0.1)
                continue
            x = bytearray(x)
            x = filter(lambda x : chr(x) in string.printable, x)
            x = "".join([chr(c) for c in x])
            output += x
            process_failure_content += x
            x = x.rstrip()
            some_output = "%s: %s" % (prefix, x)
            if show_output:
                print(some_output)
        (_, status) = returncode
        if status != 0:
            if not show_output and show_output_on_error:
                # we were told not to show output, but we just
                # failed... so show output...
                print(output)
            self.progress("Process failed (%s)" %
                          str(returncode))
            try:
                path = pathlib.Path(self.tmpdir, f"process-failure-{int(time.time())}")
                path.write_text(process_failure_content)
                self.progress("Wrote process failure file (%s)" % path)
            except Exception:
                self.progress("Writing process failure file failed")
            raise subprocess.CalledProcessError(
                returncode, cmd_list)
        return output

    def run_git(self, args, show_output=True, source_dir=None):
        '''run git with args git_args; returns git's output'''
        cmd_list = ["git"]
        cmd_list.extend(args)
        return self.run_program("TMB-git", cmd_list, show_output=show_output, cwd=source_dir)

    def find_git_branch_merge_base(self, branch, master_branch):
        output = self.run_git(["merge-base", branch, master_branch])
        output = output.strip()
        return output

    def find_current_git_branch_or_sha1(self):
        try:
            output = self.run_git(["symbolic-ref", "--short", "HEAD"])
            output = output.strip()
            return output
        except subprocess.CalledProcessError:
            pass

        # probably in a detached-head state.  Get a sha1 instead:
        output = self.run_git(["rev-parse", "--short", "HEAD"])
        output = output.strip()
        return output

    def get_modifed_hwdef_paths(self) -> Set[str]:
        '''returns list of hwdef filepaths relative to the root.  A
        mixture of .inc and .dat files, for both main and bootloader
        firmwares'''
        current_commitish = self.find_current_git_branch_or_sha1()
        merge_base = self.find_git_branch_merge_base(current_commitish, self.master_branch)
        delta_files = self.run_git([
            'diff',
            '--name-only',
            f"{merge_base}",
        ], show_output=False)

        ret = set()

        for f in delta_files.split("\n"):
            if "/hwdef/" not in f:
                continue
            if not f.endswith(("hwdef.dat", "hwdef.inc", "hwdef-bl.dat", "hwdef-bl.inc")):
                continue

            ret.add(f)

        return ret

    def build_board(self, board : board_list.Board):
        self.run_waf(["configure", "--board", board.name], show_output=False)
        if board.is_ap_periph:
            self.run_waf(["AP_Periph"], show_output=False)
        else:
            for t in board.autobuild_targets:
                self.run_waf([t], show_output=False)

    def build_bootloader(self, board : board_list.Board) -> None:
        self.run_waf([
            "configure",
            "--board",
            board.name,
            "--bootloader"
        ], show_output=False)
        self.run_waf(["bootloader"], show_output=False)

    def run(self) -> None:
        hwdef_filepaths = self.get_modifed_hwdef_paths()
        for hwdef_filepath in hwdef_filepaths:
            self.progress(f"Modified hwdef: {hwdef_filepath}")
            bl = board_list.BoardList()
            bootloader_blacklist = bl.bootloader_blacklist
            for board in bl.boards:
                if board.uses_hwdef(hwdef_filepath):
                    self.progress(f"{board.name} uses modified hwdef")
                    self.build_board(board)
                if board.name not in bootloader_blacklist:
                    if board.uses_hwdef_bootloader(hwdef_filepath):
                        self.progress(f"{board.name} bootloader uses modfied hwdef")
                        self.build_bootloader(board)


def main():
    parser = optparse.OptionParser("size_compare_branches.py")
    parser.add_option("",
                      "--master-branch",
                      type="string",
                      default="master",
                      help="master branch to use")
    cmd_opts, cmd_args = parser.parse_args()

    tmb = TestModifiedBoards(
        master_branch=cmd_opts.master_branch,
    )
    tmb.run()


if __name__ == '__main__':
    main()
