from __future__ import annotations

'''
Base class for ArduPilot build scripts providing common utilities

AP_FLAKE8_CLEAN
'''

import os
import pathlib
import string
import subprocess
import sys
import time


class BuildScriptBase:
    """Base class for build scripts with common utilities for running programs"""

    def __init__(self):
        self.tmpdir = None  # Can be set by subclasses that need it

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

    def find_git_branch_merge_base(self, branch, master_branch):
        output = self.run_git(["merge-base", branch, master_branch])
        output = output.strip()
        return output

    def run_git(self, args, show_output=True, source_dir=None):
        '''run git with args git_args; returns git's output'''
        cmd_list = ["git"]
        cmd_list.extend(args)
        return self.run_program("SCB-GIT", cmd_list, show_output=show_output, cwd=source_dir)

    def run_waf(self, args, compiler=None, show_output=True, source_dir=None):
        # try to modify the environment so we can consistent builds:
        consistent_build_envs = {
            "CHIBIOS_GIT_VERSION": "12345678",
            "GIT_VERSION": "abcdef",
            "GIT_VERSION_EXTENDED": "0123456789abcdef",
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
                raise Exception("BB-WAF: Missing compiler %s" % gcc_path)
        self.run_program("SCB-WAF", cmd_list, env=env, show_output=show_output, cwd=source_dir)

    def progress(self, string):
        '''pretty-print progress'''
        print(f"{self.progress_prefix()}: {string}", file=sys.stderr)
