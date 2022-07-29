#!/usr/bin/env python

"""
script to build the latest binaries for each vehicle type, ready to upload
Peter Barker, August 2017
based on build_binaries.sh by Andrew Tridgell, March 2013

AP_FLAKE8_CLEAN
"""

from __future__ import print_function

import datetime
import optparse
import os
import re
import shutil
import time
import string
import subprocess
import sys
import traceback

# local imports
import generate_manifest
import gen_stable
import build_binaries_history
import extract_features

import board_list
from board_list import AP_PERIPH_BOARDS

if sys.version_info[0] < 3:
    running_python3 = False
else:
    running_python3 = True


def is_chibios_build(board):
    '''see if a board is using HAL_ChibiOS'''
    # cope with both running from Tools/scripts or running from cwd
    hwdef_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "libraries", "AP_HAL_ChibiOS", "hwdef")
    if os.path.exists(os.path.join(hwdef_dir, board, "hwdef.dat")):
        return True
    hwdef_dir = os.path.join("libraries", "AP_HAL_ChibiOS", "hwdef")
    if os.path.exists(os.path.join(hwdef_dir, board, "hwdef.dat")):
        return True
    return False


def get_required_compiler(vehicle, tag, board):
    '''return required compiler for a build tag.
       return format is the version string that waf configure will detect.
       You should setup a link from this name in $HOME/arm-gcc directory pointing at the
       appropriate compiler
    '''
    if not is_chibios_build(board):
        # only override compiler for ChibiOS builds
        return None
    if vehicle == 'Sub' and tag in ['stable', 'beta']:
        # sub stable and beta is on the old compiler
        return "g++-6.3.1"
    # use 10.2.1 compiler for all other builds
    return "g++-10.2.1"


class build_binaries(object):
    def __init__(self, tags):
        self.tags = tags
        self.dirty = False
        self.board_list = board_list.BoardList()

    def progress(self, string):
        '''pretty-print progress'''
        print("BB: %s" % string)

    def run_git(self, args):
        '''run git with args git_args; returns git's output'''
        cmd_list = ["git"]
        cmd_list.extend(args)
        return self.run_program("BB-GIT", cmd_list)

    def board_branch_bit(self, board):
        '''return a fragment which might modify the branch name.
        this was previously used to have a master-AVR branch etc
        if the board type was apm1 or apm2'''
        return None

    def board_options(self, board):
        '''return board-specific options'''
        if board in ["bebop", "disco"]:
            return ["--static"]
        return []

    def run_waf(self, args, compiler=None):
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
        self.run_program("BB-WAF", cmd_list, env=env)

    def run_program(self, prefix, cmd_list, show_output=True, env=None):
        if show_output:
            self.progress("Running (%s)" % " ".join(cmd_list))
        p = subprocess.Popen(cmd_list, bufsize=1, stdin=None,
                             stdout=subprocess.PIPE, close_fds=True,
                             stderr=subprocess.STDOUT, env=env)
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
            if running_python3:
                x = bytearray(x)
                x = filter(lambda x : chr(x) in string.printable, x)
                x = "".join([chr(c) for c in x])
            output += x
            x = x.rstrip()
            if show_output:
                print("%s: %s" % (prefix, x))
        (_, status) = returncode
        if status != 0 and show_output:
            self.progress("Process failed (%s)" %
                          str(returncode))
            raise subprocess.CalledProcessError(
                returncode, cmd_list)
        return output

    def run_make(self, args):
        cmd_list = ["make"]
        cmd_list.extend(args)
        self.run_program("BB-MAKE", cmd_list)

    def run_git_update_submodules(self):
        '''if submodules are present initialise and update them'''
        if os.path.exists(os.path.join(self.basedir, ".gitmodules")):
            self.run_git(["submodule",
                          "update",
                          "--init",
                          "--recursive",
                          "-f"])

    def checkout(self, vehicle, ctag, cboard=None, cframe=None, submodule_update=True):
        '''attempt to check out a git tree.  Various permutations are
attempted based on ctag - for examplle, if the board is avr and ctag
is bob we will attempt to checkout bob-AVR'''
        if self.dirty:
            self.progress("Skipping checkout for dirty build")
            return True

        self.progress("Trying checkout %s %s %s %s" %
                      (vehicle, ctag, cboard, cframe))
        self.run_git(['stash'])
        if ctag == "latest":
            vtag = "master"
        else:
            tagvehicle = vehicle
            if tagvehicle == "Rover":
                # FIXME: Rover tags in git still named APMrover2 :-(
                tagvehicle = "APMrover2"
            vtag = "%s-%s" % (tagvehicle, ctag)

        branches = []
        if cframe is not None:
            # try frame specific tag
            branches.append("%s-%s" % (vtag, cframe))
        if cboard is not None:
            bbb = self.board_branch_bit(cboard)
            if bbb is not None:
                # try board type specific branch extension
                branches.append("".join([vtag, bbb]))
        branches.append(vtag)

        for branch in branches:
            try:
                self.progress("Trying branch %s" % branch)
                self.run_git(["checkout", "-f", branch])
                if submodule_update:
                    self.run_git_update_submodules()
                self.run_git(["log", "-1"])
                return True
            except subprocess.CalledProcessError:
                self.progress("Checkout branch %s failed" % branch)

        self.progress("Failed to find tag for %s %s %s %s" %
                      (vehicle, ctag, cboard, cframe))
        return False

    def skip_board_waf(self, board):
        '''check if we should skip this build because we do not support the
        board in this release
        '''

        try:
            out = self.run_program('waf', ['./waf', 'configure', '--board=BOARDTEST'], False)
            lines = out.split('\n')
            needles = ["BOARDTEST' (choose from", "BOARDTEST': choices are"]
            for line in lines:
                for needle in needles:
                    idx = line.find(needle)
                    if idx != -1:
                        break
                if idx != -1:
                    line = line[idx+len(needle):-1]
                    line = line.replace("'", "")
                    line = line.replace(" ", "")
                    boards = line.split(",")
                    ret = board not in boards
                    if ret:
                        self.progress("Skipping board (%s) - not in board list" % board)
                    return ret
        except IOError as e:
            if e.errno != 2:
                raise

        self.progress("Skipping unsupported board %s" % (board,))
        return True

    def skip_frame(self, board, frame):
        '''returns true if this board/frame combination should not be built'''
        if frame == "heli":
            if board in ["bebop", "aerofc-v1", "skyviper-v2450", "CubeSolo", "CubeGreen-solo", 'skyviper-journey']:
                self.progress("Skipping heli build for %s" % board)
                return True
        return False

    def first_line_of_filepath(self, filepath):
        '''returns the first (text) line from filepath'''
        with open(filepath) as fh:
            line = fh.readline()
        return line

    def skip_build(self, buildtag, builddir):
        '''check if we should skip this build because we have already built
        this version
        '''

        if os.getenv("FORCE_BUILD", False):
            return False

        if not os.path.exists(os.path.join(self.basedir, '.gitmodules')):
            self.progress("Skipping build without submodules")
            return True

        bname = os.path.basename(builddir)
        ldir = os.path.join(os.path.dirname(os.path.dirname(
            os.path.dirname(builddir))), buildtag, bname)  # FIXME: WTF

        oldversion_filepath = os.path.join(ldir, "git-version.txt")
        if not os.path.exists(oldversion_filepath):
            self.progress("%s doesn't exist - building" % oldversion_filepath)
            return False

        oldversion = self.first_line_of_filepath(oldversion_filepath)
        newversion = self.run_git(["log", "-1"])
        newversion = newversion.splitlines()[0]
        oldversion = oldversion.rstrip()
        newversion = newversion.rstrip()
        self.progress("oldversion=%s newversion=%s" %
                      (oldversion, newversion,))
        if oldversion == newversion:
            self.progress("Skipping build - version match (%s)" %
                          (newversion,))
            return True

        self.progress("%s needs rebuild" % (ldir,))
        return False

    def write_string_to_filepath(self, string, filepath):
        '''writes the entirety of string to filepath'''
        with open(filepath, "w") as x:
            x.write(string)

    def version_h_path(self, src):
        '''return path to version.h'''
        if src == 'AP_Periph':
            return os.path.join('Tools', src, "version.h")
        return os.path.join(src, "version.h")

    def addfwversion_gitversion(self, destdir, src):
        # create git-version.txt:
        gitlog = self.run_git(["log", "-1"])
        gitversion_filepath = os.path.join(destdir, "git-version.txt")
        gitversion_content = gitlog
        versionfile = self.version_h_path(src)
        if os.path.exists(versionfile):
            content = self.read_string_from_filepath(versionfile)
            match = re.search('define.THISFIRMWARE "([^"]+)"', content)
            if match is None:
                self.progress("Failed to retrieve THISFIRMWARE from version.h")
                self.progress("Content: (%s)" % content)
            self.progress("Writing version info to %s" %
                          (gitversion_filepath,))
            gitversion_content += "\nAPMVERSION: %s\n" % (match.group(1))
        else:
            self.progress("%s does not exist" % versionfile)

        self.write_string_to_filepath(gitversion_content, gitversion_filepath)

    def addfwversion_firmwareversiontxt(self, destdir, src):
        # create firmware-version.txt
        versionfile = self.version_h_path(src)
        if not os.path.exists(versionfile):
            self.progress("%s does not exist" % (versionfile,))
            return
        ss = r".*define +FIRMWARE_VERSION[	 ]+(?P<major>\d+)[ ]*,[ 	]*" \
             r"(?P<minor>\d+)[ ]*,[	 ]*(?P<point>\d+)[ ]*,[	 ]*" \
             r"(?P<type>[A-Z_]+)[	 ]*"
        content = self.read_string_from_filepath(versionfile)
        match = re.search(ss, content)
        if match is None:
            self.progress("Failed to retrieve FIRMWARE_VERSION from version.h")
            self.progress("Content: (%s)" % content)
            return
        ver = "%d.%d.%d-%s\n" % (int(match.group("major")),
                                 int(match.group("minor")),
                                 int(match.group("point")),
                                 match.group("type"))
        firmware_version_filepath = "firmware-version.txt"
        self.progress("Writing version (%s) to %s" %
                      (ver, firmware_version_filepath,))
        self.write_string_to_filepath(
            ver, os.path.join(destdir, firmware_version_filepath))

    def addfwversion(self, destdir, src):
        '''write version information into destdir'''
        self.addfwversion_gitversion(destdir, src)
        self.addfwversion_firmwareversiontxt(destdir, src)

    def read_string_from_filepath(self, filepath):
        '''returns content of filepath as a string'''
        with open(filepath, 'rb') as fh:
            content = fh.read()

        if running_python3:
            return content.decode('ascii')

        return content

    def string_in_filepath(self, string, filepath):
        '''returns true if string exists in the contents of filepath'''
        return string in self.read_string_from_filepath(filepath)

    def mkpath(self, path):
        '''make directory path and all elements leading to it'''
        '''distutils.dir_util.mkpath was playing up'''
        try:
            os.makedirs(path)
        except OSError as e:
            if e.errno != 17:  # EEXIST
                raise e

    def touch_filepath(self, filepath):
        '''creates a file at filepath, or updates the timestamp on filepath'''
        if os.path.exists(filepath):
            os.utime(filepath, None)
        else:
            with open(filepath, "a"):
                pass

    def build_vehicle(self, tag, vehicle, boards, vehicle_binaries_subdir,
                      binaryname, frames=[None]):
        '''build vehicle binaries'''
        self.progress("Building %s %s binaries (cwd=%s)" %
                      (vehicle, tag, os.getcwd()))

        board_count = len(boards)
        count = 0
        for board in sorted(boards, key=str.lower):
            now = datetime.datetime.now()
            count += 1
            self.progress("[%u/%u] Building board: %s at %s" %
                          (count, board_count, board, str(now)))
            for frame in frames:
                if frame is not None:
                    self.progress("Considering frame %s for board %s" %
                                  (frame, board))
                if frame is None:
                    framesuffix = ""
                else:
                    framesuffix = "-%s" % frame
                if not self.checkout(vehicle, tag, board, frame, submodule_update=False):
                    msg = ("Failed checkout of %s %s %s %s" %
                           (vehicle, board, tag, frame,))
                    self.progress(msg)
                    self.error_strings.append(msg)
                    continue

                self.progress("Building %s %s %s binaries %s" %
                              (vehicle, tag, board, frame))
                ddir = os.path.join(self.binaries,
                                    vehicle_binaries_subdir,
                                    self.hdate_ym,
                                    self.hdate_ymdhm,
                                    "".join([board, framesuffix]))
                if self.skip_build(tag, ddir):
                    continue
                if self.skip_frame(board, frame):
                    continue

                # we do the submodule update after the skip_board_waf check to avoid doing it on
                # builds we will not be running
                self.run_git_update_submodules()

                if self.skip_board_waf(board):
                    continue

                if os.path.exists(self.buildroot):
                    shutil.rmtree(self.buildroot)

                self.remove_tmpdir()

                githash = self.run_git(["rev-parse", "HEAD"]).rstrip()

                t0 = time.time()

                self.progress("Configuring for %s in %s" %
                              (board, self.buildroot))
                try:
                    waf_opts = ["configure",
                                "--board", board,
                                "--out", self.buildroot,
                                "clean"]
                    gccstring = get_required_compiler(vehicle, tag, board)
                    if gccstring is not None and gccstring.find("g++-6.3") == -1:
                        # versions using the old compiler don't have the --assert-cc-version option
                        waf_opts += ["--assert-cc-version", gccstring]

                    waf_opts.extend(self.board_options(board))
                    self.run_waf(waf_opts, compiler=gccstring)
                except subprocess.CalledProcessError:
                    self.progress("waf configure failed")
                    continue

                time_taken_to_configure = time.time() - t0

                try:
                    target = os.path.join("bin",
                                          "".join([binaryname, framesuffix]))
                    self.run_waf(["build", "--targets", target], compiler=gccstring)
                except subprocess.CalledProcessError:
                    msg = ("Failed build of %s %s%s %s" %
                           (vehicle, board, framesuffix, tag))
                    self.progress(msg)
                    self.error_strings.append(msg)
                    # record some history about this build
                    t1 = time.time()
                    time_taken_to_build = t1-t0
                    self.history.record_build(githash, tag, vehicle, board, frame, None, t0, time_taken_to_build)
                    continue

                time_taken_to_build = (time.time()-t0) - time_taken_to_configure

                time_taken = time.time()-t0
                self.progress("Making %s %s %s %s took %u seconds (configure=%u build=%u)" %
                              (vehicle, tag, board, frame, time_taken, time_taken_to_configure, time_taken_to_build))

                bare_path = os.path.join(self.buildroot,
                                         board,
                                         "bin",
                                         "".join([binaryname, framesuffix]))
                files_to_copy = []
                extensions = [".apj", ".abin", "_with_bl.hex", ".hex"]
                if vehicle == 'AP_Periph':
                    # need bin file for uavcan-gui-tool and MissionPlanner
                    extensions.append('.bin')
                for extension in extensions:
                    filepath = "".join([bare_path, extension])
                    if os.path.exists(filepath):
                        files_to_copy.append((filepath, os.path.basename(filepath)))
                if not os.path.exists(bare_path):
                    raise Exception("No elf file?!")

                ef = extract_features.ExtractFeatures(bare_path)
                features_text = ef.extract()

                # only rename the elf if we have have other files to
                # copy.  So linux gets "arducopter" and stm32 gets
                # "arducopter.elf"
                target_elf_filename = os.path.basename(bare_path)
                if len(files_to_copy) > 0:
                    target_elf_filename += ".elf"
                files_to_copy.append((bare_path, target_elf_filename))

                for (path, target_filename) in files_to_copy:
                    try:
                        '''copy path into various places, adding metadata'''
                        bname = os.path.basename(ddir)
                        tdir = os.path.join(os.path.dirname(os.path.dirname(
                            os.path.dirname(ddir))), tag, bname)
                        if tag == "latest":
                            # we keep a permanent archive of all
                            # "latest" builds, their path including a
                            # build timestamp:
                            if not os.path.exists(ddir):
                                self.mkpath(ddir)
                            self.addfwversion(ddir, vehicle)
                            self.write_string_to_filepath(features_text, os.path.join(ddir, "features.txt"))
                            self.progress("Copying %s to %s" % (path, ddir,))
                            shutil.copy(path, os.path.join(ddir, target_filename))
                        # the most recent build of every tag is kept around:
                        self.progress("Copying %s to %s" % (path, tdir))
                        if not os.path.exists(tdir):
                            self.mkpath(tdir)
                        # must addfwversion even if path already
                        # exists as we re-use the "beta" directories
                        self.addfwversion(tdir, vehicle)
                        self.write_string_to_filepath(features_text, os.path.join(ddir, "features.txt"))
                        shutil.copy(path, os.path.join(tdir, target_filename))
                    except Exception as e:
                        self.print_exception_caught(e)
                        self.progress("Failed to copy %s to %s: %s" % (path, ddir, str(e)))
                # why is touching this important? -pb20170816
                self.touch_filepath(os.path.join(self.binaries,
                                                 vehicle_binaries_subdir, tag))

                # record some history about this build
                self.history.record_build(githash, tag, vehicle, board, frame, bare_path, t0, time_taken_to_build)

        self.checkout(vehicle, "latest")

    def get_exception_stacktrace(self, e):
        if sys.version_info[0] >= 3:
            ret = "%s\n" % e
            ret += ''.join(traceback.format_exception(type(e),
                                                      e,
                                                      tb=e.__traceback__))
            return ret

        # Python2:
        return traceback.format_exc(e)

    def print_exception_caught(self, e, send_statustext=True):
        self.progress("Exception caught: %s" %
                      self.get_exception_stacktrace(e))

    def AP_Periph_boards(self):
        return AP_PERIPH_BOARDS

    def build_arducopter(self, tag):
        '''build Copter binaries'''

        boards = []
        boards.extend(["aerofc-v1", "bebop"])
        boards.extend(self.board_list.find_autobuild_boards('Copter'))
        self.build_vehicle(tag,
                           "ArduCopter",
                           boards,
                           "Copter",
                           "arducopter",
                           frames=[None, "heli"])

    def build_arduplane(self, tag):
        '''build Plane binaries'''
        boards = self.board_list.find_autobuild_boards('Plane')[:]
        boards.append("disco")
        self.build_vehicle(tag,
                           "ArduPlane",
                           boards,
                           "Plane",
                           "arduplane")

    def build_antennatracker(self, tag):
        '''build Tracker binaries'''
        self.build_vehicle(tag,
                           "AntennaTracker",
                           self.board_list.find_autobuild_boards('Tracker')[:],
                           "AntennaTracker",
                           "antennatracker")

    def build_rover(self, tag):
        '''build Rover binaries'''
        self.build_vehicle(tag,
                           "Rover",
                           self.board_list.find_autobuild_boards('Rover')[:],
                           "Rover",
                           "ardurover")

    def build_ardusub(self, tag):
        '''build Sub binaries'''
        self.build_vehicle(tag,
                           "ArduSub",
                           self.board_list.find_autobuild_boards('Sub')[:],
                           "Sub",
                           "ardusub")

    def build_AP_Periph(self, tag):
        '''build AP_Periph binaries'''
        boards = self.AP_Periph_boards()
        self.build_vehicle(tag,
                           "AP_Periph",
                           boards,
                           "AP_Periph",
                           "AP_Periph")

    def build_blimp(self, tag):
        '''build Blimp binaries'''
        self.build_vehicle(tag,
                           "Blimp",
                           self.board_list.find_autobuild_boards('Blimp')[:],
                           "Blimp",
                           "blimp")

    def generate_manifest(self):
        '''generate manigest files for GCS to download'''
        self.progress("Generating manifest")
        base_url = 'https://firmware.ardupilot.org'
        generator = generate_manifest.ManifestGenerator(self.binaries,
                                                        base_url)
        generator.run()

        generator.write_manifest_json(os.path.join(self.binaries, "manifest.json"))
        generator.write_features_json(os.path.join(self.binaries, "features.json"))
        self.progress("Manifest generation successful")

        self.progress("Generating stable releases")
        gen_stable.make_all_stable(self.binaries)
        self.progress("Generate stable releases done")

    def validate(self):
        '''run pre-run validation checks'''
        if "dirty" in self.tags:
            if len(self.tags) > 1:
                raise ValueError("dirty must be only tag if present (%s)" %
                                 (str(self.tags)))
            self.dirty = True

    def remove_tmpdir(self):
        if os.path.exists(self.tmpdir):
            self.progress("Removing (%s)" % (self.tmpdir,))
            shutil.rmtree(self.tmpdir)

    def buildlogs_dirpath(self):
        return os.getenv("BUILDLOGS",
                         os.path.join(os.getcwd(), "..", "buildlogs"))

    def run(self):
        self.validate()

        self.mkpath(self.buildlogs_dirpath())

        binaries_history_filepath = os.path.join(
            self.buildlogs_dirpath(), "build_binaries_history.sqlite")
        self.history = build_binaries_history.BuildBinariesHistory(binaries_history_filepath)

        prefix_bin_dirpath = os.path.join(os.environ.get('HOME'),
                                          "prefix", "bin")
        origin_env_path = os.environ.get("PATH")
        os.environ["PATH"] = ':'.join([prefix_bin_dirpath, origin_env_path,
                                       "/bin", "/usr/bin"])
        if 'BUILD_BINARIES_PATH' in os.environ:
            self.tmpdir = os.environ['BUILD_BINARIES_PATH']
        else:
            self.tmpdir = os.path.join(os.getcwd(), 'build.tmp.binaries')
        os.environ["TMPDIR"] = self.tmpdir

        print(self.tmpdir)
        self.remove_tmpdir()

        self.progress("Building in %s" % self.tmpdir)

        now = datetime.datetime.now()
        self.progress(now)

        if not self.dirty:
            self.run_git(["checkout", "-f", "master"])
        githash = self.run_git(["rev-parse", "HEAD"])
        githash = githash.rstrip()
        self.progress("git hash: %s" % str(githash))

        self.hdate_ym = now.strftime("%Y-%m")
        self.hdate_ymdhm = now.strftime("%Y-%m-%d-%H:%m")

        self.mkpath(os.path.join("binaries", self.hdate_ym,
                                 self.hdate_ymdhm))
        self.binaries = os.path.join(self.buildlogs_dirpath(), "binaries")
        self.basedir = os.getcwd()
        self.error_strings = []

        if not self.dirty:
            self.run_git_update_submodules()
        self.buildroot = os.path.join(os.environ.get("TMPDIR"),
                                      "binaries.build")

        for tag in self.tags:
            t0 = time.time()
            self.build_arducopter(tag)
            self.build_arduplane(tag)
            self.build_rover(tag)
            self.build_antennatracker(tag)
            self.build_ardusub(tag)
            self.build_AP_Periph(tag)
            self.build_blimp(tag)
            self.history.record_run(githash, tag, t0, time.time()-t0)

        if os.path.exists(self.tmpdir):
            shutil.rmtree(self.tmpdir)

        self.generate_manifest()

        for error_string in self.error_strings:
            self.progress("%s" % error_string)
        sys.exit(len(self.error_strings))


if __name__ == '__main__':
    parser = optparse.OptionParser("build_binaries.py")

    parser.add_option("", "--tags", action="append", type="string",
                      default=[], help="tags to build")
    cmd_opts, cmd_args = parser.parse_args()

    tags = cmd_opts.tags
    if len(tags) == 0:
        # FIXME: wedge this defaulting into parser somehow
        tags = ["stable", "beta", "latest"]

    bb = build_binaries(tags)
    bb.run()
