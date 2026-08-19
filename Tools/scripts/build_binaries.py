#!/usr/bin/env python3

"""
script to build the latest binaries for each vehicle type, ready to upload
Peter Barker, August 2017
Amilcar Lucas, October 2025 - added parameter metadata XML generation
based on build_binaries.sh by Andrew Tridgell, March 2013

AP_FLAKE8_CLEAN
"""

from __future__ import annotations

import datetime
import gzip
import lzma
import optparse
import os
import pathlib
import re
import shutil
import string
import subprocess
import sys
import tempfile
import time
import traceback

import board_list
import build_binaries_history
import gen_stable

# local imports
import generate_manifest

from board_list import AP_PERIPH_BOARDS


def topdir():
    '''return path to ardupilot checkout directory.  This is to cope with
    running on developer's machines (where autotest is typically
    invoked from the root directory), and on the autotest server where
    it is invoked in the checkout's parent directory.
    '''
    for path in [
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."),
            "",
            ]:
        if os.path.exists(os.path.join(path, "libraries", "AP_HAL_ChibiOS")):
            return path
    raise Exception("Unable to find ardupilot checkout dir")


def is_chibios_build(board):
    '''see if a board is using HAL_ChibiOS'''
    # cope with both running from Tools/scripts or running from cwd
    hwdef_dir = os.path.join(topdir(), "libraries", "AP_HAL_ChibiOS", "hwdef")

    return os.path.exists(os.path.join(hwdef_dir, board, "hwdef.dat"))


def get_required_compiler(vehicle, tag, board):
    '''return required compiler for a build tag.
       return format is the version string that waf configure will detect.
       You should setup a link from this name in $HOME/arm-gcc directory pointing at the
       appropriate compiler
    '''
    if not is_chibios_build(board):
        # only override compiler for ChibiOS builds
        return None
    # use 10.2.1 compiler for all other builds
    return "g++-10.2.1"


class build_binaries(object):
    # Metadata name: (vehicle source directory, binaries subdirectory).
    metadata_vehicles = {
        'Copter': ('ArduCopter', 'Copter'),
        'Plane': ('ArduPlane', 'Plane'),
        'Rover': ('Rover', 'Rover'),
        'Tracker': ('AntennaTracker', 'AntennaTracker'),
        'Sub': ('ArduSub', 'Sub'),
        'Blimp': ('Blimp', 'Blimp'),
        'AP_Periph': ('AP_Periph', 'AP_Periph'),
    }
    def __init__(self, tags, metadata_only=False, metadata_vehicles=None):
        self.tags = tags
        self.dirty = False
        self.metadata_only = metadata_only
        self.metadata_vehicles_to_generate = metadata_vehicles
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
        cmd_list = ["python3", waf]
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

    def run_program(self, prefix, cmd_list, show_output=True, env=None, force_success=False, cwd=None):
        if show_output:
            self.progress("Running (%s)" % " ".join(cmd_list))
        p = subprocess.Popen(cmd_list, stdin=None,
                              stdout=subprocess.PIPE, close_fds=True,
                              stderr=subprocess.STDOUT, env=env, cwd=cwd)
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
            x = x.rstrip()
            if show_output:
                print("%s: %s" % (prefix, x))
        (_, status) = returncode
        if status != 0 and not force_success:
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
            out = self.run_program(
                'waf',
                ["python3", './waf', 'configure', '--board=BOARDTEST'],
                show_output=False,
                force_success=True
            )
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
            content = pathlib.Path(versionfile).read_text(encoding='ascii')
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
        content = pathlib.Path(versionfile).read_text(encoding='ascii')
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
                      binaryname, frames: list | None = None):
        '''build vehicle binaries'''
        if frames is None:
            frames = [None]
        tag_dir = os.path.join(self.binaries, vehicle_binaries_subdir, tag)
        self.progress("Building %s %s binaries (cwd=%s)" %
                      (vehicle, tag, os.getcwd()))

        board_count = len(boards)
        count = 0
        built_any = False
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
                    built_any = True
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
                if vehicle == 'AP_Periph' or board == "Here4FC":
                    # need bin file for uavcan-gui-tool and MissionPlanner
                    extensions.append('.bin')
                for extension in extensions:
                    filepath = "".join([bare_path, extension])
                    if os.path.exists(filepath):
                        files_to_copy.append((filepath, os.path.basename(filepath)))
                if not os.path.exists(bare_path):
                    raise Exception("No elf file?!")

                # attempt to run an extract_features.py to create features.txt:
                features_text = None
                ef_path = os.path.join(topdir(), "Tools", "scripts", "extract_features.py")
                if os.path.exists(ef_path):
                    try:
                        features_text = self.run_program("EF", [ef_path, bare_path], show_output=False)
                    except Exception as e:  # noqa: BLE001
                        self.print_exception_caught(e)
                        self.progress("Failed to extract features")
                        pass
                else:
                    self.progress("Not extracting features as (%s) does not exist" % (ef_path,))

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
                        tdir = os.path.join(tag_dir, bname)
                        if tag == "latest":
                            # we keep a permanent archive of all
                            # "latest" builds, their path including a
                            # build timestamp:
                            if not os.path.exists(ddir):
                                self.mkpath(ddir)
                            self.addfwversion(ddir, vehicle)
                            features_filepath = os.path.join(ddir, "features.txt",)
                            if features_text is not None:
                                self.progress("Writing (%s)" % features_filepath)
                                self.write_string_to_filepath(features_text, features_filepath)
                            self.progress("Copying %s to %s" % (path, ddir,))
                            shutil.copy(path, os.path.join(ddir, target_filename))
                        # the most recent build of every tag is kept around:
                        self.progress("Copying %s to %s" % (path, tdir))
                        if not os.path.exists(tdir):
                            self.mkpath(tdir)
                        # must addfwversion even if path already
                        # exists as we reuse the "beta" directories
                        self.addfwversion(tdir, vehicle)
                        features_filepath = os.path.join(tdir, "features.txt")
                        if features_text is not None:
                            self.progress("Writing (%s)" % features_filepath)
                            self.write_string_to_filepath(features_text, features_filepath)
                        shutil.copy(path, os.path.join(tdir, target_filename))
                    except Exception as e:  # noqa: BLE001
                        self.print_exception_caught(e)
                        self.progress("Failed to copy %s to %s: %s" % (path, tdir, str(e)))
                # why is touching this important? -pb20170816
                self.touch_filepath(tag_dir)

                # record some history about this build
                self.history.record_build(githash, tag, vehicle, board, frame, bare_path, t0, time_taken_to_build)

        if built_any:
            if not self.checkout(vehicle, tag, submodule_update=False):
                msg = "Failed metadata checkout for %s %s" % (vehicle, tag)
                self.progress(msg)
                self.error_strings.append(msg)
            elif not self.generate_parameter_metadata_for_vehicle(tag, vehicle, tag_dir):
                msg = "Failed metadata generation for %s %s" % (vehicle, tag)
                self.progress(msg)
                self.error_strings.append(msg)

        self.checkout(vehicle, "latest")

    def _get_exception_stacktrace(self, e):
        if sys.version_info[0] >= 3:
            ret = "%s\n" % e
            ret += ''.join(traceback.format_exception(type(e),
                                                      e,
                                                      tb=e.__traceback__))
            return ret

        # Python2:
        return traceback.format_exc(e)

    def get_exception_stacktrace(self, e):
        try:
            return self._get_exception_stacktrace(e)
        except Exception:  # noqa: BLE001 — defensive wrapper, must not raise
            return "FAILED TO GET EXCEPTION STACKTRACE"

    def print_exception_caught(self, e, send_statustext=True):
        self.progress("Exception caught: %s" %
                      self.get_exception_stacktrace(e))

    def AP_Periph_boards(self):
        return AP_PERIPH_BOARDS

    def build_arducopter(self, tag):
        '''build Copter binaries'''

        boards = []
        boards.extend(["aerofc-v1"])
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
        if tag == "stable":
            self.progress("Skipping Blimp stable build")
            return
        self.build_vehicle(tag,
                           "Blimp",
                           self.board_list.find_autobuild_boards('Blimp')[:],
                           "Blimp",
                           "blimp")

    def generate_manifest(self):
        '''generate manifest files for GCS to download'''
        self.progress("Generating stable releases")
        gen_stable.make_all_stable(self.binaries)
        self.progress("Generate stable releases done")

        self.progress("Generating manifest")
        base_url = 'https://firmware.ardupilot.org'
        generator = generate_manifest.ManifestGenerator(self.binaries,
                                                        base_url)
        generator.run()

        generator.write_manifest_json(os.path.join(self.binaries, "manifest.json"))
        generator.write_features_json(os.path.join(self.binaries, "features.json"))
        self.progress("Manifest generation successful")

    def validate(self):
        '''run pre-run validation checks'''
        if "dirty" in self.tags:
            if len(self.tags) > 1:
                raise ValueError("dirty must be only tag if present (%s)" %
                                 (str(self.tags)))
            self.dirty = True
        if self.metadata_vehicles_to_generate is not None:
            unknown_vehicles = set(self.metadata_vehicles_to_generate) - set(self.metadata_vehicles)
            if unknown_vehicles:
                raise ValueError("Unknown metadata vehicles: %s" %
                                 ', '.join(sorted(unknown_vehicles)))

    def checkout_metadata_source(self, vehicle, tag):
        '''checkout metadata source without stashing the working tree'''
        if tag == 'latest':
            try:
                branch = self.run_git(
                    ['symbolic-ref', '--quiet', '--short', 'refs/remotes/origin/HEAD']
                ).strip()
            except subprocess.CalledProcessError:
                branch = ''
            if not branch:
                branch = 'master'
            branches = [branch]
        else:
            tag_vehicle = 'APMrover2' if vehicle == 'Rover' else vehicle
            branches = [tag, f'{tag_vehicle}-{tag}']

        for branch in branches:
            try:
                self.progress(f"Trying metadata source {branch}")
                self.run_git(['checkout', branch])
                return True
            except subprocess.CalledProcessError:
                self.progress(f"Checkout metadata source {branch} failed")

        return False

    def run_metadata_only(self):
        '''generate metadata for existing binary directories without rebuilding firmware'''
        if self.run_git(['diff', '--name-only']).strip() or \
           self.run_git(['diff', '--cached', '--name-only']).strip():
            raise ValueError("--metadata-only requires a clean tracked working tree")
        vehicle_names = self.metadata_vehicles_to_generate
        if vehicle_names is None:
            vehicle_names = self.metadata_vehicles.keys()
        original_branch = self.run_git(['branch', '--show-current']).strip()
        original_head = self.run_git(['rev-parse', 'HEAD']).strip()

        try:
            for tag in self.tags:
                for vehicle_name in vehicle_names:
                    vehicle_type, vehicle_binaries_subdir = self.metadata_vehicles[vehicle_name]
                    tag_dir = os.path.join(self.binaries, vehicle_binaries_subdir, tag)
                    if not os.path.isdir(tag_dir):
                        self.progress(f"No existing build directory at {tag_dir}")
                        self.error_strings.append(f"Missing build directory: {tag_dir}")
                        continue
                    if not self.checkout_metadata_source(vehicle_type, tag):
                        msg = f"Failed checkout of {vehicle_type} {tag} for metadata generation"
                        self.progress(msg)
                        self.error_strings.append(msg)
                        continue
                    if not self.generate_parameter_metadata_for_vehicle(tag, vehicle_type, tag_dir):
                        self.error_strings.append(
                            f"Failed metadata generation for {vehicle_type} {tag}")
        finally:
            if original_branch:
                self.run_git(['checkout', original_branch])
            else:
                self.run_git(['checkout', '--detach', original_head])

    def remove_tmpdir(self):
        if os.path.exists(self.tmpdir):
            self.progress("Removing (%s)" % (self.tmpdir,))
            shutil.rmtree(self.tmpdir)

    def buildlogs_dirpath(self):
        return os.getenv("BUILDLOGS",
                         os.path.join(os.getcwd(), "..", "buildlogs"))

    def exact_release_tag(self, vehicle_type: str):
        '''Return the release tag at HEAD, or None when HEAD is untagged.'''
        tag_vehicles = [vehicle for vehicle, (source, _) in self.metadata_vehicles.items()
                        if source == vehicle_type]
        if vehicle_type not in tag_vehicles:
            tag_vehicles.append(vehicle_type)

        for tag_vehicle in tag_vehicles:
            tag_pattern = f'{tag_vehicle}-[0-9]*.[0-9]*.[0-9]*'
            try:
                return self.run_program(
                    'GIT-DESCRIBE',
                    ['git', 'describe', '--tags', '--exact-match', '--match', tag_pattern],
                    show_output=False,
                ).strip()
            except subprocess.CalledProcessError:
                continue
        return None

    def create_pdef_xml_file(self, vehicle_type: str, dst_dir: str, git_tag: str | None) -> bool:
        '''generate parameter metadata XML file for a specific vehicle and version'''
        self.progress(f"Generating apm.pdef.xml for {vehicle_type} {git_tag}")

        try:
            param_parse_path = os.path.join(topdir(), 'Tools', 'autotest',
                                            'param_metadata', 'param_parse.py')

            if not os.path.exists(param_parse_path):
                self.progress(f"param_parse.py not found at {param_parse_path}")
                return False

            # Get current git SHA to embed in the metadata
            git_sha = self.run_git(["rev-parse", "HEAD"]).rstrip()

            # Older release branches may not support newer optional arguments.
            # The probe must succeed; otherwise metadata generation is invalid.
            help_output = self.run_program(
                'PARAM-PARSE-HELP',
                [sys.executable, param_parse_path, '--help'],
                show_output=False,
            )
            supports_git_sha = '--git-sha' in help_output
            supports_git_tag = '--git-tag' in help_output
            supports_format = '--format' in help_output
            supports_compress = '--compress' in help_output

            parameter_output_files = ['apm.pdef.xml']
            # Remove any stale output file before generating
            for output_file in parameter_output_files:
                for filename in (output_file, output_file + '.xz'):
                    filepath = os.path.join(dst_dir, filename)
                    if os.path.exists(filepath):
                        os.remove(filepath)

            apm_pdef_path = os.path.join(dst_dir, 'apm.pdef.xml')
            apm_pdef_xz_path = apm_pdef_path + '.xz'
            apm_pdef_gz_path = apm_pdef_path + '.gz'

            cmd = [
                sys.executable, param_parse_path,
                '--vehicle', vehicle_type,
            ]
            if supports_git_sha:
                cmd.extend(['--git-sha', git_sha])
            if supports_git_tag:
                if git_tag is not None:
                    cmd.extend(['--git-tag', git_tag])
            if supports_format:
                cmd.extend(['--format', 'xml'])
            if supports_compress:
                cmd.append('--compress')

            self.run_program('PARAM-PARSE', cmd, show_output=True, cwd=dst_dir)

            # If the parser did not produce a compressed file, create it here.
            if not os.path.exists(apm_pdef_xz_path):
                if not os.path.exists(apm_pdef_path):
                    self.progress("apm.pdef.xml was not generated")
                    return False
                with open(apm_pdef_path, 'rb') as f_in:
                    with lzma.open(apm_pdef_xz_path, 'wb', preset=9 | lzma.PRESET_EXTREME) as f_out:
                        shutil.copyfileobj(f_in, f_out)

            with open(apm_pdef_path, 'rb') as f_in:
                with gzip.open(apm_pdef_gz_path, 'wb', compresslevel=9) as f_out:
                    shutil.copyfileobj(f_in, f_out)

            # Check if the compressed output file was created
            if not os.path.isfile(apm_pdef_xz_path):
                self.progress("apm.pdef.xml.xz was not generated")
                return False

            # Create destination directory including __METADATA__ subdirectory
            metadata_dir = os.path.join(dst_dir, '__METADATA__')
            self.mkpath(metadata_dir)

            parameter_output_files.append('apm.pdef.xml.gz')
            for output_file in parameter_output_files:
                for filename in (output_file, output_file + '.xz'):
                    filepath = os.path.join(dst_dir, filename)
                    if os.path.isfile(filepath):
                        dst_file = os.path.join(metadata_dir, filename)
                        shutil.copy(filepath, dst_file)
                        self.progress(f"Created {dst_file}")
            return True

        except (subprocess.CalledProcessError, IOError, FileNotFoundError, shutil.Error) as e:
            self.print_exception_caught(e)
            self.progress(f"Failed to generate pdef.xml for {git_tag}")
            return False

    def create_log_messages_files(self, vehicle_type: str, dst_dir: str, git_branch: str) -> bool:
        '''generate logger metadata files for a specific vehicle and version'''
        self.progress(f"Generating LogMessages documentation for {vehicle_type}")

        if vehicle_type == 'AP_Periph':
            self.progress("Skipping LogMessages documentation for AP_Periph")
            return True

        logger_vehicle_map = {
            'ArduCopter': 'Copter',
            'ArduPlane': 'Plane',
            'ArduSub': 'Sub',
            'AntennaTracker': 'Tracker',
        }
        logger_vehicle_type = logger_vehicle_map.get(vehicle_type, vehicle_type)

        parser_path = os.path.join(topdir(), 'Tools', 'autotest',
                                   'logger_metadata', 'parse.py')
        if not os.path.exists(parser_path):
            self.progress(f"parse.py not found at {parser_path}")
            return False

        required_output_files = ['LogMessages.html', 'LogMessages.rst',
                                 'LogMessages.xml', 'LogMessages.xml.gz',
                                 'LogMessages.xml.xz']
        optional_output_files = ['LogMessages.md']
        json5_path = os.path.join(os.path.dirname(parser_path), 'emit_json5.py')
        if os.path.exists(json5_path):
            required_output_files.extend(['LogMessages.json5', 'LogMessages.json5.xz'])
        output_files = required_output_files + optional_output_files
        try:
            git_sha = self.run_git(["rev-parse", "HEAD"]).rstrip()
            help_output = self.run_program(
                'LOGGER-METADATA-HELP',
                [sys.executable, parser_path, '--help'],
                show_output=False,
                force_success=True,
            )

            for output_file in output_files:
                filepath = os.path.join(dst_dir, output_file)
                if os.path.exists(filepath):
                    os.remove(filepath)

            cmd = [sys.executable, parser_path, '--vehicle', logger_vehicle_type]
            supports_git_sha = '--git-sha' in help_output
            supports_git_branch = '--git-branch' in help_output
            if supports_git_sha:
                cmd.extend(['--git-sha', git_sha])
            if supports_git_branch and git_branch is not None:
                cmd.extend(['--git-branch', git_branch])
            self.run_program('LOGGER-METADATA', cmd, show_output=True, cwd=dst_dir)

            log_messages_xml = os.path.join(dst_dir, 'LogMessages.xml')
            if not os.path.exists(log_messages_xml):
                self.progress("LogMessages.xml was not generated")
                return False
            with open(log_messages_xml, 'rb') as source:
                with gzip.open(os.path.join(dst_dir, 'LogMessages.xml.gz'), 'wb', compresslevel=9) as compressed:
                    shutil.copyfileobj(source, compressed)
            with open(log_messages_xml, 'rb') as source:
                with lzma.open(os.path.join(dst_dir, 'LogMessages.xml.xz'), 'wb', preset=9 | lzma.PRESET_EXTREME) as compressed:
                    shutil.copyfileobj(source, compressed)
            log_messages_json5 = os.path.join(dst_dir, 'LogMessages.json5')
            if os.path.exists(log_messages_json5):
                with open(log_messages_json5, 'rb') as source:
                    with lzma.open(os.path.join(dst_dir, 'LogMessages.json5.xz'), 'wb', preset=9 | lzma.PRESET_EXTREME) as compressed:
                        shutil.copyfileobj(source, compressed)

            metadata_dir = os.path.join(dst_dir, '__METADATA__')
            self.mkpath(metadata_dir)
            for output_file in required_output_files:
                filepath = os.path.join(dst_dir, output_file)
                if not os.path.exists(filepath):
                    self.progress(f"{output_file} was not generated")
                    return False
                shutil.copy(filepath, os.path.join(metadata_dir, output_file))
                self.progress(f"Created {os.path.join(metadata_dir, output_file)}")

            for output_file in optional_output_files:
                dst_file = os.path.join(metadata_dir, output_file)
                filepath = os.path.join(dst_dir, output_file)
                if os.path.exists(filepath):
                    shutil.copy(filepath, dst_file)
                    self.progress(f"Created {dst_file}")
                elif os.path.exists(dst_file):
                    os.remove(dst_file)
                    self.progress(f"Removed stale optional {dst_file}")
            return True
        except (subprocess.CalledProcessError, IOError, FileNotFoundError, shutil.Error) as e:
            self.print_exception_caught(e)
            self.progress(f"Failed to generate LogMessages documentation for {vehicle_type}")
            return False

    def generate_parameter_metadata_for_vehicle(self, tag: str, vehicle_type: str, tag_dir: str):
        '''generate parameter metadata XML file for a specific vehicle and version'''

        self.progress(f"Generating parameter metadata for {vehicle_type} {tag}")
        staging_dir = tempfile.mkdtemp(prefix='.metadata-', dir=tag_dir)
        try:
            git_branch = self.run_git(['branch', '--show-current']).strip() or None
        except subprocess.CalledProcessError:
            git_branch = None
        try:
            log_messages_ok = self.create_log_messages_files(vehicle_type, staging_dir, git_branch)
            if not log_messages_ok:
                self.progress(f"Failed to create LogMessages documentation for {vehicle_type} {tag}")
                return False

            if tag == 'dirty':
                git_tag = tag
            else:
                # An untagged HEAD is normal for latest and beta builds.
                git_tag = self.exact_release_tag(vehicle_type)
                if git_tag is None:
                    self.progress(f"No exact release tag for {vehicle_type}; continuing without one")

            version = git_tag.split('-', 1)[1] if git_tag is not None and '-' in git_tag else tag
            if not self.create_pdef_xml_file(vehicle_type, staging_dir, git_tag):
                self.progress(f"Failed to create pdef.xml for {vehicle_type} {version}")
                return False

            metadata_dir = os.path.join(staging_dir, '__METADATA__')
            if not os.path.isdir(metadata_dir):
                self.progress("Metadata directory was not generated")
                return False
            self.publish_metadata_dir(tag_dir, metadata_dir)
            self.progress(f"Parameter metadata generation complete for {vehicle_type} {version}")
            return True
        finally:
            if os.path.exists(staging_dir):
                shutil.rmtree(staging_dir)

    def publish_metadata_dir(self, tag_dir: str, generated_metadata_dir: str):
        '''atomically replace published metadata after successful generation'''
        metadata_dir = os.path.join(tag_dir, '__METADATA__')
        backup_dir = None
        try:
            if os.path.exists(metadata_dir):
                backup_dir = tempfile.mkdtemp(prefix='.metadata-backup-', dir=tag_dir)
                os.rmdir(backup_dir)
                os.replace(metadata_dir, backup_dir)
            os.replace(generated_metadata_dir, metadata_dir)
        except OSError:
            if backup_dir is not None and not os.path.exists(metadata_dir):
                os.replace(backup_dir, metadata_dir)
            raise
        else:
            if backup_dir is not None:
                shutil.rmtree(backup_dir)

    def run(self):
        self.validate()

        if self.metadata_only:
            self.binaries = os.path.join(self.buildlogs_dirpath(), "binaries")
            self.error_strings = []
            self.run_metadata_only()
            for error_string in self.error_strings:
                self.progress("%s" % error_string)
            sys.exit(len(self.error_strings))

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

        self.binaries = os.path.join(self.buildlogs_dirpath(), "binaries")
        self.basedir = os.getcwd()
        self.error_strings = []

        self.mkpath(os.path.join("binaries", self.hdate_ym,
                                 self.hdate_ymdhm))

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
    parser.add_option("", "--metadata-only", action="store_true", default=False,
                      help="generate metadata for existing binary directories without rebuilding firmware")
    parser.add_option("", "--metadata-vehicle", action="append", type="string",
                      dest="metadata_vehicles",
                      help="vehicle metadata to generate (repeatable: Copter, Plane, Rover, Tracker, Sub, Blimp, AP_Periph)")
    cmd_opts, cmd_args = parser.parse_args()

    tags = cmd_opts.tags
    if len(tags) == 0:
        # FIXME: wedge this defaulting into parser somehow
        tags = ["stable", "beta", "latest"]

    bb = build_binaries(tags, metadata_only=cmd_opts.metadata_only,
                        metadata_vehicles=cmd_opts.metadata_vehicles)
    bb.run()
