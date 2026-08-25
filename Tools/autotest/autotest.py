#!/usr/bin/env python3
"""
ArduPilot automatic test suite.

Andrew Tridgell, October 2011

 AP_FLAKE8_CLEAN
"""
import atexit
import json
import copy
import fnmatch
import glob
import optparse
import os
import pathlib
import re
import shutil
import signal
import subprocess
import sys
import time
import traceback

from pymavlink.generator import mavtemplate

import antennatracker
import arducopter
import arduplane
import ardusub
import balancebot
import blimp
import examples
import helicopter
import quadplane
import rover
import sailboat

from pysim import util
from vehicle_test_suite import Test

tester = None

autotest_start_time = time.time()


def buildlogs_dirpath():
    """Return BUILDLOGS directory path."""
    return os.getenv("BUILDLOGS", util.reltopdir("../buildlogs"))


def buildlogs_path(path):
    """Return a string representing path in the buildlogs directory."""
    bits = [buildlogs_dirpath()]
    if isinstance(path, list):
        bits.extend(path)
    else:
        bits.append(path)
    return os.path.join(*bits)


def build_all_filepath():
    """Get build_all.sh path."""
    return util.reltopdir('Tools/scripts/build_all.sh')


def build_all():
    """Run the build_all.sh script."""
    print("Running build_all.sh")
    if util.run_cmd(build_all_filepath(), directory=util.reltopdir('.')) != 0:
        print("Failed build_all.sh")
        return False
    return True


def build_binaries():
    """Run the build_binaries.py script."""
    print("Running build_binaries.py")

    # copy the script (and various libraries used by the script) as it
    # changes git branch, which can change the script while running
    for thing in [
            "board_list.py",
            "build_binaries_history.py",
            "build_binaries.py",
            "build_sizes/build_sizes.py",
            "generate_manifest.py",
            "gen_stable.py",
    ]:
        orig = util.reltopdir('Tools/scripts/%s' % thing)
        copy = util.reltopdir('./%s' % os.path.basename(thing))
        shutil.copy2(orig, copy)

    if util.run_cmd("./build_binaries.py", directory=util.reltopdir('.')) != 0:
        print("Failed build_binaries.py")
        return False
    return True


def build_examples(**kwargs):
    """Build examples."""
    for target in 'Pixhawk1', 'navio', 'linux', 'sitl':
        print("Running build.examples for %s" % target)
        try:
            util.build_examples(target, **kwargs)
        except Exception as e:  # noqa: BLE001
            print("Failed build_examples on board=%s" % target)
            print(str(e))
            return False

    return True


def build_unit_tests(**kwargs):
    """Build tests."""
    for target in ['linux', 'sitl']:
        print("Running build.unit_tests for %s" % target)
        try:
            util.build_tests(target, **kwargs)
        except Exception as e:  # noqa: BLE001
            print("Failed build.unit_tests on board=%s" % target)
            print(str(e))
            return False

    return True


def run_unit_test(test):
    """Run unit test file."""
    print("Running (%s)" % test)
    subprocess.check_call([test])


def run_unit_tests():
    """Run all unit tests files."""
    success = True
    fail_list = []
    for target in ['linux', 'sitl']:
        binary_dir = util.reltopdir(os.path.join('build',
                                                 target,
                                                 'tests',
                                                 ))
        tests = glob.glob("%s/*" % binary_dir)
        for test in tests:
            try:
                run_unit_test(test)
            except subprocess.CalledProcessError:
                print("Exception running (%s)" % test)
                fail_list.append(target + '/' + os.path.basename(test))
                success = False

    print("Failing tests:")
    for failure in fail_list:
        print("  %s" % failure)
    return success


def run_clang_scan_build():
    """Run Clang Scan-build utility."""
    if util.run_cmd("scan-build python3 waf configure",
                    directory=util.reltopdir('.')) != 0:
        print("Failed scan-build-configure")
        return False

    if util.run_cmd("scan-build python3 waf clean",
                    directory=util.reltopdir('.')) != 0:
        print("Failed scan-build-clean")
        return False

    # directories we never want in the reports: git submodules, vendored
    # third-party code and machine-generated sources.  --exclude keeps them
    # out of the browsable HTML; process_scan_build_output.py filters the
    # same list (EXCLUDE_DIRS) out of the plists, which is what the ratchet
    # counts.
    from scan_build_suppressions import EXCLUDE_DIRS
    exclude_args = ' '.join(
        '--exclude %s' % util.reltopdir(d.rstrip('/')) for d in EXCLUDE_DIRS
    )
    # -plist-html emits both the browsable HTML reports and .plist files;
    # the .plist files carry issue_hash_content_of_line_in_context, a
    # line-number-independent hash used to match the suppressions list.
    if util.run_cmd("scan-build -plist-html %s python3 waf build" % exclude_args,
                    directory=util.reltopdir('.')) != 0:
        print("Failed scan-build-build")
        return False

    return True


def param_parse_filepath():
    """Get param_parse.py script path."""
    return util.reltopdir('Tools/autotest/param_metadata/param_parse.py')


def all_vehicles():
    """Get all vehicles name."""
    return ('ArduPlane',
            'ArduCopter',
            'Rover',
            'AntennaTracker',
            'ArduSub',
            'Blimp',
            'AP_Periph',
            )


def build_parameters():
    """Run the param_parse.py script."""
    print("Running param_parse.py")
    for vehicle in all_vehicles():
        if util.run_cmd([param_parse_filepath(), '--vehicle', vehicle],
                        directory=util.reltopdir('.')) != 0:
            print("Failed param_parse.py (%s)" % vehicle)
            return False
    return True


def mavtogpx_filepath():
    """Get mavtogpx script path."""
    return util.reltopdir("modules/mavlink/pymavlink/tools/mavtogpx.py")


def convert_gpx():
    """Convert this run's tlog files to GPX and KML."""
    # the buildlogs directory is shared and accumulates tlogs from
    # previous runs; only convert files this run produced
    mavlog = [m for m in glob.glob(buildlogs_path("*.tlog"))
              if os.path.getmtime(m) >= autotest_start_time]
    passed = True
    for m in mavlog:
        util.run_cmd(mavtogpx_filepath() + " --nofixcheck " + m)
        gpx = m + '.gpx'
        kml = m + '.kml'
        try:
            util.run_cmd('gpsbabel -i gpx -f %s '
                         '-o kml,units=m,floating=1,extrude=1 -F %s' %
                         (gpx, kml))
        except subprocess.CalledProcessError:
            passed = False
        try:
            util.run_cmd('zip %s.kmz %s.kml' % (m, m))
        except subprocess.CalledProcessError:
            passed = False
        util.run_cmd("mavflightview.py --imagefile=%s.png %s" % (m, m))
    return passed


def test_prerequisites():
    """Check we have the right directories and tools to run tests."""
    print("Testing prerequisites")
    util.mkdir_p(buildlogs_dirpath())
    return True


def alarm_handler(signum, frame):
    """Handle test timeout."""
    try:
        print("Alarm handler called")
        if tester is not None:
            if tester.rc_thread is not None:
                tester.rc_thread_should_quit = True
                tester.rc_thread.join()
                tester.rc_thread = None
        results.add('TIMEOUT',
                    '<span class="failed-text">FAILED</span>',
                    opts.timeout)
        util.pexpect_close_all()
        convert_gpx()
        write_fullresults()
        os.killpg(0, signal.SIGKILL)
    except Exception:  # noqa: BLE001
        pass
    sys.exit(1)


def should_run_step(step):
    """See if a step should be skipped."""
    for skip in skipsteps:
        if fnmatch.fnmatch(step.lower(), skip.lower()):
            return False
    return True


__bin_names = {
    "Copter": "arducopter",
    "CopterTests1a": "arducopter",
    "CopterTests1b": "arducopter",
    "CopterTests1c": "arducopter",
    "CopterTests1d": "arducopter",
    "CopterTests1e": "arducopter",

    "CopterTests2a": "arducopter",
    "CopterTests2b": "arducopter",

    "Plane": "arduplane",
    "PlaneTests1a": "arduplane",
    "PlaneTests1b": "arduplane",
    "PlaneTests1c": "arduplane",

    "Rover": "ardurover",
    "Tracker": "antennatracker",
    "Helicopter": "arducopter-heli",
    "QuadPlane": "arduplane",
    "Sub": "ardusub",
    "Blimp": "blimp",
    "BalanceBot": "ardurover",
    "Sailboat": "ardurover",
    "SITLPeriphUniversal": ("sitl_periph_universal", "AP_Periph"),
    "SITLPeriphBattMon": ("sitl_periph_battmon", "AP_Periph"),
    "CAN": "arducopter",
    "BattCAN": "arducopter",
}


# the canonical build step which produces each entry in __bin_names;
# several test suites share one binary (QuadPlane flies arduplane,
# BalanceBot drives ardurover) so this is keyed by the binary, not the
# vehicle
__canonical_build_step_for_binary = {
    "arduplane": "build.Plane",
    "ardurover": "build.Rover",
    "arducopter": "build.Copter",
    "arducopter-heli": "build.Helicopter",
    "ardusub": "build.Sub",
    "antennatracker": "build.Tracker",
    "blimp": "build.Blimp",
    ("sitl_periph_universal", "AP_Periph"): "build.SITLPeriphUniversal",
    ("sitl_periph_battmon", "AP_Periph"): "build.SITLPeriphBattMon",
}


def canonical_build_step(vehicle):
    '''canonical build step producing the binary `vehicle`'s tests run,
    or None for names with no vehicle binary'''
    if vehicle not in __bin_names:
        return None
    return __canonical_build_step_for_binary.get(__bin_names[vehicle])


def implied_build_steps_for_test_step(step):
    '''the canonical build steps a test step needs: its vehicle binary
    plus any supplementary peripheral binaries its suite launches'''
    ret = []
    try:
        vehicle = step.split(".")[1]
    except IndexError:
        return ret
    b = canonical_build_step(vehicle)
    if b is not None:
        ret.append(b)
    for key, values in supplementary_test_binary_map.items():
        if step == key or step.startswith(key + "."):
            for spec in values:
                a = spec.split(':')
                sup = __canonical_build_step_for_binary.get((a[0], a[1]))
                if sup is not None and sup not in ret:
                    ret.append(sup)
    return ret


# build-option overrides for derived build steps, filled in by
# expand_build_steps(): later builds sharing the sitl board must not
# re-clean (wiping the binaries built moments earlier) or re-configure
derived_build_opt_overrides = {}


def expand_build_steps(steps):
    '''absorb explicit vehicle build steps into a derived build phase.

    Vehicle build steps no longer need to be given: each test step
    implies the builds it needs (via __bin_names and the supplementary
    binaries map).  Explicit vehicle build steps are still honoured -
    a build-only invocation must still build - but are deduplicated
    into the derived phase, which runs peripheral boards first (so the
    default waf lockfile is left pointing at the sitl board) and
    cleans/configures the sitl board only once.

    Returns the new step list.'''
    builds = []
    kept = []
    for step in steps:
        if step.startswith("build."):
            b = canonical_build_step(step.split(".")[1])
            if b is not None:
                if b != step:
                    print("Note: %s builds via %s" % (step, b))
                if b not in builds:
                    builds.append(b)
                continue
        kept.append(step)
        if step.startswith("test."):
            for b in implied_build_steps_for_test_step(step):
                if b not in builds:
                    builds.append(b)
    periph_boards = ("build.SITLPeriphUniversal", "build.SITLPeriphBattMon")
    periph = [b for b in builds if b in periph_boards]
    sitl = [b for b in builds if b not in periph_boards]
    derived_build_opt_overrides.clear()
    for b in sitl[1:]:
        # auto-configure already skips the later same-board configures;
        # cleaning again would wipe the binaries built moments earlier
        derived_build_opt_overrides[b] = {"clean": False}
    if builds:
        print("Derived build steps: %s" % " ".join(periph + sitl))
    return periph + sitl + kept


def binary_path(step, debug=False):
    """Get vehicle binary path."""
    try:
        vehicle = step.split(".")[1]
    except IndexError:
        return None

    if vehicle not in __bin_names:
        # cope with builds that don't have a specific binary
        return None

    try:
        (config_name, binary_name) = __bin_names[vehicle]
    except ValueError:
        config_name = "sitl"
        binary_name = __bin_names[vehicle]

    binary = util.reltopdir(os.path.join('build',
                                         config_name,
                                         'bin',
                                         binary_name))
    if not os.path.exists(binary):
        if os.path.exists(binary + ".exe"):
            binary += ".exe"
        else:
            raise ValueError("Binary (%s) does not exist" % (binary,))

    return binary


def split_specific_test_step(step):
    """Extract test from argument."""
    print('step=%s' % str(step))
    m = re.match("((fly|drive|dive|test)[.][^.]+)[.](.*)", step)
    if m is None:
        return None
    return ((m.group(1), m.group(3)))


def find_specific_test_to_run(step):
    """Find test to run in argument."""
    t = split_specific_test_step(step)
    if t is None:
        return None
    (testname, test) = t
    return "%s.%s" % (testname, test)


tester_class_map = {
    "test.Blimp": blimp.AutoTestBlimp,
    "test.Copter": arducopter.AutoTestCopter,
    "test.CopterTests1a": arducopter.AutoTestCopterTests1a, # 8m43s
    "test.CopterTests1b": arducopter.AutoTestCopterTests1b, # 8m5s
    "test.CopterTests1c": arducopter.AutoTestCopterTests1c, # 5m17s
    "test.CopterTests1d": arducopter.AutoTestCopterTests1d, # 8m20s
    "test.CopterTests1e": arducopter.AutoTestCopterTests1e, # 8m32s
    "test.CopterTests2a": arducopter.AutoTestCopterTests2a, # 8m23s
    "test.CopterTests2b": arducopter.AutoTestCopterTests2b, # 8m18s
    "test.Plane": arduplane.AutoTestPlane,
    "test.PlaneTests1a": arduplane.AutoTestPlaneTests1a,
    "test.PlaneTests1b": arduplane.AutoTestPlaneTests1b,
    "test.PlaneTests1c": arduplane.AutoTestPlaneTests1c,
    "test.QuadPlane": quadplane.AutoTestQuadPlane,
    "test.Rover": rover.AutoTestRover,
    "test.BalanceBot": balancebot.AutoTestBalanceBot,
    "test.Sailboat": sailboat.AutoTestSailboat,
    "test.Helicopter": helicopter.AutoTestHelicopter,
    "test.Sub": ardusub.AutoTestSub,
    "test.Tracker": antennatracker.AutoTestTracker,
    "test.CAN": arducopter.AutoTestCAN,
    "test.BattCAN": arducopter.AutoTestBattCAN,
}

supplementary_test_binary_map = {
    "test.CAN": ["sitl_periph_universal:AP_Periph:0:Tools/autotest/default_params/periph.parm,Tools/autotest/default_params/quad-periph.parm", # noqa: E501
                 "sitl_periph_universal:AP_Periph:1:Tools/autotest/default_params/periph.parm"],
    "test.BattCAN": [
        "sitl_periph_battmon:AP_Periph:0:Tools/autotest/default_params/periph-battmon.parm,Tools/autotest/default_params/quad-periph.parm", # noqa: E501
    ],
}


def run_specific_test(step, *args, **kwargs):
    """Run a specific test."""
    t = split_specific_test_step(step)
    if t is None:
        return []
    (testname, test) = t
    tests = set()
    tests.update(test.split(","))

    tester_class = tester_class_map[testname]
    global tester
    tester = tester_class(*args, **kwargs)

    # print("Got %s" % str(tester))
    run = []
    for a in tester.tests():
        if not isinstance(a, Test):
            a = Test(a)
        # print("Got %s" % (a.name))
        if a.name in tests:
            run.append(a)
            tests.remove(a.name)
    if len(tests):
        print(f"Failed to find tests {tests}")
        sys.exit(1)
    return tester.autotest(tests=run, allow_skips=False, step_name=step), tester


def make_build_opts():
    '''assemble the build options from the command-line options'''
    build_opts = {
        "j": opts.j,
        "debug": opts.debug,
        "clean": not opts.no_clean,
        # configuration is checked automatically: reconfigure only
        # when the wanted configuration differs from the tree's
        "configure": "auto",
        "math_check_indexes": opts.math_check_indexes,
        "ekf_single": opts.ekf_single,
        "postype_single": opts.postype_single,
        "extra_configure_args": list(opts.waf_configure_args),
        "coverage": opts.coverage,
        "force_32bit" : opts.force_32bit,
        "ubsan" : opts.ubsan,
        "ubsan_abort" : opts.ubsan_abort,
        "num_aux_imus" : opts.num_aux_imus,
        "dronecan_tests" : opts.dronecan_tests,
        "asan" : opts.asan,
    }

    if opts.Werror:
        build_opts['extra_configure_args'].append("--Werror")

    return build_opts


def run_step(step):
    """Run one step."""
    # remove old logs
    util.run_cmd('rm -f logs/*.BIN logs/LASTLOG.TXT')

    if step == "prerequisites":
        return test_prerequisites()

    build_opts = make_build_opts()
    # a derived build phase cleans and configures the shared sitl
    # board only once; see expand_build_steps()
    build_opts.update(derived_build_opt_overrides.get(step, {}))

    vehicle_binary = None
    board = "sitl"
    if step == 'build.Plane':
        vehicle_binary = 'bin/arduplane'

    if step == 'build.Rover':
        vehicle_binary = 'bin/ardurover'

    if step == 'build.Copter':
        vehicle_binary = 'bin/arducopter'

    if step == 'build.Blimp':
        vehicle_binary = 'bin/blimp'

    if step == 'build.Tracker':
        vehicle_binary = 'bin/antennatracker'

    if step == 'build.Helicopter':
        vehicle_binary = 'bin/arducopter-heli'

    if step == 'build.Sub':
        vehicle_binary = 'bin/ardusub'

    if step == 'build.SITLPeriphUniversal':
        vehicle_binary = 'bin/AP_Periph'
        board = 'sitl_periph_universal'

    if step == 'build.SITLPeriphBattMon':
        vehicle_binary = 'bin/AP_Periph'
        board = 'sitl_periph_battmon'

    if step == 'build.Replay':
        return util.build_replay(board='SITL')

    if vehicle_binary is not None:
        try:
            binary = binary_path(step, debug=opts.debug)
            os.unlink(binary)
        except (FileNotFoundError, ValueError):
            pass
        # boards other than sitl build fully isolated - their own
        # output directory and waf lockfile, the binary copied back to
        # build/<board>/ where consumers expect it.  Sharing the
        # default output directory is not an option even with a
        # separate lockfile: configure overwrites the shared
        # configuration cache (build/c4che), so the next sitl build
        # would compile against this board's environment.
        isolated = False
        if board != 'sitl':
            isolated = 'build-%s' % board
        return util.build_SITL(
            vehicle_binary,
            board=board,
            isolated=isolated,
            **build_opts
        )

    if step == 'build.All':
        return build_all()

    if step == 'build.Binaries':
        return build_binaries()

    if step == 'build.examples':
        return build_examples(**build_opts)

    if step == 'run.examples':
        return examples.run_examples(debug=opts.debug, valgrind=False, gdb=False)

    if step == 'build.Parameters':
        return build_parameters()

    if step == 'convertgpx':
        return convert_gpx()

    if step == 'build.unit_tests':
        return build_unit_tests(**build_opts)

    if step == 'run.unit_tests':
        return run_unit_tests()

    if step == 'clang-scan-build':
        return run_clang_scan_build()

    binary = binary_path(step, debug=opts.debug)

    fly_opts = make_fly_opts(step, build_opts)

    # handle "test.Copter" etc:
    if step in tester_class_map:
        # create an instance of the tester class:
        global tester
        tester = tester_class_map[step](binary, **fly_opts)
        # run the test and return its result and the tester itself
        return (tester.autotest(step_name=step, parallel=opts.parallel), tester)

    # handle "test.Copter.CPUFailsafe" etc:
    specific_test_to_run = find_specific_test_to_run(step)
    if specific_test_to_run is not None:
        return run_specific_test(specific_test_to_run, binary, **fly_opts)

    raise RuntimeError("Unknown step %s" % step)


def make_fly_opts(step, build_opts):
    '''assemble the tester-construction options for a test step'''
    # see if we need any supplementary binaries
    supplementary_binaries = []
    for key, value in supplementary_test_binary_map.items():
        if step.startswith(key):
            # this test needs to use supplementary binaries
            for supplementary_test_binary in value:
                a = supplementary_test_binary.split(':')
                if len(a) != 4:
                    raise ValueError("Bad supplementary_test_binary %s" % supplementary_test_binary)
                config_name = a[0]
                binary_name = a[1]
                param_file = a[3].split(",")
                bin_path = util.reltopdir(os.path.join('build', config_name, 'bin', binary_name))
                # note that the instance-number field is ignored:
                # an instance number allocates real machine resources,
                # so the test framework derives each supplementary
                # peripheral's instance from its own worker instance
                # instead (sup_instance_number)
                sup_binary = {"binary" : bin_path,
                              "param_file" : param_file}
                supplementary_binaries.append(sup_binary)
            # note that speedup is permitted here: the vehicle SITL is
            # started with --sim-periph-lockstep so it cannot outrun
            # the supplementary peripherals
            break

    fly_opts = {
        "viewerip": opts.viewerip,
        "use_map": opts.map,
        "valgrind": opts.valgrind,
        "callgrind": opts.callgrind,
        "asan": opts.asan,
        "gdb": opts.gdb,
        "gdb_no_tui": opts.gdb_no_tui,
        "lldb": opts.lldb,
        "strace": opts.strace,
        "gdbserver": opts.gdbserver,
        "breakpoints": opts.breakpoint,
        "disable_breakpoints": opts.disable_breakpoints,
        "_show_test_timings": opts.show_test_timings,
        "force_ahrs_type": opts.force_ahrs_type,
        "num_aux_imus" : opts.num_aux_imus,
        "replay": opts.replay,
        "logs_dir": buildlogs_dirpath(),
        "sup_binaries": supplementary_binaries,
        "reset_after_every_test": opts.reset_after_every_test,
        "build_opts": copy.copy(build_opts),
        "generate_junit": opts.junit,
        "enable_fgview": opts.enable_fgview,
        "instance": opts.instance,
    }
    if opts.speedup is not None:
        fly_opts["speedup"] = opts.speedup

    fly_opts["check_parameter_leaks"] = opts.check_parameter_leaks
    if opts.shuffle_seed is not None:
        fly_opts["shuffle_seed"] = opts.shuffle_seed

    fly_opts["move_logs_on_test_failure"] = opts.move_logs_on_test_failure

    return fly_opts


def test_durations_path():
    return buildlogs_path('autotest-test-durations.json')


def load_test_durations():
    '''previously-recorded per-test durations, keyed "step::testname"'''
    try:
        with open(test_durations_path()) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_test_durations(durations):
    try:
        with open(test_durations_path(), 'w') as f:
            json.dump(durations, f, indent=1, sort_keys=True)
    except OSError as e:
        print("Could not save test durations: %s" % e)


def run_unified_test_steps(test_steps):
    '''run several suites' tests in one parallel pool.

    Each suite's exclusive (serial-only) tests run first, suite by
    suite, at the base instance, exactly as the per-suite runner does.
    Every remaining test from every suite then goes into a single
    parallel pool: workers hold one live session at a time and swap
    testers when the next test belongs to a different suite, so the
    pool stays full instead of draining to stragglers at each suite
    boundary.

    Returns a list of (step, passed, tester) in the given order.'''
    util.run_cmd('rm -f logs/*.BIN logs/LASTLOG.TXT')
    build_opts = make_build_opts()
    suites = []
    factory = {}
    for step in test_steps:
        binary = binary_path(step, debug=opts.debug)
        fly_opts = make_fly_opts(step, build_opts)
        cls = tester_class_map[step]
        tester = cls(binary, **fly_opts)
        (tests, skip_list) = tester.prepare_tests()
        exclusive = tester.tests_needing_exclusive_run()
        serial = [t for t in tests if t.name in exclusive]
        parallel = [t for t in tests if t.name not in exclusive]
        # only the pooled tests are tagged with their suite: the serial
        # (exclusive) tests run on their own suite's tester, which has
        # no unified factory to construct testers from
        for t in parallel:
            t.suite_step = step
        suites.append({
            "step": step,
            "tester": tester,
            "serial": serial,
            "parallel": parallel,
            "skip_list": skip_list,
        })
        factory[step] = (cls, binary, fly_opts)

    base = opts.instance
    results_by_step = dict((suite["step"], []) for suite in suites)

    for suite in suites:
        if not len(suite["serial"]):
            continue
        print("Running %u %s test(s) serially (blacklisted from parallel run)" %
              (len(suite["serial"]), suite["step"]))
        results_by_step[suite["step"]] += suite["tester"].run_tests_in_processes(
            suite["serial"], 1, base_instance=base)

    # annotate each pooled test with its recorded duration and order
    # each suite longest-first: the dispatcher schedules from these -
    # a worker stays on its current suite (longest remaining test
    # next), idle workers adopt the suite with the most work left, and
    # the pool's tail is the shortest work.  A test with no recorded
    # duration is assumed long, so it runs early and gets measured.
    durations = load_test_durations()

    def test_duration(step, test):
        return durations.get("%s::%s" % (step, test.name), 300.0)

    for suite in suites:
        for t in suite["parallel"]:
            t.expected_duration = test_duration(suite["step"], t)
        suite["parallel"].sort(key=lambda t: -t.expected_duration)

    combined = []
    for suite in suites:
        combined += suite["parallel"]
    if len(combined):
        print("Running %u test(s) from %u suite(s) %u-way parallel" %
              (len(combined), len(suites), opts.parallel))
        conductor = suites[0]["tester"]
        conductor.unified_tester_factory = factory
        results = conductor.run_tests_in_processes(
            combined, opts.parallel, base_instance=base + 1)
        for result in results:
            step = getattr(result.test, "suite_step", suites[0]["step"])
            results_by_step[step].append(result)
            # only a pass measures the test; a failure's duration
            # measures where it died, and a fast failure recorded
            # here would schedule the test as short next run:
            if result.time_elapsed > 0 and getattr(result, "passed", False):
                durations["%s::%s" % (step, result.test.name)] = result.time_elapsed
        save_test_durations(durations)

    ret = []
    for suite in suites:
        step = suite["step"]
        ok = suite["tester"].report_results(
            results_by_step[step], suite["skip_list"], step_name=step)
        ret.append((step, ok, suite["tester"]))
    return ret


class TestResult(object):
    """Test result class."""

    def __init__(self, name, result, elapsed):
        """Init test result class."""
        self.name = name
        self.result = result
        self.elapsed = "%.1f" % elapsed


class TestFile(object):
    """Test result file."""

    def __init__(self, name, fname):
        """Init test result file."""
        self.name = name
        self.fname = fname


class TestResults(object):
    """Test results class."""

    def __init__(self):
        """Init test results class."""
        self.date = time.asctime()
        self.githash = util.get_git_hash()
        self.tests = []
        self.files = []
        self.images = []

    def add(self, name, result, elapsed):
        """Add a result."""
        self.tests.append(TestResult(name, result, elapsed))

    def addfile(self, name, fname):
        """Add a result file."""
        self.files.append(TestFile(name, fname))

    def addimage(self, name, fname):
        """Add a result image."""
        self.images.append(TestFile(name, fname))

    def addglob(self, name, pattern):
        """Add a set of files."""
        for f in glob.glob(buildlogs_path(pattern)):
            self.addfile(name, os.path.basename(f))

    def addglobimage(self, name, pattern):
        """Add a set of images."""
        for f in glob.glob(buildlogs_path(pattern)):
            self.addimage(name, os.path.basename(f))

    def generate_badge(self):
        """Get the badge template, populates and saves the result to buildlogs path."""
        passed_tests = len([t for t in self.tests if "PASSED" in t.result])
        total_tests = len(self.tests)
        badge_color = "#4c1" if passed_tests == total_tests else "#e05d44"

        badge_text = "{0}/{1}".format(passed_tests, total_tests)
        # Text length so it is not stretched by svg
        text_length = len(badge_text) * 70

        # Load template file
        template_path = 'Tools/autotest/web/autotest-badge-template.svg'
        template = pathlib.Path(util.reltopdir(template_path)).read_text()

        # Add our results to the template
        badge = template.format(color=badge_color,
                                text=badge_text,
                                text_length=text_length)
        with open(buildlogs_path("autotest-badge.svg"), "w") as f:
            f.write(badge)


def copy_tree(f, t, dirs_exist_ok=False):
    shutil.copytree(f, t, dirs_exist_ok=dirs_exist_ok)


def write_webresults(results_to_write):
    """Write webpage results."""
    t = mavtemplate.MAVTemplate()
    for h in glob.glob(util.reltopdir('Tools/autotest/web/*.html')):
        html = util.loadfile(h)
        f = open(buildlogs_path(os.path.basename(h)), mode='w')
        t.write(f, html, results_to_write)
        f.close()
    for f in glob.glob(util.reltopdir('Tools/autotest/web/*.png')):
        shutil.copy(f, buildlogs_path(os.path.basename(f)))
    copy_tree(util.reltopdir("Tools/autotest/web/css"), buildlogs_path("css"), dirs_exist_ok=True)
    results_to_write.generate_badge()


def write_fullresults():
    """Write out full results set."""
    results.addglob("Google Earth track", '*.kmz')
    results.addfile('Full Logs', 'autotest-output.txt')
    results.addglob('DataFlash Log', '*-log.bin')
    results.addglob("MAVLink log", '*.tlog')
    results.addglob("GPX track", '*.gpx')

    # results common to all vehicles:
    vehicle_files = [
        ('{vehicle} core', '{vehicle}.core'),
        ('{vehicle} ELF', '{vehicle}.elf'),
    ]
    vehicle_globs = [('{vehicle} log', '{vehicle}-*.BIN'), ]
    for vehicle in all_vehicles():
        subs = {'vehicle': vehicle}
        for vehicle_file in vehicle_files:
            description = vehicle_file[0].format(**subs)
            filename = vehicle_file[1].format(**subs)
            results.addfile(description, filename)
        for vehicle_glob in vehicle_globs:
            description = vehicle_glob[0].format(**subs)
            glob = vehicle_glob[1].format(**subs)
            results.addglob(description, glob)

    results.addglob("CopterAVC log", 'CopterAVC-*.BIN')
    results.addfile("CopterAVC core", 'CopterAVC.core')

    results.addglob('APM:Libraries documentation', 'docs/libraries/index.html')
    results.addglob('APM:Plane documentation', 'docs/ArduPlane/index.html')
    results.addglob('APM:Copter documentation', 'docs/ArduCopter/index.html')
    results.addglob('APM:Rover documentation', 'docs/Rover/index.html')
    results.addglob('APM:Sub documentation', 'docs/ArduSub/index.html')
    results.addglob('APM:Blimp documentation', 'docs/Blimp/index.html')
    results.addglobimage("Flight Track", '*.png')

    write_webresults(results)


# highest instance number the per-instance port allocation supports;
# instance 86's RC-in port (5501+3*86) is instance 0's SITL port (5760)
MAX_AUTOTEST_INSTANCE = 85


def run_tests(steps):
    """Run a list of steps."""

    # A serial, instance-0 run uses the repo-root working directory.  The
    # ArduPilot scripting engine loads every file in "scripts/", so stale
    # content there (e.g. left over from a previous run, or a dangling
    # symlink) silently pollutes the run.  Refuse to start rather than
    # produce confusing failures.  Parallel runs - and serial "-I N" runs -
    # each use their own fresh per-instance directory, so they are immune
    # and exempt from this check.
    if opts.parallel == 1 and opts.instance == 0:
        if os.path.isdir("scripts"):
            # "scripts/modules" is exempt: the engine reads it only when a
            # script requires a module rather than loading it on sight, and
            # removing an installed module leaves its parent directory
            # behind, so an empty one is what a clean run looks like.
            scripts_contents = [
                x for x in os.listdir("scripts")
                if not (x == "modules" and os.path.isdir(os.path.join("scripts", x)))
            ]
            if len(scripts_contents) > 0:
                print("ERROR: refusing to start: serial autotest runs in the "
                      "repo-root working directory but 'scripts/' is not empty: "
                      "%s" % sorted(scripts_contents))
                print("Remove its contents first (parallel runs use "
                      "per-instance directories and are unaffected).")
                sys.exit(1)

    # The per-instance port formulas in vehicle_test_suite.py only stay
    # clear of one another for so many instances: RC-in is
    # 5501+3*instance and SITL's TCP ports are 5760+10*instance, so by
    # instance 86 the RC-in block has walked into instance 0's SITL
    # port.  Past that, two workers bind the same port, one of them
    # never gets a SITL up, and the run sits waiting for a result which
    # is never coming - the runner only gives up when *every* worker has
    # died, so one stuck worker hangs the lot.  Say so now instead.
    highest_instance = opts.instance + opts.parallel
    if highest_instance > MAX_AUTOTEST_INSTANCE:
        print("ERROR: --parallel=%u with -I %u would use instances up to "
              "%u, but the per-instance port allocation only supports up "
              "to %u (instance %u's RC-in port is instance 0's SITL "
              "port)." % (opts.parallel, opts.instance, highest_instance,
                          MAX_AUTOTEST_INSTANCE, MAX_AUTOTEST_INSTANCE + 1))
        sys.exit(1)

    corefiles = glob.glob("core*")
    corefiles.extend(glob.glob("ap-*.core"))
    if corefiles:
        print('Removing corefiles: %s' % str(corefiles))
        for f in corefiles:
            os.unlink(f)

    diagnostic_files = []
    for p in "dumpstack.sh_*", "dumpcore.sh_*", "autotest-*tlog":
        diagnostic_files.extend(glob.glob(p))
    if diagnostic_files:
        print('Removing diagnostic files: %s' % str(diagnostic_files))
        for f in diagnostic_files:
            os.unlink(f)

    # each parallel worker (and serial "-I N" run) runs in its own
    # "parallel-autotest/<instance>" directory.  Wipe the directories THIS
    # run will use so per-instance logs/eeprom/etc. don't accumulate across
    # runs - but only this run's instance range, so concurrent runs started
    # with different -I values don't delete each other's directories.
    lo = opts.instance
    hi = opts.instance + opts.parallel  # parallel pass uses base+1..base+parallel
    instance_dirs = " ".join("parallel-autotest/%u" % n for n in range(lo, hi + 1))
    print("Removing parallel autotest instance directories %u..%u" % (lo, hi))
    util.run_cmd("rm -rf " + instance_dirs, checkfail=False)

    steps = expand_build_steps(steps)

    # with a parallel pool and more than one whole-suite test step, all
    # the suites' tests share one pool: pull those steps out of the
    # sequential loop (their builds have already been derived)
    unified_steps = []
    if opts.parallel > 1:
        unified_steps = [s for s in steps if s in tester_class_map]
    if len(unified_steps) > 1:
        steps = [s for s in steps if s not in unified_steps]
    else:
        unified_steps = []

    passed = True
    failed = []
    failed_testinstances = dict()
    for step in steps:
        util.pexpect_close_all()

        t1 = time.time()
        if step.startswith("test."):
            broken = [b for b in implied_build_steps_for_test_step(step)
                      if b in failed]
            if broken:
                print(">>>> SKIPPED STEP: %s at %s (%s failed)" %
                      (step, time.asctime(), " ".join(broken)))
                passed = False
                failed.append(step)
                results.add(step, '<span class="failed-text">SKIPPED (build failed)</span>',
                            0.0)
                continue
        print(">>>> RUNNING STEP: %s at %s" % (step, time.asctime()))
        try:
            success = run_step(step)
            testinstance = None
            if isinstance(success, tuple):
                (success, testinstance) = success
            if success:
                results.add(step, '<span class="passed-text">PASSED</span>',
                            time.time() - t1)
                print(">>>> PASSED STEP: %s at %s" % (step, time.asctime()))
            else:
                print(">>>> FAILED STEP: %s at %s" % (step, time.asctime()))
                passed = False
                failed.append(step)
                if testinstance is not None:
                    if failed_testinstances.get(step) is None:
                        failed_testinstances[step] = []
                    failed_testinstances[step].append(testinstance)
                results.add(step, '<span class="failed-text">FAILED</span>',
                            time.time() - t1)
        except Exception as msg:  # noqa: BLE001
            passed = False
            failed.append(step)
            print(">>>> FAILED STEP: %s at %s (%s)" %
                  (step, time.asctime(), msg))
            traceback.print_exc(file=sys.stdout)
            results.add(step,
                        '<span class="failed-text">FAILED</span>',
                        time.time() - t1)

        if tester is not None and tester.rc_thread is not None:
            if passed:
                print("BAD: RC Thread still alive after run_step")
            tester.rc_thread_should_quit = True
            tester.rc_thread.join()
            tester.rc_thread = None

    if len(unified_steps):
        t1 = time.time()
        print(">>>> RUNNING UNIFIED STEPS: %s at %s" %
              (" ".join(unified_steps), time.asctime()))
        try:
            outcomes = run_unified_test_steps(unified_steps)
        except Exception as msg:  # noqa: BLE001
            print(">>>> FAILED UNIFIED STEPS at %s (%s)" %
                  (time.asctime(), msg))
            traceback.print_exc(file=sys.stdout)
            outcomes = [(step, False, None) for step in unified_steps]
        elapsed = time.time() - t1
        for (step, success, testinstance) in outcomes:
            if success:
                results.add(step, '<span class="passed-text">PASSED</span>', elapsed)
                print(">>>> PASSED STEP: %s at %s" % (step, time.asctime()))
            else:
                print(">>>> FAILED STEP: %s at %s" % (step, time.asctime()))
                passed = False
                failed.append(step)
                if testinstance is not None:
                    failed_testinstances[step] = [testinstance]
                results.add(step, '<span class="failed-text">FAILED</span>', elapsed)

    if not passed:
        keys = failed_testinstances.keys()
        if len(keys):
            print("Failure Summary:")
        for key in keys:
            print("  %s:" % key)
            for testinstance in failed_testinstances[key]:
                for failure in testinstance.fail_list:
                    print("  " + str(failure))

        print("FAILED %u tests: %s" % (len(failed), failed))

    util.pexpect_close_all()

    write_fullresults()

    return passed


vehicle_list = ['Sub', 'Copter', 'Plane', 'Tracker', 'Rover', 'QuadPlane', 'BalanceBot', 'Helicopter', 'Sailboat', 'Blimp']


def list_subtests():
    """Print the list of tests and tests description for each vehicle."""
    for vehicle in sorted(vehicle_list):
        tester_class = tester_class_map["test.%s" % vehicle]
        tester = tester_class("/bin/true", None)
        subtests = tester.tests()
        sorted_list = []
        for subtest in subtests:
            if str(type(subtest)) == "<class 'method'>":
                subtest = Test(subtest)
            sorted_list.append([subtest.name, subtest.description])
        sorted_list.sort()

        print("%s:" % vehicle)
        for subtest in sorted_list:
            print("    %s: %s" % (subtest[0], subtest[1]))
        print("")


def list_subtests_for_vehicle(vehicle_type):
    """Print the list of tests for a vehicle."""
    # Check that we aren't in a sub test
    if "Test" in vehicle_type:
        vehicle_type = re.findall('[A-Z][a-z0-9]*', vehicle_type)[0]
    if vehicle_type in vehicle_list:
        tester_class = tester_class_map["test.%s" % vehicle_type]
        tester = tester_class("/bin/true", None)
        subtests = tester.tests()
        sorted_list = []
        for subtest in subtests:
            if not isinstance(subtest, Test):
                subtest = Test(subtest)
            sorted_list.append([subtest.name, subtest.description])
        sorted_list.sort()
        for subtest in sorted_list:
            print("%s " % subtest[0], end='')
        print("")  # needed to clear the trailing %


if __name__ == "__main__":
    ''' main program '''
    os.environ['PYTHONUNBUFFERED'] = '1'

    # pin SITL's multicast traffic (the simulation state a periph
    # consumes, and multicast CAN) to the loopback interface.  By
    # default it follows the routing table, which means it goes out
    # whichever interface has the default route and stops working when
    # that route is not up or is not multicast-capable; a test should
    # not pass or fail on the state of the machine's network.  Every
    # SITL we start inherits this.
    os.environ.setdefault('SITL_MULTICAST_IF_ADDR', '127.0.0.1')

    if sys.platform != "darwin":
        os.putenv('TMPDIR', util.reltopdir('tmp'))

    class MyOptionParser(optparse.OptionParser):
        """Custom option parse class."""

        def format_epilog(self, formatter):
            """Return customized option parser epilog."""
            return self.epilog

    parser = MyOptionParser(
        "autotest", epilog=""
        "e.g. autotest.py build.Rover test.Rover # test Rover\n"
        "e.g. autotest.py build.Rover test.Rover build.Plane test.Plane # test Rover and Plane\n"
        "e.g. autotest.py --debug --valgrind build.Rover test.Rover # test Rover under Valgrind\n"
        "e.g. autotest.py --debug --gdb build.Tracker test.Tracker # run Tracker under gdb\n"
        "e.g. autotest.py --debug --gdb build.Sub test.Sub.DiveManual # do specific Sub test\n"
    )
    parser.add_option("--autotest-server",
                      action='store_true',
                      default=False,
                      help='Run in autotest-server mode; dangerous!')
    parser.add_option("--move-logs-on-test-failure",
                      action='store_true',
                      default=None,
                      help='Move logs to ../buildlogs if a test fails (default)')
    parser.add_option("--no-move-logs-on-test-failure",
                      action='store_false',
                      dest='move_logs_on_test_failure',
                      help='Leave logs where they are when a test fails')
    parser.add_option("--skip",
                      type='string',
                      default='',
                      help='list of steps to skip (comma separated)')
    parser.add_option("--list",
                      action='store_true',
                      default=False,
                      help='list the available steps')
    parser.add_option("--list-subtests",
                      action='store_true',
                      default=False,
                      help='list available subtests e.g. test.Copter')
    parser.add_option("--viewerip",
                      default=None,
                      help='IP address to send MAVLink and fg packets to')
    parser.add_option("--enable-fgview",
                      action='store_true',
                      help="Enable FlightGear output")
    parser.add_option("--map",
                      action='store_true',
                      default=False,
                      help='show map')
    parser.add_option("--experimental",
                      default=False,
                      action='store_true',
                      help='enable experimental tests')
    parser.add_option("--timeout",
                      default=None,
                      type='int',
                      help='maximum runtime in seconds')
    parser.add_option("--parallel",
                      default=1,
                      type='int',
                      help='number of tests to run in parallel')
    parser.add_option("-I", "--instance",
                      default=0,
                      type='int',
                      help='base instance number (like sim_vehicle.py -I): offsets '
                           'the ports and per-instance working directories.  For a '
                           'serial run this is the instance used; with --parallel it '
                           'is the lowest instance, and workers count up from it.  '
                           'Use distinct -I values to run several parallel suites at '
                           'once without colliding (give each its own BUILDLOGS too).')
    parser.add_option("--show-test-timings",
                      action="store_true",
                      default=False,
                      help="show how long each test took to run")
    parser.add_option("--validate-parameters",
                      action="store_true",
                      default=False,
                      help="validate vehicle parameter files")
    parser.add_option("--Werror",
                      action='store_true',
                      default=False,
                      help='configure with --Werror')
    parser.add_option("--junit",
                      default=False,
                      action='store_true',
                      help='Generate Junit XML tests report')

    group_build = optparse.OptionGroup(parser, "Build options")
    # deprecated: configuration is now checked automatically and only
    # rerun when the wanted configuration differs from the tree's
    group_build.add_option("--no-configure",
                           default=False,
                           action='store_true',
                           help='do not configure before building',
                           dest="no_configure")
    group_build.add_option("", "--waf-configure-args",
                           action="append",
                           dest="waf_configure_args",
                           type="string",
                           default=[],
                           help="extra arguments passed to waf in configure")
    group_build.add_option("-j", default=None, type='int', help='build CPUs')
    group_build.add_option("--no-clean",
                           default=False,
                           action='store_true',
                           help='do not clean before building',
                           dest="no_clean")
    group_build.add_option("--debug",
                           default=None,
                           action='store_true',
                           help='make built SITL binaries debug binaries')
    group_build.add_option("--no-debug",
                           default=None,
                           action='store_true',
                           help='do not make built SITL binaries debug binaries')
    group_build.add_option("--coverage",
                           default=False,
                           action='store_true',
                           help='make built binaries coverage binaries')
    group_build.add_option("--enable-math-check-indexes",
                           default=False,
                           action="store_true",
                           dest="math_check_indexes",
                           help="enable checking of math indexes")
    group_build.add_option("--postype-single",
                           default=False,
                           action="store_true",
                           dest="postype_single",
                           help="force single precision copter position controller")
    group_build.add_option("--ekf-single",
                           default=False,
                           action="store_true",
                           dest="ekf_single",
                           help="force single precision EKF")
    group_build.add_option("--force-32bit",
                           default=False,
                           action='store_true',
                           dest="force_32bit",
                           help="compile sitl using 32-bit")
    group_build.add_option("", "--ubsan",
                           default=False,
                           action='store_true',
                           dest="ubsan",
                           help="compile sitl with undefined behaviour sanitiser")
    group_build.add_option("", "--ubsan-abort",
                           default=False,
                           action='store_true',
                           dest="ubsan_abort",
                           help="compile sitl with undefined behaviour sanitiser and abort on error")
    group_build.add_option("--num-aux-imus",
                           dest="num_aux_imus",
                           default=0,
                           type='int',
                           help='number of auxiliary IMUs to simulate')
    group_build.add_option("--enable-dronecan-tests",
                           default=False,
                           action='store_true',
                           dest="dronecan_tests",
                           help="enable dronecan tests")
    parser.add_option_group(group_build)

    group_sim = optparse.OptionGroup(parser, "Simulation options")
    group_sim.add_option("--speedup",
                         default=None,
                         type='int',
                         help='speedup to run the simulations at')
    group_sim.add_option("--check-parameter-leaks",
                         action='store_true',
                         dest='check_parameter_leaks',
                         default=True,
                         help='after each test, check no parameter the suite '
                         'could not revert has been left changed; catches '
                         'leaks into the tests which follow.  On by default')
    group_sim.add_option("--no-check-parameter-leaks",
                         action='store_false',
                         dest='check_parameter_leaks',
                         help='do not check for parameter leaks after each '
                         'test.  The check downloads the full parameter set '
                         'once per test')
    group_sim.add_option("--shuffle-seed",
                         default=None,
                         type='int',
                         help='shuffle the test order with this seed; '
                         'varies which tests run next to one another, '
                         'and can be repeated to reproduce a run')
    group_sim.add_option("--valgrind",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under valgrind')
    group_sim.add_option("--asan",
                         default=False,
                         action='store_true',
                         help='enable ASAN error checking (binary must be built with --asan --debug)')
    group_sim.add_option("", "--callgrind",
                         action='store_true',
                         default=False,
                         help="enable valgrind for performance analysis (slow!!)")
    group_sim.add_option("--gdb",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under gdb')
    group_sim.add_option("--gdb-no-tui",
                         default=False,
                         action='store_true',
                         help='when running under GDB do NOT start in TUI mode')
    group_sim.add_option("--gdbserver",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under gdbserver')
    group_sim.add_option("--lldb",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under lldb')
    group_sim.add_option("", "--strace",
                         action='store_true',
                         default=False,
                         help="strace the ArduPilot binary")
    group_sim.add_option("-B", "--breakpoint",
                         type='string',
                         action="append",
                         default=[],
                         help="add a breakpoint at given location in debugger")
    group_sim.add_option("--disable-breakpoints",
                         default=False,
                         action='store_true',
                         help="disable all breakpoints before starting")
    group_sim.add_option("", "--force-ahrs-type",
                         dest="force_ahrs_type",
                         default=None,
                         help="force a specific AHRS type (e.g. 10 for SITL-ekf")
    group_sim.add_option("", "--replay",
                         action='store_true',
                         help="enable replay logging for tests")
    parser.add_option_group(group_sim)

    group_completion = optparse.OptionGroup(parser, "Completion helpers")
    group_completion.add_option("--list-vehicles",
                                action='store_true',
                                default=False,
                                help='list available vehicles')
    group_completion.add_option("--list-vehicles-test",
                                action='store_true',
                                default=False,
                                help='list available vehicle tester')
    group_completion.add_option("--list-subtests-for-vehicle",
                                type='string',
                                default="",
                                help='list available subtests for a vehicle e.g Copter')
    group_completion.add_option("--reset-after-every-test",
                                action='store_true',
                                default=False,
                                help='reset everything after every test run')
    parser.add_option_group(group_completion)

    opts, args = parser.parse_args()

    # canonicalise on opts.debug:
    if opts.debug is None and opts.no_debug is None:
        # default is to create debug SITL binaries
        opts.debug = True
    elif opts.debug is not None and opts.no_debug is not None:
        if opts.debug == opts.no_debug:
            raise ValueError("no_debug != !debug")
    elif opts.no_debug is not None:
        opts.debug = not opts.no_debug

    if opts.timeout is None:
        opts.timeout = 5400
        # adjust if we're running in a regime which may slow us down e.g. Valgrind
        if opts.valgrind:
            opts.timeout *= 10
        elif opts.callgrind:
            opts.timeout *= 10
        elif opts.asan:
            opts.timeout *= 2
        elif opts.gdb:
            opts.timeout = None

    # Keep the telemetry and dataflash logs of a test which fails.  They
    # are what the failure has to be diagnosed from, the next run of that
    # test overwrites them, and a failure nobody can look into is a run
    # wasted.  Pass --no-move-logs-on-test-failure to leave them be.
    if opts.move_logs_on_test_failure is None:
        opts.move_logs_on_test_failure = True

    steps = [
        'prerequisites',
        'build.Binaries',
        'build.All',
        'build.Parameters',

        'build.Replay',

        'build.unit_tests',
        'run.unit_tests',
        'build.examples',
        'run.examples',

        'build.Plane',
        'test.Plane',
        'test.QuadPlane',

        'build.Rover',
        'test.Rover',
        'test.BalanceBot',
        'test.Sailboat',

        'build.Copter',
        'test.Copter',

        'build.Helicopter',
        'test.Helicopter',

        'build.Tracker',
        'test.Tracker',

        'build.Sub',
        'test.Sub',

        'build.Blimp',
        'test.Blimp',

        'build.SITLPeriphUniversal',
        'test.CAN',

        'build.SITLPeriphBattMon',
        'test.BattCAN',

        # convertgps disabled as it takes 5 hours
        # 'convertgpx',
    ]

    moresteps = [
        'test.CopterTests1a',
        'test.CopterTests1b',
        'test.CopterTests1c',
        'test.CopterTests1d',
        'test.CopterTests1e',

        'test.CopterTests2a',
        'test.CopterTests2b',

        'test.PlaneTests1a',
        'test.PlaneTests1b',
        'test.PlaneTests1c',

        'clang-scan-build',
    ]

    # canonicalise the step names.  This allows
    # backwards-compatability from the hodge-podge
    # fly.ArduCopter/drive.APMrover2 to the more common test.Copter
    # test.Rover
    step_mapping = {
        "build.ArduPlane": "build.Plane",
        "build.ArduCopter": "build.Copter",
        "build.APMrover2": "build.Rover",
        "build.ArduSub": "build.Sub",
        "build.AntennaTracker": "build.Tracker",
        "fly.ArduCopter": "test.Copter",
        "fly.ArduPlane": "test.Plane",
        "fly.QuadPlane": "test.QuadPlane",
        "dive.ArduSub": "test.Sub",
        "drive.APMrover2": "test.Rover",
        "drive.BalanceBot": "test.BalanceBot",
        "drive.balancebot": "test.BalanceBot",
        "fly.CopterAVC": "test.Helicopter",
        "test.AntennaTracker": "test.Tracker",
        "fly.ArduCopterTests1a": "test.CopterTests1a",
        "fly.ArduCopterTests1b": "test.CopterTests1b",
        "fly.ArduCopterTests1c": "test.CopterTests1c",
        "fly.ArduCopterTests1d": "test.CopterTests1d",
        "fly.ArduCopterTests1e": "test.CopterTests1e",

        "fly.ArduCopterTests2a": "test.CopterTests2a",
        "fly.ArduCopterTests2b": "test.CopterTests2b",

    }

    # form up a list of bits NOT to run, mapping from old step names
    # to new step names as appropriate.
    skipsteps = opts.skip.split(',')
    new_skipsteps = []
    for skipstep in skipsteps:
        if skipstep in step_mapping:
            new_skipsteps.append(step_mapping[skipstep])
        else:
            new_skipsteps.append(skipstep)
    skipsteps = new_skipsteps

    # ensure we catch timeouts
    signal.signal(signal.SIGALRM, alarm_handler)
    if opts.timeout is not None:
        signal.alarm(opts.timeout)

    if opts.list:
        for step in steps:
            print(step)
        sys.exit(0)

    if opts.list_subtests:
        list_subtests()
        sys.exit(0)

    if opts.list_subtests_for_vehicle:
        list_subtests_for_vehicle(opts.list_subtests_for_vehicle)
        sys.exit(0)

    if opts.list_vehicles_test:
        print(' '.join(__bin_names.keys()))
        sys.exit(0)

    if opts.list_vehicles:
        print(' '.join(vehicle_list))
        sys.exit(0)

    util.mkdir_p(buildlogs_dirpath())

    if opts.no_configure:
        print("Note: --no-configure is deprecated; configuration is "
              "checked automatically and rerun only when it differs")

    lckfile = buildlogs_path('autotest.lck')
    print("lckfile=%s" % repr(lckfile))
    lck = util.lock_file(lckfile)

    if lck is None:
        print("autotest is locked - exiting.  lckfile=(%s)" % (lckfile,))
        sys.exit(1)

    atexit.register(util.pexpect_close_all)

    # provide backwards-compatability from (e.g.) drive.APMrover2 -> test.Rover
    newargs = []
    for arg in args:
        for _from, to in step_mapping.items():
            arg = re.sub("^%s" % _from, to, arg)
        newargs.append(arg)
    args = newargs

    if len(args) == 0 and not opts.autotest_server:
        print("Steps must be supplied; try --list and/or --list-subtests or --help")
        sys.exit(1)

    if len(args) > 0:
        # allow a wildcard list of steps
        matched = []
        for a in args:
            matches = [step for step in steps
                       if fnmatch.fnmatch(step.lower(), a.lower())]
            x = find_specific_test_to_run(a)
            if x is not None:
                matches.append(x)

            if a in moresteps:
                matches.append(a)

            # any vehicle with a binary may be named in a build step;
            # expand_build_steps() folds it into the derived build
            # phase (e.g. build.QuadPlane builds via build.Plane)
            if (not matches and a.startswith("build.") and
                    canonical_build_step(a.split(".")[1]) is not None):
                matches.append(a)

            if not len(matches):
                print("No steps matched {}".format(a))
                sys.exit(1)
            matched.extend(matches)
        steps = matched

    # skip steps according to --skip option:
    steps_to_run = [s for s in steps if should_run_step(s)]

    results = TestResults()

    try:
        if not run_tests(steps_to_run):
            sys.exit(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught; closing pexpect connections")
        util.pexpect_close_all()
        raise
    except Exception:
        # make sure we kill off any children
        util.pexpect_close_all()
        raise
