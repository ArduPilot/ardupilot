"""
Contains functions used to test the ArduPilot examples

AP_FLAKE8_CLEAN
"""

import glob
import os
import signal
import subprocess
import time
import traceback

import pexpect

from pysim import util


def asan_log_base(filepath):
    """Return the ASAN log path prefix used for an example binary."""
    return util.asan_log_filepath(binary=filepath, model=None)


def asan_logs(filepath):
    """Return any non-empty ASAN logs left behind by an example binary."""
    # ASAN appends .<pid> to the log path
    return [x for x in glob.glob(asan_log_base(filepath) + ".*") if os.path.getsize(x) > 0]


def run_example(name, filepath, valgrind=False, gdb=False, asan=False):
    cmd = []
    if valgrind:
        cmd.append("valgrind")
    if gdb:
        cmd.append("gdb")
    cmd.append(filepath)
    print("Running: (%s)" % str(cmd))
    env = None
    if asan:
        # a report left behind by an earlier run would be taken as a
        # failure of this one
        for old_log in glob.glob(asan_log_base(filepath) + ".*"):
            os.unlink(old_log)
        # send the sanitizer report to a file; the example's own output
        # goes to DEVNULL, so anything printed to stderr would be lost
        env = dict(os.environ)
        our_opts = "log_path=%s:symbolize=1:verbosity=0" % asan_log_base(filepath)
        existing = env.get("ASAN_OPTIONS")
        env["ASAN_OPTIONS"] = (existing + ":" + our_opts) if existing else our_opts
        # llvm-symbolizer will try to fetch debug info over the network
        # for every lookup if this is set, which it is by default on
        # Ubuntu.  That blocks for ~90 seconds a frame, so the report is
        # never finished before we terminate the example
        env["DEBUGINFOD_URLS"] = ""
    bob = subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           env=env)

    expect_exit = False
    timeout = 10
    if name in [
            'RCProtocolTest',
            'Scheduler_test',
            'TransferFunctionCheck',
            'XPlane',
    ]:
        expect_exit = True

    time.sleep(timeout)

    if expect_exit:
        retcode = bob.poll()
        if retcode is None:
            # should maybe be an error in the future; that was the original intent
            print("process did not exit by the expected time")

        retcode = bob.wait()
    else:
        retcode = bob.poll()
        if retcode is not None:
            raise ValueError("Process exited before I could kill it (%s)" % str(retcode))

        bob.send_signal(signal.SIGTERM)
        time.sleep(1)
        retcode = bob.poll()
        print("retcode: %s" % str(retcode))
        if retcode is None:
            # if we get this far then we're not going to get a gcda file
            # out of this process for coverage analysis; it has to exit
            # normally, and it hasn't responded to a TERM.
            bob.kill()
            retcode2 = bob.wait()
            print("retcode2: %s" % str(retcode2))
            return

    if retcode == -15:
        print("process exited with -15, indicating it didn't catch the TERM signal and exit properly")
    elif retcode != 0:
        # note that process could exit with code 0 and we couldn't tell...
        raise ValueError("Process exited with non-zero exitcode %s" % str(retcode))

    print("Ran: (%s)" % str(cmd))


def print_exception_stacktrace(e):
    print(f"{e}\n")
    print(''.join(traceback.format_exception(type(e),
                                             e,
                                             tb=e.__traceback__)))


def run_examples(debug=False, valgrind=False, gdb=False, asan=False):
    dirpath = util.reltopdir(os.path.join('build', 'sitl', 'examples'))

    print("Running Hello")
    # explicitly run helloworld and check for output
    hello_path = os.path.join(dirpath, "Hello")
    p = pexpect.spawn(hello_path, ["Hello"])
    ex = None
    try:
        p.expect("hello world", timeout=5)
    except pexpect.TIMEOUT as e:
        ex = e
    print("ran Hello")

    p.close()

    if ex is not None:
        raise ex

    # note that some of the comments on examples here are incorrect -
    # since we are running on SITL it's not a matter of not having the
    # hardware, rather the simulation hasn't been set up
    # appropriately.  We run with a model of "NoVehicle", which
    # doesn't update the Aircraft base class.
    skip = {
        "AHRS_Test": "segfault as AP_Logger not instantiated",
        "AP_FW_Controller_test": "exits with a status code of 1 (failure) for some reason",
        "BARO_generic": "Most linux computers don't have baros...",
        "DSP_test": "exits with an arithmetic exception",
        "FlashTest": "https://github.com/ArduPilot/ardupilot/issues/14168",
        "INS_generic": "SITL is not available, segfaults",
        "ModuleTest": "test aborts",
        "NMEA_Output": "segfault as AP_Logger not instantiated",
        "RCProtocolDecoder": "This assumes specific hardware is connected",
        "SlewLimiter": "exits with a status code of 1 (failure) for some reason",
        "UART_chargen": "This nuke the term",
        "AP_Logger_AllTypes": "sanity checks fail on log write as we are attempting to write LOG_FILE_MSG items out and that doesn't exist in the structure we are using in this test",  # noqa:E501
        "CompassCalibrator_index_test": "flow of control error, invalid rotation created in auto_rotation_index_test?",
        "ReplayGyroFFT": "gyro data file /tmp/gyro0.dat (should this be a tool?)",
        "jedec_test": "external flash not found in SITL",
    }

    failures = []
    for afile in sorted(os.listdir(dirpath)):
        if afile in skip:
            print("Skipping %s: %s" % (afile, skip[afile]))
            continue
        filepath = os.path.join(dirpath, afile)
        if not os.path.isfile(filepath):
            continue
        try:
            run_example(afile, filepath, valgrind=valgrind, gdb=gdb, asan=asan)
        except Exception as e:  # noqa: BLE001
            print("Example failed with exception")
            print_exception_stacktrace(e)
            failures.append(afile)
        if asan:
            # a report can be left behind even when the example was
            # terminated normally, so check regardless of the outcome above
            logs = asan_logs(filepath)
            if len(logs):
                print("ASAN failure in %s:" % afile)
                for log in logs:
                    with open(log) as fh:
                        print(fh.read())
                if afile not in failures:
                    failures.append(afile)

    if len(failures):
        print("Failed examples:")
        for failure in failures:
            print(f"    {failure}")
        return False

    return True
