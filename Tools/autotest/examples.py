"""
Contains functions used to test the ArduPilot examples

AP_FLAKE8_CLEAN
"""

import os
import pexpect
import signal
import subprocess
import time
import traceback

from pysim import util


def run_example(name, filepath, valgrind=False, gdb=False):
    cmd = []
    if valgrind:
        cmd.append("valgrind")
    if gdb:
        cmd.append("gdb")
    cmd.append(filepath)
    print("Running: (%s)" % str(cmd))
    devnull = open("/dev/null", "w")
    bob = subprocess.Popen(cmd, stdin=devnull, stdout=devnull, stderr=devnull, close_fds=True)

    expect_exit = False
    timeout = 10
    if name in [
            'RCProtocolTest',
            'Scheduler_test',
            'TransferFunctionCheck',
            'XPlane',
    ]:
        expect_exit = True

    tstart = time.time()
    while True:
        if time.time() - tstart > timeout:
            break
        if not expect_exit:
            retcode = bob.poll()
            if retcode is not None:
                raise ValueError("Process exited before I could kill it (%s)" % str(retcode))

    if expect_exit:
        retcode = bob.wait()
        if retcode is None:
            raise ValueError("Expected example to exit, it did not")
    else:
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


def run_examples(debug=False, valgrind=False, gdb=False):
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
            run_example(afile, filepath, valgrind=valgrind, gdb=gdb)
        except Exception as e:
            print("Example failed with exception")
            print_exception_stacktrace(e)
            failures.append(afile)

    if len(failures):
        print("Failed examples:")
        for failure in failures:
            print(f"    {failure}")
        return False

    return True
