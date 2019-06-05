#!/usr/bin/env python

"""
script to run configre for all hwdef.dat, to check for syntax errors
"""

import os
import shutil
import subprocess
import sys
import fnmatch

import argparse
import threading

try:
    import queue as Queue
except ImportError:
    import Queue

parser = argparse.ArgumentParser(description='configure all ChibiOS boards')
parser.add_argument('--build', action='store_true', default=False, help='build as well as configure')
parser.add_argument('--stop', action='store_true', default=False, help='stop on build fail')
parser.add_argument('--pattern', default='*')
parser.add_argument('--workers', type=int, default=4)
args = parser.parse_args()

os.environ['PYTHONUNBUFFERED'] = '1'

worker_failures = Queue.Queue()

workqueue = Queue.Queue()

def worker_thread():
    global workqueue
    while True:
        try:
            job = workqueue.get(block=False)
        except Queue.Empty:
            break
        (cmd_list, build) = job
        print("Processing %s" % build)
        run_program(cmd_list, build)

def get_board_list():
    '''add boards based on existance of hwdef-bl.dat in subdirectories for ChibiOS'''
    board_list = []
    dirname, dirlist, filenames = next(os.walk('libraries/AP_HAL_ChibiOS/hwdef'))
    for d in dirlist:
        hwdef = os.path.join(dirname, d, 'hwdef.dat')
        if os.path.exists(hwdef):
            board_list.append(d)
    return board_list

def run_program(cmd_list, build):
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
        print("Build failed: %s %s" % (build, ' '.join(cmd_list)))
        global worker_failures
        worker_failures.put(build)
        if args.stop:
            sys.exit(1)

for board in get_board_list():
    if not fnmatch.fnmatch(board, args.pattern):
        continue
    print("Configuring for %s" % board)
    workqueue.put( (["./waf", "configure", "--board", board], "configure: " + board) )
    if args.build:
        if board == "iomcu":
            target = "iofirmware"
        else:
            target = "copter"
        workqueue.put( (["./waf", target], "build: " + board) )
    # check for bootloader def
    hwdef_bl = os.path.join('libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef-bl.dat' % board)
    if os.path.exists(hwdef_bl):
        print("Configuring bootloader for %s" % board)
        workqueue.put( (["./waf", "configure", "--board", board, "--bootloader"], "configure: " + board + "-bl") )
        if args.build:
            workqueue.put( (["./waf", "bootloader"], "build: " + board + "-bl") )

# start workers:
workers = []
for i in range(0, args.workers):
    t = threading.Thread(target=worker_thread, name='worker_thread %u' % i)
    t.start()
    workers.append(t)

# wait for workers to finish:
for worker in workers:
    print("Waiting for %s to finish" % str(worker))
    worker.join()

failures = []
while True:
    try:
        failure = worker_failures.get(block=False)
        failures.append(failure)
    except Queue.Empty:
        break

if len(failures) > 0:
    print("Failed builds:")
    for f in failures:
        print('  ' + f)
    sys.exit(1)

sys.exit(0)
