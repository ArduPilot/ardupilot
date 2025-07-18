#!/usr/bin/env python3

# flake8: noqa

import pexpect, time, sys
from pymavlink import mavutil

def wait_heartbeat(mav, timeout=10):
    '''wait for a heartbeat'''
    start_time = time.time()
    while time.time() < start_time+timeout:
        if mav.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5) is not None:
            return
    raise Exception("Failed to get heartbeat")    

def wait_mode(mav, modes, timeout=10):
    '''wait for one of a set of flight modes'''
    start_time = time.time()
    last_mode = None
    while time.time() < start_time+timeout:
        wait_heartbeat(mav, timeout=10)
        if mav.flightmode != last_mode:
            print("Flightmode %s" % mav.flightmode)
            last_mode = mav.flightmode
        if mav.flightmode in modes:
            return
    print("Failed to get mode from %s" % modes)
    sys.exit(1)

def wait_time(mav, simtime):
    '''wait for simulation time to pass'''
    imu = mav.recv_match(type='RAW_IMU', blocking=True)
    t1 = imu.time_usec*1.0e-6
    while True:
        imu = mav.recv_match(type='RAW_IMU', blocking=True)
        t2 = imu.time_usec*1.0e-6
        if t2 - t1 > simtime:
            break

cmd = '../Tools/autotest/sim_vehicle.py -D'
mavproxy = pexpect.spawn(cmd, logfile=sys.stdout, timeout=30)
mavproxy.expect("Frame")

mav = mavutil.mavlink_connection('127.0.0.1:14550')

wait_mode(mav, ['STABILIZE'])
mavproxy.send('speedup 40\n')
mavproxy.expect('using GPS')
mavproxy.expect('using GPS')
mavproxy.expect('using GPS')
mavproxy.expect('using GPS')
mavproxy.send('arm throttle\n')
mavproxy.expect('Arming')
mavproxy.send('mode loiter\n')
wait_mode(mav, ['LOITER'])
mavproxy.send('rc 3 2000\n')
wait_time(mav, 20)
mavproxy.send('rc 3 1500\n')
mavproxy.send('mode CIRCLE\n')
wait_time(mav, 90)
