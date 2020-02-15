#!/usr/bin/env python

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

cmd = 'sim_vehicle.py -j4 -D -f plane'
mavproxy = pexpect.spawn(cmd, logfile=sys.stdout, timeout=30)
mavproxy.expect("Ground start complete")

mav = mavutil.mavlink_connection('127.0.0.1:14550')

wait_mode(mav, ['MANUAL'])
mavproxy.send('wp list\n')
mavproxy.expect('Requesting')
wait_time(mav, 30)
mavproxy.send('arm throttle\n')
mavproxy.send('auto\n')
wait_mode(mav, ['AUTO'])
mavproxy.send('module load console\n')
mavproxy.send('module load map\n')
mavproxy.send('map set showsimpos 1\n')
mavproxy.logfile = None
mavproxy.interact()
