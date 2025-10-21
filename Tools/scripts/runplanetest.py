#!/usr/bin/env python3

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

def wait_prearm_ok(mav, timeout=30):
    '''wait for pre-arm OK'''
    start_time = time.time()
    last_mode = None
    while time.time() < start_time+timeout:
        m = mav.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
        if m is None:
            return
        if m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK != 0:
            print("Prearm OK")
            return
    print("Failed to get pre-arm OK")
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

cmd = '../Tools/autotest/sim_vehicle.py -D -f quadplane'
mavproxy = pexpect.spawn(cmd, logfile=sys.stdout.buffer, timeout=30)
mavproxy.expect("ArduPilot Ready")

mav = mavutil.mavlink_connection('127.0.0.1:14550')

mavproxy.send('speedup 40\n')
wait_prearm_ok(mav)
mavproxy.send('mode guided\n')
wait_mode(mav, ['GUIDED'])
mavproxy.send('arm throttle\n')
mavproxy.send('takeoff 40\n')
wait_time(mav, 30)
mavproxy.send('mode cruise\n')
wait_mode(mav, ['CRUISE'])
wait_time(mav, 10)
mavproxy.send('mode qrtl\n')
wait_mode(mav, ['QRTL'])
mavproxy.send('module load console\n')
mavproxy.send('module load map\n')
mavproxy.logfile = None
mavproxy.interact()
