import arducopter

import util, pexpect, sys, time, math, shutil, os
from common import *
import mavutil, mavwp, random

def unit_test(mavproxy, mav):
    '''A scripted flight plan for testing AP_Limits'''
 
    time.sleep(5)   
    print "# Setting AP_Limits parameters"
    mavproxy.send('param set LIM_ENABLED 1\n')
    mavproxy.send('param set LIM_REQUIRED 0\n')
    mavproxy.send('param set LIM_DEBUG 1\n')
    mavproxy.send('param set LIM_SAFETIME 1\n')
    
    mavproxy.send('param set LIM_ALT_ON 1\n')
    mavproxy.send('param set LIM_ALT_REQ 0\n')
    mavproxy.send('param set LIM_ALT_MIN 0\n')
    mavproxy.send('param set LIM_ALT_MAX 50\n')
    
    mavproxy.send('param set LIM_FNC_ON 0\n')
    mavproxy.send('param set LIM_FNC_REQ 0\n')
    mavproxy.send('param set LIM_FNC_SMPL 1\n')
    mavproxy.send('param set LIM_FNC_RAD 50\n')
    
    time.sleep(5)
    print "# Listing AP_Limits parameters"
    mavproxy.send('param show LIM*\n')

    if ( 
        arducopter.calibrate_level(mavproxy, mav) and
        arducopter.arm_motors(mavproxy, mav) and  
        arducopter.takeoff(mavproxy,mav, alt_min=30, takeoff_throttle=1510) and
        arducopter.hover(mavproxy, mav, hover_throttle=1500)
    ):
    
    # Trigger for ALT_MAX
        climb_rate = 0
        previous_alt = 0
        timeout = 30
    
         # Do Not Exceed altitude
        alt_dne = 55
        tstart = time.time()
        mavproxy.send('rc 3 1550\n')
        while (time.time() < tstart + timeout):
            m = mav.recv_match(type='VFR_HUD', blocking=True)
            climb_rate = m.alt - previous_alt
            previous_alt = m.alt
            print("Trigger Altitude Limit: Cur:%u, climb_rate: %u" % (m.alt, climb_rate))
            if abs(climb_rate) > 0:
                tstart = time.time();
            if (mav.recv_match(condition='MAV.flightmode=="GUIDED"', blocking=False) != None):
                print "Triggered!"
                return True
            if m.alt >= alt_dne :
                print("Altitude Exceeded")
                return False
        return False
    return False

