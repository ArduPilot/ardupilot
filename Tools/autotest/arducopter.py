# fly ArduCopter in SITL
# Flight mode switch positions are set-up in arducopter.param to be
#   switch 1 = Circle
#   switch 2 = Land
#   switch 3 = RTL
#   switch 4 = Auto
#   switch 5 = Loiter
#   switch 6 = Stabilize

import util, pexpect, sys, time, math, shutil, os
from common import *
from pymavlink import mavutil, mavwp
import random

# get location of scripts
testdir=os.path.dirname(os.path.realpath(__file__))

FRAME='+'
TARGET='sitl'
HOME=mavutil.location(-35.362938,149.165085,584,270)
AVCHOME=mavutil.location(40.072842,-105.230575,1586,0)

homeloc = None
num_wp = 0
speedup_default = 5

def hover(mavproxy, mav, hover_throttle=1450):
    mavproxy.send('rc 3 %u\n' % hover_throttle)
    return True

def arm_motors(mavproxy, mav):
    '''arm motors'''
    print("Arming motors")
    mavproxy.send('switch 6\n') # stabilize mode
    wait_mode(mav, 'STABILIZE')
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 4 2000\n')
    mavproxy.expect('APM: Arming motors')
    mavproxy.send('rc 4 1500\n')
    mav.motors_armed_wait()
    print("MOTORS ARMED OK")
    return True

def disarm_motors(mavproxy, mav):
    '''disarm motors'''
    print("Disarming motors")
    mavproxy.send('switch 6\n') # stabilize mode
    wait_mode(mav, 'STABILIZE')
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 4 1000\n')
    mavproxy.expect('APM: Disarming motors')
    mavproxy.send('rc 4 1500\n')
    mav.motors_disarmed_wait()
    print("MOTORS DISARMED OK")
    return True


def takeoff(mavproxy, mav, alt_min = 30, takeoff_throttle=1700):
    '''takeoff get to 30m altitude'''
    mavproxy.send('switch 6\n') # stabilize mode
    wait_mode(mav, 'STABILIZE')
    mavproxy.send('rc 3 %u\n' % takeoff_throttle)
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    if (m.alt < alt_min):
        wait_altitude(mav, alt_min, (alt_min + 5))
    hover(mavproxy, mav)
    print("TAKEOFF COMPLETE")
    return True

# loiter - fly south west, then hold loiter within 5m position and altitude
def loiter(mavproxy, mav, holdtime=10, maxaltchange=5, maxdistchange=5):
    '''hold loiter position'''
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')

    # first aim south east
    print("turn south east")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 170):
        return False
    mavproxy.send('rc 4 1500\n')

    #fly south east 50m
    mavproxy.send('rc 2 1100\n')
    if not wait_distance(mav, 50):
        return False
    mavproxy.send('rc 2 1500\n')

    # wait for copter to slow moving
    if not wait_groundspeed(mav, 0, 2):
        return False

    success = True
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    start = mav.location()
    tstart = get_sim_time(mav)
    tholdstart = get_sim_time(mav)
    print("Holding loiter at %u meters for %u seconds" % (start_altitude, holdtime))
    while get_sim_time(mav) < tstart + holdtime:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        delta = get_distance(start, pos)
        alt_delta = math.fabs(m.alt - start_altitude)
        print("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
        if alt_delta > maxaltchange:
            print("Loiter alt shifted %u meters (> limit of %u)" % (alt_delta, maxaltchange))
            success = False
        if delta > maxdistchange:
            print("Loiter shifted %u meters (> limit of %u)" % (delta, maxdistchange))
            success = False
    if success:
        print("Loiter OK for %u seconds" % holdtime)
    else:
        print("Loiter FAILED")
    return success

def change_alt(mavproxy, mav, alt_min, climb_throttle=1920, descend_throttle=1080):
    '''change altitude'''
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    if(m.alt < alt_min):
        print("Rise to alt:%u from %u" % (alt_min, m.alt))
        mavproxy.send('rc 3 %u\n' % climb_throttle)
        wait_altitude(mav, alt_min, (alt_min + 5))
    else:
        print("Lower to alt:%u from %u" % (alt_min, m.alt))
        mavproxy.send('rc 3 %u\n' % descend_throttle)
        wait_altitude(mav, (alt_min -5), alt_min)
    hover(mavproxy, mav)
    return True

# fly a square in stabilize mode
def fly_square(mavproxy, mav, side=50, timeout=300):
    '''fly a square, flying N then E'''
    tstart = get_sim_time(mav)
    success = True

    # ensure all sticks in the middle
    mavproxy.send('rc 1 1500\n')
    mavproxy.send('rc 2 1500\n')
    mavproxy.send('rc 3 1500\n')
    mavproxy.send('rc 4 1500\n')

    # switch to loiter mode temporarily to stop us from rising
    mavproxy.send('switch 5\n')
    wait_mode(mav, 'LOITER')

    # first aim north
    print("turn right towards north")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 10):
        print("Failed to reach heading")
        success = False
    mavproxy.send('rc 4 1500\n')
    mav.recv_match(condition='RC_CHANNELS_RAW.chan4_raw==1500', blocking=True)

    # save bottom left corner of box as waypoint
    print("Save WP 1 & 2")
    save_wp(mavproxy, mav)

    # switch back to stabilize mode
    mavproxy.send('rc 3 1430\n')
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')

    # pitch forward to fly north
    print("Going north %u meters" % side)
    mavproxy.send('rc 2 1300\n')
    if not wait_distance(mav, side):
        print("Failed to reach distance of %u") % side
        success = False
    mavproxy.send('rc 2 1500\n')

    # save top left corner of square as waypoint
    print("Save WP 3")
    save_wp(mavproxy, mav)

    # roll right to fly east
    print("Going east %u meters" % side)
    mavproxy.send('rc 1 1700\n')
    if not wait_distance(mav, side):
        print("Failed to reach distance of %u") % side
        success = False
    mavproxy.send('rc 1 1500\n')

    # save top right corner of square as waypoint
    print("Save WP 4")
    save_wp(mavproxy, mav)

    # pitch back to fly south
    print("Going south %u meters" % side)
    mavproxy.send('rc 2 1700\n')
    if not wait_distance(mav, side):
        print("Failed to reach distance of %u") % side
        success = False
    mavproxy.send('rc 2 1500\n')

    # save bottom right corner of square as waypoint
    print("Save WP 5")
    save_wp(mavproxy, mav)

    # roll left to fly west
    print("Going west %u meters" % side)
    mavproxy.send('rc 1 1300\n')
    if not wait_distance(mav, side):
        print("Failed to reach distance of %u") % side
        success = False
    mavproxy.send('rc 1 1500\n')

    # save bottom left corner of square (should be near home) as waypoint
    print("Save WP 6")
    save_wp(mavproxy, mav)

    # descend to 10m
    print("Descend to 10m in Loiter")
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')
    mavproxy.send('rc 3 1300\n')
    time_left = timeout - (get_sim_time(mav) - tstart)
    print("timeleft = %u" % time_left)
    if time_left < 20:
        time_left = 20
    if not wait_altitude(mav, -10, 10, time_left):
        print("Failed to reach alt of 10m")
        success = False
    save_wp(mavproxy, mav)

    return success

def fly_RTL(mavproxy, mav, side=60, timeout=250):
    '''Return, land'''
    print("# Enter RTL")
    mavproxy.send('switch 3\n')
    tstart = get_sim_time(mav)
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(HOME, pos)
        print("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        if(m.alt <= 1 and home_distance < 10):
            return True
    return False

def fly_throttle_failsafe(mavproxy, mav, side=60, timeout=180):
    '''Fly east, Failsafe, return, land'''

    # switch to loiter mode temporarily to stop us from rising
    mavproxy.send('switch 5\n')
    wait_mode(mav, 'LOITER')

    # first aim east
    print("turn east")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 135):
        return False
    mavproxy.send('rc 4 1500\n')

    # switch to stabilize mode
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    hover(mavproxy, mav)
    failed = False

    # fly east 60 meters
    print("# Going forward %u meters" % side)
    mavproxy.send('rc 2 1350\n')
    if not wait_distance(mav, side, 5, 60):
        failed = True
    mavproxy.send('rc 2 1500\n')

    # pull throttle low
    print("# Enter Failsafe")
    mavproxy.send('rc 3 900\n')

    tstart = get_sim_time(mav)
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(HOME, pos)
        print("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        # check if we've reached home
        if m.alt <= 1 and home_distance < 10:
            # reduce throttle
            mavproxy.send('rc 3 1100\n')
            # switch back to stabilize
            mavproxy.send('switch 2\n') # land mode
            wait_mode(mav, 'LAND')
            mavproxy.send('switch 6\n') # stabilize mode
            wait_mode(mav, 'STABILIZE')
            print("Reached failsafe home OK")
            return True
    print("Failed to land on failsafe RTL - timed out after %u seconds" % timeout)
    # reduce throttle
    mavproxy.send('rc 3 1100\n')
    # switch back to stabilize mode
    mavproxy.send('switch 2\n') # land mode
    wait_mode(mav, 'LAND')
    mavproxy.send('switch 6\n') # stabilize mode
    wait_mode(mav, 'STABILIZE')
    return False

def fly_battery_failsafe(mavproxy, mav, timeout=30):
    # assume failure
    success = False

    # switch to loiter mode so that we hold position
    mavproxy.send('switch 5\n')
    wait_mode(mav, 'LOITER')
    mavproxy.send("rc 3 1500\n")

    # enable battery failsafe
    mavproxy.send("param set FS_BATT_ENABLE 1\n")

    # trigger low voltage
    mavproxy.send('param set SIM_BATT_VOLTAGE 10\n')

    # wait for LAND mode
    new_mode = wait_mode(mav, 'LAND')
    if new_mode == 'LAND':
        success = True

    # disable battery failsafe
    mavproxy.send('param set FS_BATT_ENABLE 0\n')

    # return status
    if success:
        print("Successfully entered LAND mode after battery failsafe")
    else:
        print("Failed to enter LAND mode after battery failsafe")

    return success

# fly_stability_patch - fly south, then hold loiter within 5m position and altitude and reduce 1 motor to 60% efficiency
def fly_stability_patch(mavproxy, mav, holdtime=30, maxaltchange=5, maxdistchange=10):
    '''hold loiter position'''
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')

    # first south
    print("turn south")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 180):
        return False
    mavproxy.send('rc 4 1500\n')

    #fly west 80m
    mavproxy.send('rc 2 1100\n')
    if not wait_distance(mav, 80):
        return False
    mavproxy.send('rc 2 1500\n')

    # wait for copter to slow moving
    if not wait_groundspeed(mav, 0, 2):
        return False

    success = True
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    start = mav.location()
    tstart = get_sim_time(mav)
    tholdstart = get_sim_time(mav)
    print("Holding loiter at %u meters for %u seconds" % (start_altitude, holdtime))

    # cut motor 1 to 55% efficiency
    print("Cutting motor 1 to 60% efficiency")
    mavproxy.send('param set SIM_ENGINE_MUL 0.60\n')

    while get_sim_time(mav) < tstart + holdtime:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        delta = get_distance(start, pos)
        alt_delta = math.fabs(m.alt - start_altitude)
        print("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
        if alt_delta > maxaltchange:
            print("Loiter alt shifted %u meters (> limit of %u)" % (alt_delta, maxaltchange))
            success = False
        if delta > maxdistchange:
            print("Loiter shifted %u meters (> limit of %u)" % (delta, maxdistchange))
            success = False

    # restore motor 1 to 100% efficiency
    mavproxy.send('param set SIM_ENGINE_MUL 1.0\n')

    if success:
        print("Stability patch and Loiter OK for %u seconds" % holdtime)            
    else:
        print("Stability Patch FAILED")

    return success

# fly_fence_test - fly east until you hit the horizontal circular fence
def fly_fence_test(mavproxy, mav, timeout=180):
    '''hold loiter position'''
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')

    # enable fence
    mavproxy.send('param set FENCE_ENABLE 1\n')

    # first east
    print("turn east")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 160):
        return False
    mavproxy.send('rc 4 1500\n')

    # fly forward (east) at least 20m
    pitching_forward = True
    mavproxy.send('rc 2 1100\n')
    if not wait_distance(mav, 20):
        return False

    # start timer
    tstart = get_sim_time(mav)
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(HOME, pos)
        print("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        # recenter pitch sticks once we reach home so we don't fly off again
        if pitching_forward and home_distance < 10 :
            pitching_forward = False
            mavproxy.send('rc 2 1500\n')
            # disable fence
            mavproxy.send('param set FENCE_ENABLE 0\n')
        if m.alt <= 1 and home_distance < 10:
            # reduce throttle
            mavproxy.send('rc 3 1000\n')
            # switch mode to stabilize
            mavproxy.send('switch 2\n') # land mode
            wait_mode(mav, 'LAND')
            mavproxy.send('switch 6\n') # stabilize mode
            wait_mode(mav, 'STABILIZE')
            print("Reached home OK")
            return True
    # disable fence
    mavproxy.send('param set FENCE_ENABLE 0\n')
    # reduce throttle
    mavproxy.send('rc 3 1000\n')
    # switch mode to stabilize
    mavproxy.send('switch 2\n') # land mode
    wait_mode(mav, 'LAND')
    mavproxy.send('switch 6\n') # stabilize mode
    wait_mode(mav, 'STABILIZE')
    print("Fence test failed to reach home - timed out after %u seconds" % timeout)
    return False

def show_gps_and_sim_positions(mavproxy, on_off):
    if on_off == True:
        # turn on simulator display of gps and actual position
        mavproxy.send('map set showgpspos 1\n')
        mavproxy.send('map set showsimpos 1\n')
    else:
        # turn off simulator display of gps and actual position
        mavproxy.send('map set showgpspos 0\n')
        mavproxy.send('map set showsimpos 0\n')
    
# fly_gps_glitch_loiter_test - fly south east in loiter and test reaction to gps glitch
def fly_gps_glitch_loiter_test(mavproxy, mav, timeout=30, max_distance=20):
    '''hold loiter position'''
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')

    # turn on simulator display of gps and actual position
    show_gps_and_sim_positions(mavproxy, True)

    # set-up gps glitch array
    glitch_lat = [0.0002996,0.0006958,0.0009431,0.0009991,0.0009444,0.0007716,0.0006221]
    glitch_lon = [0.0000717,0.0000912,0.0002761,0.0002626,0.0002807,0.0002049,0.0001304]
    glitch_num = len(glitch_lat)
    print("GPS Glitches:")
    for i in range(1,glitch_num):
        print("glitch %d %.7f %.7f" % (i,glitch_lat[i],glitch_lon[i]))

    # turn south east
    print("turn south east")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 150):
        show_gps_and_sim_positions(mavproxy, False)
        return False
    mavproxy.send('rc 4 1500\n')

    # fly forward (south east) at least 60m
    mavproxy.send('rc 2 1100\n')
    if not wait_distance(mav, 60):
        show_gps_and_sim_positions(mavproxy, False)
        return False
    mavproxy.send('rc 2 1500\n')

    # wait for copter to slow down
    if not wait_groundspeed(mav, 0, 1):
        show_gps_and_sim_positions(mavproxy, False)
        return False

    # record time and position
    tstart = get_sim_time(mav)
    tnow = tstart
    start_pos = sim_location(mav)
    success = True

    # initialise current glitch
    glitch_current = 0;
    print("Apply first glitch")
    mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
    mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

    # record position for 30 seconds
    while tnow < tstart + timeout:
        tnow = get_sim_time(mav)
        desired_glitch_num = int((tnow - tstart) * 2.2)
        if desired_glitch_num > glitch_current and glitch_current != -1:
            glitch_current = desired_glitch_num
            # turn off glitching if we've reached the end of the glitch list
            if glitch_current >= glitch_num:
                glitch_current = -1
                print("Completed Glitches")
                mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
                mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')
            else:
                print("Applying glitch %u" % glitch_current)
                #move onto the next glitch
                mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
                mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

        # start displaying distance moved after all glitches applied
        if (glitch_current == -1):
            m = mav.recv_match(type='VFR_HUD', blocking=True)
            curr_pos = sim_location(mav)
            moved_distance = get_distance(curr_pos, start_pos)
            print("Alt: %u  Moved: %.0f" % (m.alt, moved_distance))
            if moved_distance > max_distance:
                print("Moved over %u meters, Failed!" % max_distance)
                success = False

    # disable gps glitch
    if glitch_current != -1:
        glitch_current = -1
        mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
        mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')
    show_gps_and_sim_positions(mavproxy, False)

    if success:
        print("GPS glitch test passed!  stayed within %u meters for %u seconds" % (max_distance, timeout))
    else:
        print("GPS glitch test FAILED!")
    return success

# fly_gps_glitch_auto_test - fly mission and test reaction to gps glitch
def fly_gps_glitch_auto_test(mavproxy, mav, timeout=120):

    # set-up gps glitch array
    glitch_lat = [0.0002996,0.0006958,0.0009431,0.0009991,0.0009444,0.0007716,0.0006221]
    glitch_lon = [0.0000717,0.0000912,0.0002761,0.0002626,0.0002807,0.0002049,0.0001304]
    glitch_num = len(glitch_lat)
    print("GPS Glitches:")
    for i in range(1,glitch_num):
        print("glitch %d %.7f %.7f" % (i,glitch_lat[i],glitch_lon[i]))

    # Fly mission #1
    print("# Load copter_glitch_mission")
    if not load_mission_from_file(mavproxy, mav, os.path.join(testdir, "copter_glitch_mission.txt")):
        print("load copter_glitch_mission failed")
        return False

    # turn on simulator display of gps and actual position
    show_gps_and_sim_positions(mavproxy, True)

    # load the waypoint count
    global homeloc
    global num_wp
    print("test: Fly a mission from 1 to %u" % num_wp)
    mavproxy.send('wp set 1\n')

    # switch into AUTO mode and raise throttle
    mavproxy.send('switch 4\n') # auto mode
    wait_mode(mav, 'AUTO')
    mavproxy.send('rc 3 1500\n')

    # wait until 100m from home
    if not wait_distance(mav, 100, 5, 60):
        show_gps_and_sim_positions(mavproxy, False)
        return False

    # record time and position
    tstart = get_sim_time(mav)
    tnow = tstart

    # initialise current glitch
    glitch_current = 0;
    print("Apply first glitch")
    mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
    mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

    # record position for 30 seconds
    while glitch_current < glitch_num:
        tnow = get_sim_time(mav)
        desired_glitch_num = int((tnow - tstart) * 2.2)
        if desired_glitch_num > glitch_current and glitch_current != -1:
            glitch_current = desired_glitch_num
            # apply next glitch
            if glitch_current < glitch_num:
                print("Applying glitch %u" % glitch_current)
                mavproxy.send('param set SIM_GPS_GLITCH_X %.7f\n' % glitch_lat[glitch_current])
                mavproxy.send('param set SIM_GPS_GLITCH_Y %.7f\n' % glitch_lon[glitch_current])

    # turn off glitching
    print("Completed Glitches")
    mavproxy.send('param set SIM_GPS_GLITCH_X 0\n')
    mavproxy.send('param set SIM_GPS_GLITCH_Y 0\n')

    # continue with the mission
    ret = wait_waypoint(mav, 0, num_wp-1, timeout=500)

    # wait for arrival back home
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    pos = mav.location()
    dist_to_home = get_distance(HOME, pos)
    while dist_to_home > 5:
        if get_sim_time(mav) > (tstart + timeout):
            print("GPS Glitch testing failed - exceeded timeout %u seconds" % timeout)
            ret = False
            break
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        dist_to_home = get_distance(HOME, pos)
        print("Dist from home: %u" % dist_to_home)

    # turn off simulator display of gps and actual position
    show_gps_and_sim_positions(mavproxy, False)

    print("GPS Glitch test Auto completed: passed=%s" % ret)

    return ret

#fly_simple - assumes the simple bearing is initialised to be directly north
#   flies a box with 100m west, 15 seconds north, 50 seconds east, 15 seconds south
def fly_simple(mavproxy, mav, side=50, timeout=120):

    failed = False

    # hold position in loiter
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')

    #set SIMPLE mode for all flight modes
    mavproxy.send('param set SIMPLE 63\n')

    # switch to stabilize mode
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    mavproxy.send('rc 3 1430\n')

    # fly south 50m
    print("# Flying south %u meters" % side)
    mavproxy.send('rc 1 1300\n')
    if not wait_distance(mav, side, 5, 60):
        failed = True
    mavproxy.send('rc 1 1500\n')

    # fly west 8 seconds
    print("# Flying west for 8 seconds")
    mavproxy.send('rc 2 1300\n')
    tstart = get_sim_time(mav)
    while get_sim_time(mav) < (tstart + 8):
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        delta = (get_sim_time(mav) - tstart)
        #print("%u" % delta)
    mavproxy.send('rc 2 1500\n')

    # fly north 25 meters
    print("# Flying north %u meters" % (side/2.0))
    mavproxy.send('rc 1 1700\n')
    if not wait_distance(mav, side/2, 5, 60):
        failed = True
    mavproxy.send('rc 1 1500\n')

    # fly east 8 seconds
    print("# Flying east for 8 seconds")
    mavproxy.send('rc 2 1700\n')
    tstart = get_sim_time(mav)
    while get_sim_time(mav) < (tstart + 8):
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        delta = (get_sim_time(mav) - tstart)
        #print("%u" % delta)
    mavproxy.send('rc 2 1500\n')

    #restore to default
    mavproxy.send('param set SIMPLE 0\n')

    #hover in place
    hover(mavproxy, mav)
    return not failed

#fly_super_simple - flies a circle around home for 45 seconds
def fly_super_simple(mavproxy, mav, timeout=45):

    failed = False

    # hold position in loiter
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')

    # fly forward 20m
    print("# Flying forward 20 meters")
    mavproxy.send('rc 2 1300\n')
    if not wait_distance(mav, 20, 5, 60):
        failed = True
    mavproxy.send('rc 2 1500\n')

    #set SUPER SIMPLE mode for all flight modes
    mavproxy.send('param set SUPER_SIMPLE 63\n')

    # switch to stabilize mode
    mavproxy.send('switch 6\n')
    wait_mode(mav, 'STABILIZE')
    mavproxy.send('rc 3 1430\n')

    # start copter yawing slowly
    mavproxy.send('rc 4 1550\n')

    # roll left for timeout seconds
    print("# rolling left from pilot's point of view for %u seconds" % timeout)
    mavproxy.send('rc 1 1300\n')
    tstart = get_sim_time(mav)
    while get_sim_time(mav) < (tstart + timeout):
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        delta = (get_sim_time(mav) - tstart)

    # stop rolling and yawing
    mavproxy.send('rc 1 1500\n')
    mavproxy.send('rc 4 1500\n')

    #restore simple mode parameters to default
    mavproxy.send('param set SUPER_SIMPLE 0\n')

    #hover in place
    hover(mavproxy, mav)
    return not failed

#fly_circle - flies a circle with 20m radius
def fly_circle(mavproxy, mav, maxaltchange=10, holdtime=36):

    # hold position in loiter
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')

    # face west
    print("turn west")
    mavproxy.send('rc 4 1580\n')
    if not wait_heading(mav, 270):
        return False
    mavproxy.send('rc 4 1500\n')

    #set CIRCLE radius
    mavproxy.send('param set CIRCLE_RADIUS 3000\n')

    # fly forward (east) at least 100m
    mavproxy.send('rc 2 1100\n')
    if not wait_distance(mav, 100):
        return False

    # return pitch stick back to middle
    mavproxy.send('rc 2 1500\n')

    # set CIRCLE mode
    mavproxy.send('switch 1\n') # circle mode
    wait_mode(mav, 'CIRCLE')

    # wait 
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    tstart = get_sim_time(mav)
    tholdstart = get_sim_time(mav)
    print("Circle at %u meters for %u seconds" % (start_altitude, holdtime))
    while get_sim_time(mav) < tstart + holdtime:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("heading %u" % m.heading)

    print("CIRCLE OK for %u seconds" % holdtime)
    return True

# fly_auto_test - fly mission which tests a significant number of commands
def fly_auto_test(mavproxy, mav):

    # Fly mission #1
    print("# Load copter_mission")
    if not load_mission_from_file(mavproxy, mav, os.path.join(testdir, "copter_mission.txt")):
        print("load copter_mission failed")
        return False

    # load the waypoint count
    global homeloc
    global num_wp
    print("test: Fly a mission from 1 to %u" % num_wp)
    mavproxy.send('wp set 1\n')

    # switch into AUTO mode and raise throttle
    mavproxy.send('switch 4\n') # auto mode
    wait_mode(mav, 'AUTO')
    mavproxy.send('rc 3 1500\n')

    # fly the mission
    ret = wait_waypoint(mav, 0, num_wp-1, timeout=500)

    # land if mission failed
    if ret == False:
        land(mavproxy, mav)
    
    # set throttle to minimum
    mavproxy.send('rc 3 1000\n')

    # wait for disarm
    mav.motors_disarmed_wait()
    print("MOTORS DISARMED OK")

    print("Auto mission completed: passed=%s" % ret)

    return ret

# fly_avc_test - fly AVC mission
def fly_avc_test(mavproxy, mav):
        
    # upload mission from file
    print("# Load copter_AVC2013_mission")
    if not load_mission_from_file(mavproxy, mav, os.path.join(testdir, "copter_AVC2013_mission.txt")):
        print("load copter_AVC2013_mission failed")
        return False

    # load the waypoint count
    global homeloc
    global num_wp
    print("Fly AVC mission from 1 to %u" % num_wp)
    mavproxy.send('wp set 1\n')

    # switch into AUTO mode and raise throttle
    mavproxy.send('switch 4\n') # auto mode
    wait_mode(mav, 'AUTO')
    mavproxy.send('rc 3 1500\n')

    # fly the mission
    ret = wait_waypoint(mav, 0, num_wp-1, timeout=500)

    # set throttle to minimum
    mavproxy.send('rc 3 1000\n')

    # wait for disarm
    mav.motors_disarmed_wait()
    print("MOTORS DISARMED OK")

    print("AVC mission completed: passed=%s" % ret)

    return ret

def land(mavproxy, mav, timeout=60):
    '''land the quad'''
    print("STARTING LANDING")
    mavproxy.send('switch 2\n') # land mode
    wait_mode(mav, 'LAND')
    print("Entered Landing Mode")
    ret = wait_altitude(mav, -5, 1)
    print("LANDING: ok= %s" % ret)
    return ret

def fly_mission(mavproxy, mav, height_accuracy=-1, target_altitude=None):
    '''fly a mission from a file'''
    global homeloc
    global num_wp
    print("test: Fly a mission from 1 to %u" % num_wp)
    mavproxy.send('wp set 1\n')
    mavproxy.send('switch 4\n') # auto mode
    wait_mode(mav, 'AUTO')
    ret = wait_waypoint(mav, 0, num_wp-1, timeout=500)
    expect_msg = "Reached command #%u" % (num_wp-1)
    if (ret):
        mavproxy.expect(expect_msg)
    print("test: MISSION COMPLETE: passed=%s" % ret)
    # wait here until ready
    mavproxy.send('switch 5\n') # loiter mode
    wait_mode(mav, 'LOITER')
    return ret

def load_mission_from_file(mavproxy, mav, filename):
    '''Load a mission from a file to flight controller'''
    global num_wp
    mavproxy.send('wp load %s\n' % filename)
    mavproxy.expect('Flight plan received')
    mavproxy.send('wp list\n')
    mavproxy.expect('Requesting [0-9]+ waypoints')

    # update num_wp
    wploader = mavwp.MAVWPLoader()
    wploader.load(filename)
    num_wp = wploader.count()
    return True

def save_mission_to_file(mavproxy, mav, filename):
    global num_wp
    mavproxy.send('wp save %s\n' % filename)
    mavproxy.expect('Saved ([0-9]+) waypoints')
    num_wp = int(mavproxy.match.group(1))
    print("num_wp: %d" % num_wp)
    return True

def setup_rc(mavproxy):
    '''setup RC override control'''
    for chan in range(1,9):
        mavproxy.send('rc %u 1500\n' % chan)
    # zero throttle
    mavproxy.send('rc 3 1000\n')

def fly_ArduCopter(viewerip=None, map=False, valgrind=False):
    '''fly ArduCopter in SIL

    you can pass viewerip as an IP address to optionally send fg and
    mavproxy packets too for local viewing of the flight in real time
    '''
    global homeloc

    if TARGET != 'sitl':
        util.build_SIL('ArduCopter', target=TARGET)

    home = "%f,%f,%u,%u" % (HOME.lat, HOME.lng, HOME.alt, HOME.heading)
    sil = util.start_SIL('ArduCopter', wipe=True, model='+', home=home, speedup=speedup_default)
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options='--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter')
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send("param load %s/copter_params.parm\n" % testdir)
    mavproxy.expect('Loaded [0-9]+ parameters')

    # reboot with new parameters
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)

    sil = util.start_SIL('ArduCopter', model='+', home=home, speedup=speedup_default, valgrind=valgrind)
    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter --streamrate=5'
    if viewerip:
        options += ' --out=%s:14550' % viewerip
    if map:
        options += ' --map'
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options=options)
    mavproxy.expect('Telemetry log: (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/ArduCopter-test.tlog")
    print("buildlog=%s" % buildlog)
    copyTLog = False
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    try:
        os.link(logfile, buildlog)
    except Exception:
        print( "WARN: Failed to create symlink: " + logfile + " => " + buildlog + ", Will copy tlog manually to target location" )
        copyTLog = True

    # the received parameters can come before or after the ready to fly message
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])

    util.expect_setup_callback(mavproxy, expect_callback)

    expect_list_clear()
    expect_list_extend([sil, mavproxy])

    # get a mavlink connection going
    try:
        mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
    except Exception, msg:
        print("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
        raise
    mav.message_hooks.append(message_hook)
    mav.idle_hooks.append(idle_hook)


    failed = False
    failed_test_msg = "None"

    try:
        mav.wait_heartbeat()
        setup_rc(mavproxy)
        homeloc = mav.location()

        # wait 10sec to allow EKF to settle
        wait_seconds(mav, 10)

        # Arm
        print("# Arm motors")
        if not arm_motors(mavproxy, mav):
            failed_test_msg = "arm_motors failed"
            print(failed_test_msg)
            failed = True

        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            print(failed_test_msg)
            failed = True

        # Fly a square in Stabilize mode
        print("#")
        print("########## Fly a square and save WPs with CH7 switch ##########")
        print("#")
        if not fly_square(mavproxy, mav):
            failed_test_msg = "fly_square failed"
            print(failed_test_msg)
            failed = True

        # save the stored mission to file
        print("# Save out the CH7 mission to file")
        if not save_mission_to_file(mavproxy, mav, os.path.join(testdir, "ch7_mission.txt")):
            failed_test_msg = "save_mission_to_file failed"
            print(failed_test_msg)
            failed = True

        # fly the stored mission
        print("# Fly CH7 saved mission")
        if not fly_mission(mavproxy, mav,height_accuracy = 0.5, target_altitude=10):
            failed_test_msg = "fly ch7_mission failed"
            print(failed_test_msg)
            failed = True

        # Throttle Failsafe
        print("#")
        print("########## Test Failsafe ##########")
        print("#")
        if not fly_throttle_failsafe(mavproxy, mav):
            failed_test_msg = "fly_throttle_failsafe failed"
            print(failed_test_msg)
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            print(failed_test_msg)
            failed = True

        # Battery failsafe
        if not fly_battery_failsafe(mavproxy, mav):
            failed_test_msg = "fly_battery_failsafe failed"
            print(failed_test_msg)
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            print(failed_test_msg)
            failed = True

        # Stability patch
        print("#")
        print("########## Test Stability Patch ##########")
        print("#")
        if not fly_stability_patch(mavproxy, mav, 30):
            failed_test_msg = "fly_stability_patch failed"
            print(failed_test_msg)
            failed = True

        # RTL
        print("# RTL #")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL after stab patch failed"
            print(failed_test_msg)
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            print(failed_test_msg)
            failed = True

        # Fence test
        print("#")
        print("########## Test Horizontal Fence ##########")
        print("#")
        if not fly_fence_test(mavproxy, mav, 180):
            failed_test_msg = "fly_fence_test failed"
            print(failed_test_msg)
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            print(failed_test_msg)
            failed = True

        # Fly GPS Glitch Loiter test
        print("# GPS Glitch Loiter Test")
        if not fly_gps_glitch_loiter_test(mavproxy, mav):
            failed_test_msg = "fly_gps_glitch_loiter_test failed"
            print(failed_test_msg)
            failed = True

        # RTL after GPS Glitch Loiter test
        print("# RTL #")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL failed"
            print(failed_test_msg)
            failed = True

        # Fly GPS Glitch test in auto mode
        print("# GPS Glitch Auto Test")
        if not fly_gps_glitch_auto_test(mavproxy, mav):
            failed_test_msg = "fly_gps_glitch_auto_test failed"
            print(failed_test_msg)
            failed = True

        # take-off ahead of next test
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            print(failed_test_msg)
            failed = True

        # Loiter for 10 seconds
        print("#")
        print("########## Test Loiter for 10 seconds ##########")
        print("#")
        if not loiter(mavproxy, mav):
            failed_test_msg = "loiter failed"
            print(failed_test_msg)
            failed = True

        # Loiter Climb
        print("#")
        print("# Loiter - climb to 30m")
        print("#")
        if not change_alt(mavproxy, mav, 30):
            failed_test_msg = "change_alt climb failed"
            print(failed_test_msg)
            failed = True

        # Loiter Descend
        print("#")
        print("# Loiter - descend to 20m")
        print("#")
        if not change_alt(mavproxy, mav, 20):
            failed_test_msg = "change_alt descend failed"
            print(failed_test_msg)
            failed = True

        # RTL
        print("#")
        print("########## Test RTL ##########")
        print("#")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL after Loiter climb/descend failed"
            print(failed_test_msg)
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            print(failed_test_msg)
            failed = True

        # Simple mode
        print("# Fly in SIMPLE mode")
        if not fly_simple(mavproxy, mav):
            failed_test_msg = "fly_simple failed"
            print(failed_test_msg)
            failed = True

        # RTL
        print("#")
        print("########## Test RTL ##########")
        print("#")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL after simple mode failed"
            print(failed_test_msg)
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            print(failed_test_msg)
            failed = True

        # Fly a circle in super simple mode
        print("# Fly a circle in SUPER SIMPLE mode")
        if not fly_super_simple(mavproxy, mav):
            failed_test_msg = "fly_super_simple failed"
            print(failed_test_msg)
            failed = True

        # RTL
        print("# RTL #")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL after super simple mode failed"
            print(failed_test_msg)
            failed = True

        # Takeoff
        print("# Takeoff")
        if not takeoff(mavproxy, mav, 10):
            failed_test_msg = "takeoff failed"
            print(failed_test_msg)
            failed = True

        # Circle mode
        print("# Fly CIRCLE mode")
        if not fly_circle(mavproxy, mav):
            failed_test_msg = "fly_circle failed"
            print(failed_test_msg)
            failed = True

        # RTL
        print("#")
        print("########## Test RTL ##########")
        print("#")
        if not fly_RTL(mavproxy, mav):
            failed_test_msg = "fly_RTL after circle failed"
            print(failed_test_msg)
            failed = True

        print("# Fly copter mission")
        if not fly_auto_test(mavproxy, mav):
            failed_test_msg = "fly_auto_test failed"
            print(failed_test_msg)
            failed = True
        else:
            print("Flew copter mission OK")

        # wait for disarm
        mav.motors_disarmed_wait()

        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/ArduCopter-log.bin")):
            failed_test_msg = "log_download failed"
            print(failed_test_msg)
            failed = True

    except pexpect.TIMEOUT, failed_test_msg:
        failed_test_msg = "Timeout"
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)

    if os.path.exists('ArduCopter-valgrind.log'):
        os.chmod('ArduCopter-valgrind.log', 0644)
        shutil.copy("ArduCopter-valgrind.log", util.reltopdir("../buildlogs/ArduCopter-valgrind.log"))

    # [2014/05/07] FC Because I'm doing a cross machine build (source is on host, build is on guest VM) I cannot hard link
    # This flag tells me that I need to copy the data out
    if copyTLog:
        shutil.copy(logfile, buildlog)
        
    if failed:
        print("FAILED: %s" % failed_test_msg)
        return False
    return True


def fly_CopterAVC(viewerip=None, map=False, valgrind=False):
    '''fly ArduCopter in SIL for AVC2013 mission
    '''
    global homeloc

    if TARGET != 'sitl':
        util.build_SIL('ArduCopter', target=TARGET)

    home = "%f,%f,%u,%u" % (AVCHOME.lat, AVCHOME.lng, AVCHOME.alt, AVCHOME.heading)
    sil = util.start_SIL('ArduCopter', wipe=True, model='heli', home=home, speedup=speedup_default)
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options='--sitl=127.0.0.1:5501 --out=127.0.0.1:19550')
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send("param load %s/Helicopter.parm\n" % testdir)
    mavproxy.expect('Loaded [0-9]+ parameters')

    # reboot with new parameters
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)

    sil = util.start_SIL('ArduCopter', model='heli', home=home, speedup=speedup_default, valgrind=valgrind)
    options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=5'
    if viewerip:
        options += ' --out=%s:14550' % viewerip
    if map:
        options += ' --map'
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options=options)
    mavproxy.expect('Telemetry log: (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)

    buildlog = util.reltopdir("../buildlogs/CopterAVC-test.tlog")
    print("buildlog=%s" % buildlog)
    if os.path.exists(buildlog):
        os.unlink(buildlog)
    try:
        os.link(logfile, buildlog)
    except Exception:
        pass

    # the received parameters can come before or after the ready to fly message
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])
    mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])

    util.expect_setup_callback(mavproxy, expect_callback)

    expect_list_clear()
    expect_list_extend([sil, mavproxy])

    if map:
        mavproxy.send('map icon 40.072467969730496 -105.2314389590174\n')
        mavproxy.send('map icon 40.072600990533829 -105.23146100342274\n')        

    # get a mavlink connection going
    try:
        mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
    except Exception, msg:
        print("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
        raise
    mav.message_hooks.append(message_hook)
    mav.idle_hooks.append(idle_hook)


    failed = False
    failed_test_msg = "None"

    try:
        mav.wait_heartbeat()
        setup_rc(mavproxy)
        homeloc = mav.location()

        print("Lowering rotor speed")
        mavproxy.send('rc 8 1000\n')

        # wait 20sec to allow EKF to settle
        wait_seconds(mav, 20)

        # Arm
        print("# Arm motors")
        if not arm_motors(mavproxy, mav):
            failed_test_msg = "arm_motors failed"
            print(failed_test_msg)
            failed = True

        print("Raising rotor speed")
        mavproxy.send('rc 8 2000\n')

        print("# Fly AVC mission")
        if not fly_avc_test(mavproxy, mav):
            failed_test_msg = "fly_avc_test failed"
            print(failed_test_msg)
            failed = True
        else:
            print("Flew AVC mission OK")

        print("Lowering rotor speed")
        mavproxy.send('rc 8 1000\n')

        #mission includes disarm at end so should be ok to download logs now
        if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/CopterAVC-log.bin")):
            failed_test_msg = "log_download failed"
            print(failed_test_msg)
            failed = True

    except pexpect.TIMEOUT, failed_test_msg:
        failed_test_msg = "Timeout"
        failed = True

    mav.close()
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)

    if failed:
        print("FAILED: %s" % failed_test_msg)
        return False
    return True
