#!/usr/bin/env python

import os
from common import *
from pymavlink import mavwp, mavutil

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))


#################################################
# TESTS COMMONS
#################################################
def test_common_feature(mavproxy, mav):
    failed = False
    # TEST ARMING/DISARM
    if not arm_vehicle(mavproxy, mav):
        progress("Failed to ARM")
        failed = True
    if not disarm_vehicle(mavproxy, mav):
        progress("Failed to DISARM")
        failed = True
    if not test_arm_motors_radio(mavproxy, mav):
        progress("Failed to ARM with radio")
        failed = True
    if not test_disarm_motors_radio(mavproxy, mav):
        progress("Failed to ARM with radio")
        failed = True
    if not test_autodisarm_motors(mavproxy, mav):
        progress("Failed to AUTO DISARM")
        failed = True
    # TODO: Test failure on arm (with arming check)
    # TEST MISSION FILE
    progress("TEST LOADING MISSION")
    num_wp = load_mission_from_file(mavproxy, os.path.join(testdir, "all_msg_mission.txt"))
    if num_wp == 0:
        progress("load all_msg_mission failed")
        failed = True

    progress("TEST SAVING MISSION")
    num_wp_old = num_wp
    num_wp = save_mission_to_file(mavproxy, os.path.join(testdir, "all_msg_mission2.txt"))
    if num_wp != num_wp_old:
        progress("save all_msg_mission failed")
        failed = True

    progress("TEST CLEARING MISSION")
    mavproxy.send("wp clear\n")
    num_wp = mavwp.MAVWPLoader().count()
    if num_wp != 0:
        progress("clear mission failed")
        failed = True

    return failed


# TESTS FAILSAFE
def test_throttle_failsafe(mavproxy, mav, home, side=60, timeout=180):
    """Fly east, Failsafe, return, land."""

    if mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_TYPE_HELICOPTER,
                        mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                        mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                        mavutil.mavlink.MAV_TYPE_COAXIAL,
                        mavutil.mavlink.MAV_TYPE_TRICOPTER]:
        # switch to loiter mode temporarily to stop us from rising
        mavproxy.send('switch 5\n')
        wait_mode(mav, 'LOITER')
    if mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
        mavproxy.send('switch 6\n')  # manual mode
        wait_mode(mav, 'MANUAL')
        mavproxy.send("param set FS_ACTION 1\n")

    # first aim east
    progress("turn east")
    if not reach_heading(mavproxy, mav, 135):
        return False

    if mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_TYPE_HELICOPTER,
                        mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                        mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                        mavutil.mavlink.MAV_TYPE_COAXIAL,
                        mavutil.mavlink.MAV_TYPE_TRICOPTER]:
        # raise throttle slightly to avoid hitting the ground
        mavproxy.send('rc 3 1600\n')
        # switch to stabilize mode
        mavproxy.send('switch 6\n')
        wait_mode(mav, 'STABILIZE')
        mavproxy.send('rc 3 1500\n')

    # fly east 60 meters
    progress("# Going forward %u meters" % side)
    if not reach_distance(mavproxy, mav, side):
        return False

    # pull throttle low
    progress("# Enter Failsafe")
    mavproxy.send('rc 3 900\n')

    tstart = get_sim_time(mav)
    success = False
    while get_sim_time(mav) < tstart + timeout and not success:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(home, pos)
        progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        # check if we've reached home
        if (m.alt - home.alt) <= 1 and home_distance < 10:
            success = True

    if mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                            mavutil.mavlink.MAV_TYPE_HELICOPTER,
                            mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                            mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                            mavutil.mavlink.MAV_TYPE_COAXIAL,
                            mavutil.mavlink.MAV_TYPE_TRICOPTER]:
        # reduce throttle
        mavproxy.send('rc 3 1100\n')
        # switch back to stabilize mode
        mavproxy.send('switch 2\n')  # land mode
        wait_mode(mav, 'LAND')
        mavproxy.send('switch 6\n')  # stabilize mode
        wait_mode(mav, 'STABILIZE')
    if mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
        wait_mode(mav, 'HOLD')
        # reduce throttle
        mavproxy.send('rc 3 1500\n')
        mavproxy.expect('APM: Failsafe ended')
        mavproxy.send('switch 2\n')  # manual mode
        mavproxy.send('switch 6\n')
        wait_mode(mav, 'MANUAL')

    if success:
        progress("Reached failsafe home OK")
        return True
    else:
        progress("Failed to land on failsafe RTL - timed out after %u seconds" % timeout)
        return False


##################################################################
#   IMPLEMENTATIONS
##################################################################
# TEST ARM RADIO
def test_arm_motors_radio(mavproxy, mav):
    """Test Arming motors with radio."""
    progress("Test arming motors with radio")
    mavproxy.send('switch 6\n')  # stabilize/manual mode
    if mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_TYPE_HELICOPTER,
                        mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                        mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                        mavutil.mavlink.MAV_TYPE_COAXIAL,
                        mavutil.mavlink.MAV_TYPE_TRICOPTER]:
        wait_mode(mav, 'STABILIZE')
        mavproxy.send('rc 3 1000\n')  # throttle at zero
        mavproxy.send('rc 4 2000\n')  # yaw full right
        mavproxy.expect('APM: Arming motors')
        mavproxy.send('rc 4 1500\n')
    if mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
        wait_mode(mav, 'STABILIZE')
    if mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
        wait_mode(mav, 'MANUAL')
        mavproxy.send('rc 3 1500\n')  # throttle at zero
        mavproxy.send('rc 1 2000\n')  # steer full right
        mavproxy.expect('APM: Throttle armed')
        mavproxy.send('rc 1 1500\n')

    mav.motors_armed_wait()
    progress("MOTORS ARMED OK")
    return True


# TEST DISARM RADIO
def test_disarm_motors_radio(mavproxy, mav):
    """Test Disarm motors with radio."""
    progress("Test disarming motors with radio")
    mavproxy.send('switch 6\n')  # stabilize/manual mode
    if mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_TYPE_HELICOPTER,
                        mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                        mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                        mavutil.mavlink.MAV_TYPE_COAXIAL,
                        mavutil.mavlink.MAV_TYPE_TRICOPTER]:
        wait_mode(mav, 'STABILIZE')
        mavproxy.send('rc 3 1000\n')  # throttle at zero
        mavproxy.send('rc 4 1000\n')  # yaw full right
    if mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
        wait_mode(mav, 'STABILIZE')
    if mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
        wait_mode(mav, 'MANUAL')
        mavproxy.send('rc 3 1500\n')  # throttle at zero
        mavproxy.send('rc 1 1000\n')  # steer full right

    tstart = get_sim_time(mav)
    timeout = 15
    while get_sim_time(mav) < tstart + timeout:
        if not mav.motors_armed():
            progress("MOTORS DISARMED OK WITH RADIO")
            mavproxy.send('rc 1 1500\n')  # steer full right
            mavproxy.send('rc 4 1500\n')  # yaw full right
            return True
    progress("FAILED TO DISARM WITH RADIO")
    return False


# TEST AUTO DISARM
def test_autodisarm_motors(mavproxy, mav):
    """Test Autodisarm motors."""
    progress("Test Autodisarming motors")
    mavproxy.send('switch 6\n')  # stabilize/manual mode
    if mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_TYPE_HELICOPTER,
                        mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                        mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                        mavutil.mavlink.MAV_TYPE_COAXIAL,
                        mavutil.mavlink.MAV_TYPE_TRICOPTER]:
        wait_mode(mav, 'STABILIZE')
        mavproxy.send('rc 3 1000\n')  # throttle at zero
    if mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
        wait_mode(mav, 'STABILIZE')
    if mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
        #  NOT IMPLEMENTED ON ROVER
        progress("MOTORS AUTODISARMED OK")
        return True
    arm_vehicle(mavproxy, mav)

    tstart = get_sim_time(mav)
    timeout = 15     #  TODO: adapt timeout with data from param
    while get_sim_time(mav) < tstart + timeout:
        if not mav.motors_armed():
            progress("MOTORS AUTODISARMED OK ")
            mavproxy.send('rc 1 1500\n')  # steer full right
            mavproxy.send('rc 4 1500\n')  # yaw full right
            return True
    progress("FAILED TO AUTODISARMED")
    return False

# TEST RC OVERRIDE
# TEST RC OVERRIDE TIMEOUT


def test_RTL(mavproxy, mav, home, timeout=250):
    """Return, land."""
    progress("# Enter RTL")
    mavproxy.send('switch 3\n')
    tstart = get_sim_time(mav)
    while get_sim_time(mav) < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        pos = mav.location()
        home_distance = get_distance(home, pos)
        progress("Alt: %u  HomeDistance: %.0f" % (m.alt, home_distance))
        if(m.alt <= 1 and home_distance < 10):
            progress("RTL Complete")
            return True
    return False


def test_mission(mavproxy, mav, filename):
    """Test a mission from a file."""
    progress("Test mission %s" % filename)
    num_wp = load_mission_from_file(mavproxy, filename)
    mavproxy.send('wp set 1\n')
    mavproxy.send('switch 4\n')  # auto mode
    wait_mode(mav, 'AUTO')
    ret = wait_waypoint(mav, 0, num_wp-1, max_dist=5, timeout=500)

    if mav.mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_TYPE_HELICOPTER,
                        mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                        mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                        mavutil.mavlink.MAV_TYPE_COAXIAL,
                        mavutil.mavlink.MAV_TYPE_TRICOPTER]:
        expect_msg = "Reached command #%u" % (num_wp-1)
        if ret:
            mavproxy.expect(expect_msg)
        mavproxy.send('switch 5\n')  # loiter mode
        wait_mode(mav, 'LOITER')
    if mav.mav_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
        print("NOT IMPLEMENTED")
    if mav.mav_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
        expect_msg = "No commands."
        if ret:
            mavproxy.expect(expect_msg)
        wait_mode(mav, 'HOLD')
    progress("test: MISSION COMPLETE: passed=%s" % ret)
    return ret

#TEST Guided :
 # goto
 # set point position
    #set point velocity
 # set attitude