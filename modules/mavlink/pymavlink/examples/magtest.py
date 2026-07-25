#!/usr/bin/env python3

'''
rotate APMs on bench to test magnetometers

'''
import time
from math import radians

from pymavlink import mavutil

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--device1", required=True, help="mavlink device1")
parser.add_argument("--device2", required=True, help="mavlink device2")
parser.add_argument("--baudrate", type=int,
                  help="master port baud rate", default=115200)
args = parser.parse_args()

def set_attitude(rc3, rc4):
    values = [ 65535 ] * 8
    values[2] = rc3
    values[3] = rc4
    mav1.mav.rc_channels_override_send(mav1.target_system, mav1.target_component, *values)
    mav2.mav.rc_channels_override_send(mav2.target_system, mav2.target_component, *values)


# create a mavlink instance
mav1 = mavutil.mavlink_connection(args.device1, baud=args.baudrate)

# create a mavlink instance
mav2 = mavutil.mavlink_connection(args.device2, baud=args.baudrate)

print("Waiting for HEARTBEAT")
mav1.wait_heartbeat()
mav2.wait_heartbeat()
print("Heartbeat from APM (system %u component %u)" % (mav1.target_system, mav1.target_component))
print("Heartbeat from APM (system %u component %u)" % (mav2.target_system, mav2.target_component))

print("Waiting for MANUAL mode")
mav1.recv_match(type='SYS_STATUS', condition='SYS_STATUS.mode==2 and SYS_STATUS.nav_mode==4', blocking=True)
mav2.recv_match(type='SYS_STATUS', condition='SYS_STATUS.mode==2 and SYS_STATUS.nav_mode==4', blocking=True)

print("Setting declination")
mav1.mav.param_set_send(mav1.target_system, mav1.target_component,
                       'COMPASS_DEC', radians(12.33))
mav2.mav.param_set_send(mav2.target_system, mav2.target_component,
                       'COMPASS_DEC', radians(12.33))


set_attitude(1060, 1160)

event = mavutil.periodic_event(30)
pevent = mavutil.periodic_event(0.3)
rc3_min = 1060
rc3_max = 1850
rc4_min = 1080
rc4_max = 1500
rc3 = rc3_min
rc4 = 1160
delta3 = 2
delta4 = 1
use_pitch = 1

MAV_ACTION_CALIBRATE_GYRO = 17
mav1.mav.action_send(mav1.target_system, mav1.target_component, MAV_ACTION_CALIBRATE_GYRO)
mav2.mav.action_send(mav2.target_system, mav2.target_component, MAV_ACTION_CALIBRATE_GYRO)

print("Waiting for gyro calibration")
mav1.recv_match(type='ACTION_ACK')
mav2.recv_match(type='ACTION_ACK')

print("Resetting mag offsets")
mav1.mav.set_mag_offsets_send(mav1.target_system, mav1.target_component, 0, 0, 0)
mav2.mav.set_mag_offsets_send(mav2.target_system, mav2.target_component, 0, 0, 0)

def TrueHeading(SERVO_OUTPUT_RAW):
    p = float(SERVO_OUTPUT_RAW.servo3_raw - rc3_min) / (rc3_max - rc3_min)
    return 172 + p*(326 - 172)

while True:
    mav1.recv_msg()
    mav2.recv_msg()
    if event.trigger():
        if not use_pitch:
            rc4 = 1160
        set_attitude(rc3, rc4)
        rc3 += delta3
        if rc3 > rc3_max or rc3 < rc3_min:
            delta3 = -delta3
            use_pitch ^= 1
        rc4 += delta4
        if rc4 > rc4_max or rc4 < rc4_min:
            delta4 = -delta4
    if pevent.trigger():
        print("hdg1: %3u hdg2: %3u  ofs1: %4u, %4u, %4u  ofs2: %4u, %4u, %4u" % (
            mav1.messages['VFR_HUD'].heading,
            mav2.messages['VFR_HUD'].heading,
            mav1.messages['SENSOR_OFFSETS'].mag_ofs_x,
            mav1.messages['SENSOR_OFFSETS'].mag_ofs_y,
            mav1.messages['SENSOR_OFFSETS'].mag_ofs_z,
            mav2.messages['SENSOR_OFFSETS'].mag_ofs_x,
            mav2.messages['SENSOR_OFFSETS'].mag_ofs_y,
            mav2.messages['SENSOR_OFFSETS'].mag_ofs_z,
            ))
    time.sleep(0.01)

# 314M 326G
# 160M 172G

