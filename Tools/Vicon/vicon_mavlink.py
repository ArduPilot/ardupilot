#!/usr/bin/env python3

# flake8: noqa

'''
connect to a vicon and send mavlink vision data on UDP
Released under GNU GPL v3 or later
'''

from pyvicon import pyvicon
from pymavlink.quaternion import Quaternion

# force mavlink2 for yaw in GPS_INPUT
import os
os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil
from pymavlink import mavextra
import math
import time
import sys

from argparse import ArgumentParser
parser = ArgumentParser(description="vicon to mavlink gateway")

parser.add_argument("--vicon-host", type=str,
                    help="vicon host name or IP", default="vicon")
parser.add_argument("--mavlink-dest", type=str, help="mavlink destination", default="udpout:127.0.0.1:14550")
parser.add_argument("--origin", type=str, help="origin in lat,lon,alt,yaw", default="-35.363261,149.165230,584,270")
parser.add_argument("--debug", type=int, default=0, help="debug level")
parser.add_argument("--target-sysid", type=int, default=1, help="target mavlink sysid")
parser.add_argument("--source-sysid", type=int, default=255, help="mavlink source sysid")
parser.add_argument("--source-component", type=int, default=mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, help="mavlink source component")
parser.add_argument("--rate", type=float, default=None, help="message rate for GLOBAL_VISION_POSITION_ESTIMATE")
parser.add_argument("--gps-rate", type=int, default=5, help="message rate for GPS_INPUT")
parser.add_argument("--gps-nsats", type=int, default=16, help="GPS satellite count")
parser.add_argument("--gps-only", action='store_true', default=False, help="only send GPS data")
parser.add_argument("--send-zero", action='store_true', default=False, help="send zero data")

args = parser.parse_args()

# create a mavlink serial instance
mav = mavutil.mavlink_connection(args.mavlink_dest,
                                 source_system=args.source_sysid,
                                 source_component=args.source_component)

# create vicon connection
vicon = pyvicon.PyVicon()

origin_parts = args.origin.split(',')
if len(origin_parts) != 4:
    print("Origin should be lat,lon,alt,yaw")
    sys.exit(1)
origin = (float(origin_parts[0]), float(origin_parts[1]), float(origin_parts[2]))

last_origin_send = 0

def connect_to_vicon(ip):
    '''connect to a vicon with given ip or hostname'''
    global vicon
    print("Opening connection to %s" % ip)
    vicon.connect(ip)
    print("Configuring vicon")
    vicon.set_stream_mode(pyvicon.StreamMode.ClientPull)
    vicon.enable_marker_data()
    vicon.enable_segment_data()
    vicon.enable_unlabeled_marker_data()
    vicon.enable_device_data()
    # wait for first subject to appear
    print("waiting for vehicle...")
    while True:
        vicon.get_frame()
        name = vicon.get_subject_name(0)
        if name is not None:
            break
    print("Connected to subject %s" % name)

def get_gps_time(tnow):
    '''return gps_week and gps_week_ms for current time'''
    leapseconds = 18
    SEC_PER_WEEK = 7 * 86400
    MSEC_PER_WEEK = SEC_PER_WEEK * 1000
    
    epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - leapseconds
    epoch_seconds = int(tnow - epoch)
    week = int(epoch_seconds) // SEC_PER_WEEK
    t_ms = int(tnow * 1000) % 1000
    week_ms = (epoch_seconds % SEC_PER_WEEK) * 1000 + ((t_ms//200) * 200)
    return week, week_ms


def main_loop():
    '''loop getting vicon data'''
    global vicon
    name = vicon.get_subject_name(0)
    segname = vicon.get_subject_root_segment_name(name)
    last_msg_time = time.time()
    last_gps_pos = None
    now = time.time()
    last_origin_send = now
    now_ms = int(now * 1000)
    last_gps_send_ms = now_ms
    last_gps_send = now
    gps_period_ms = 1000 // args.gps_rate

    while True:
        if args.send_zero:
            time.sleep(0.05)
        else:
            vicon.get_frame()

        now = time.time()
        now_ms = int(now * 1000)

        if args.rate is not None:
            dt = now - last_msg_time
            if dt < 1.0 / args.rate:
                continue

        last_msg_time = now

        # get position as ENU mm
        if args.send_zero:
            pos_enu = [0.0, 0.0, 0.0]
        else:
            pos_enu = vicon.get_segment_global_translation(name, segname)
        if pos_enu is None:
            continue

        # convert to NED meters
        pos_ned = [pos_enu[0]*0.001, pos_enu[1]*0.001, -pos_enu[2]*0.001]

        # get orientation in euler, converting to ArduPilot conventions
        if args.send_zero:
            quat = [0.0, 0.0, 0.0, 1.0]
        else:
            quat = vicon.get_segment_global_quaternion(name, segname)
        q = Quaternion([quat[3], quat[0], quat[1], quat[2]])
        euler = q.euler
        roll, pitch, yaw = euler[2], euler[1], euler[0]
        yaw -= math.pi*0.5

        yaw_cd = int(mavextra.wrap_360(math.degrees(yaw)) * 100)
        if yaw_cd == 0:
            # the yaw extension to GPS_INPUT uses 0 as no yaw support
            yaw_cd = 36000

        if args.debug > 0:
            print("Pose: [%.3f, %.3f, %.3f] [%.2f %.2f %.2f]" % (
                  pos_ned[0], pos_ned[1], pos_ned[2],
                math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))

        time_us = int(now * 1.0e6)

        if now - last_origin_send > 1 and not args.gps_only:
            # send a heartbeat msg
            mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)

            # send origin at 1Hz
            mav.mav.set_gps_global_origin_send(args.target_sysid,
                                               int(origin[0]*1.0e7), int(origin[1]*1.0e7), int(origin[2]*1.0e3),
                                               time_us)
            last_origin_send = now

        if args.gps_rate > 0 and now_ms - last_gps_send_ms > gps_period_ms:
            '''send GPS data at the specified rate, trying to align on the given period'''
            if last_gps_pos is None:
                last_gps_pos = pos_ned
            gps_lat, gps_lon = mavextra.gps_offset(origin[0], origin[1], pos_ned[1], pos_ned[0])
            gps_alt = origin[2] - pos_ned[2]

            gps_dt = now - last_gps_send
            gps_vel = [ (pos_ned[0]-last_gps_pos[0])/gps_dt,
                        (pos_ned[1]-last_gps_pos[1])/gps_dt,
                        (pos_ned[2]-last_gps_pos[2])/gps_dt ]
            last_gps_pos = pos_ned

            gps_week, gps_week_ms = get_gps_time(now)

            if args.gps_nsats >= 6:
                fix_type = 3
            else:
                fix_type = 1
            mav.mav.gps_input_send(time_us, 0, 0, gps_week_ms, gps_week, fix_type,
                                   int(gps_lat*1.0e7), int(gps_lon*1.0e7), gps_alt,
                                   1.0, 1.0,
                                   gps_vel[0], gps_vel[1], gps_vel[2],
                                   0.2, 1.0, 1.0,
                                   args.gps_nsats,
                                   yaw_cd)
            last_gps_send_ms = (now_ms//gps_period_ms) * gps_period_ms
            last_gps_send = now


        # send VISION_POSITION_ESTIMATE
        if not args.gps_only:
            # we force mavlink1 to avoid the covariances which seem to make the packets too large
            # for the mavesp8266 wifi bridge
            mav.mav.global_vision_position_estimate_send(time_us,
                                                         pos_ned[0], pos_ned[1], pos_ned[2],
                                                         roll, pitch, yaw, force_mavlink1=True)


connect_to_vicon("vicon")
main_loop()

