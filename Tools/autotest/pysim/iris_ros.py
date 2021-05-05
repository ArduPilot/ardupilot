#!/usr/bin/env python
"""
Python interface to euroc ROS multirotor simulator
See https://pixhawk.org/dev/ros/sitl
"""

import time

import mav_msgs.msg as mav_msgs
import px4.msg as px4
import rosgraph_msgs.msg as rosgraph_msgs
import rospy
import sensor_msgs.msg as sensor_msgs

from aircraft import Aircraft
from pymavlink.rotmat import Vector3, Matrix3


def quat_to_dcm(q1, q2, q3, q4):
    """Convert quaternion to DCM."""
    q3q3 = q3 * q3
    q3q4 = q3 * q4
    q2q2 = q2 * q2
    q2q3 = q2 * q3
    q2q4 = q2 * q4
    q1q2 = q1 * q2
    q1q3 = q1 * q3
    q1q4 = q1 * q4
    q4q4 = q4 * q4

    m = Matrix3()
    m.a.x = 1.0-2.0*(q3q3 + q4q4)
    m.a.y =   2.0*(q2q3 - q1q4)
    m.a.z =   2.0*(q2q4 + q1q3)
    m.b.x =   2.0*(q2q3 + q1q4)
    m.b.y =   1.0-2.0*(q2q2 + q4q4)
    m.b.z =   2.0*(q3q4 - q1q2)
    m.c.x =   2.0*(q2q4 - q1q3)
    m.c.y =   2.0*(q3q4 + q1q2)
    m.c.z = 1.0-2.0*(q2q2 + q3q3)
    return m


class IrisRos(Aircraft):
    """A IRIS MultiCopter from ROS."""
    def __init__(self):
        Aircraft.__init__(self)
        self.max_rpm = 1200
        self.have_new_time = False
        self.have_new_imu = False
        self.have_new_pos = False

        topics = {
            "/clock"                        : (self.clock_cb, rosgraph_msgs.Clock),
            "/iris/imu"                     : (self.imu_cb, sensor_msgs.Imu),
            "/iris/vehicle_local_position"  : (self.pos_cb, px4.vehicle_local_position),
            }

        rospy.init_node('ArduPilot', anonymous=True)
        for topic in topics.keys():
            (callback, msgtype) = topics[topic]
            rospy.Subscriber(topic, msgtype, callback)

        self.motor_pub = rospy.Publisher('/iris/command/motor_speed',
                                         mav_msgs.CommandMotorSpeed,
                                         queue_size=1)
        self.last_time = 0

        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()

    def clock_cb(self, msg):
        self.time_now = self.time_base + msg.clock.secs + msg.clock.nsecs*1.0e-9
        self.have_new_time = True

    def imu_cb(self, msg):
        self.gyro = Vector3(msg.angular_velocity.x,
                            -msg.angular_velocity.y,
                            -msg.angular_velocity.z)
        self.accel_body = Vector3(msg.linear_acceleration.x,
                                  -msg.linear_acceleration.y,
                                  -msg.linear_acceleration.z)
        self.dcm = quat_to_dcm(msg.orientation.w,
                               msg.orientation.x,
                               -msg.orientation.y,
                               -msg.orientation.z)
        self.have_new_imu = True

    def pos_cb(self, msg):
        self.velocity = Vector3(msg.vx, msg.vy, msg.vz)
        self.position = Vector3(msg.x, msg.y, msg.z)
        self.have_new_pos = True

    def update(self, actuators):
        while self.last_time == self.time_now or not self.have_new_time or not self.have_new_imu or not self.have_new_pos:
            time.sleep(0.001)
        self.have_new_time = False
        self.have_new_pos = False
        self.have_new_imu = False

        # create motor speed message
        msg = mav_msgs.CommandMotorSpeed()
        msg.header.stamp = rospy.get_rostime()
        motor_speed = []
        for i in range(len(actuators)):
            motor_speed.append(actuators[i]*self.max_rpm)
        msg.motor_speed = motor_speed

        self.last_time = self.time_now

        self.motor_pub.publish(msg)
        
        # update lat/lon/altitude
        self.update_position()
