#!/usr/bin/env python

from aircraft import Aircraft
import util, time, math
from math import degrees, radians
from rotmat import Vector3, Matrix3

def constrain(value, minv, maxv):
    '''constrain a value to a range'''
    if value < minv:
        value = minv
    if value > maxv:
        value = maxv
    return value

class Gimbal3Axis(object):
    '''a gimbal simulation'''
    def __init__(self, vehicle):
        # vehicle is the vehicle (usually a multicopter) that the gimbal is attached to
        self.vehicle = vehicle

        # URL of SITL 2nd telem port
        self.mavlink_url = 'tcp:127.0.0.1:5762'

        # rotation matrix (gimbal body -> earth)
        self.dcm = None

        # time of last update
        self.last_update_t = time.time()

        # true angular rate of gimbal in body frame (rad/s)
        self.gimbal_angular_rate = Vector3()

        # observed angular rate (including biases)
        self.gyro = Vector3()

        # joint angles, in radians. in yaw/roll/pitch order. Relative to fwd.
        # So 0,0,0 points forward.
        # Pi/2,0,0 means pointing right
        # 0, Pi/2, 0 means pointing fwd, but rolled 90 degrees to right
        # 0, 0, -Pi/2, means pointing down
        self.joint_angles = Vector3()

        # physical constraints on joint angles
        self.min_yaw = radians(-7.5)
        self.max_yaw = radians(7.5)
        self.min_roll = radians(-40)
        self.max_roll = radians(40)
        self.min_pitch = radians(-135)
        self.max_pitch = radians(45)

        # true gyro bias
        self.true_gyro_bias = Vector3()

        ##################################
        # reporting variables. gimbal pushes these to vehicle code over MAVLink at approx 100Hz

        # reporting rate in Hz
        self.reporting_rate = 100
        
        # integral of gyro vector over last time interval. In radians
        self.delta_angle = Vector3()

        # integral of accel vector over last time interval. In m/s
        self.delta_velocity = Vector3()
        # we also push joint_angles

        ##################################
        # control variables from the vehicle

        # angular rate in rad/s. In body frame of gimbal
        self.demanded_angular_rate = Vector3()

        # gyro bias provided by EKF on vehicle. In rad/s.
        # Should be subtracted from the gyro readings to get true body rotatation rates
        self.supplied_gyro_bias = Vector3()

        ###################################
        # communication variables
        self.connection = None
        self.counter = 0
        self.last_report_t = time.time()
        self.last_heartbeat_t = time.time()
        self.seen_heartbeat = False
        self.seen_gimbal_control = False

    def update(self):
        '''update the gimbal state'''

        # calculate delta time in seconds
        now = time.time()
        delta_t = now - self.last_update_t
        self.last_update_t = now

        # for the moment we will set gyros equal to demanded_angular_rate
        self.gimbal_angular_rate = self.demanded_angular_rate + self.true_gyro_bias - self.supplied_gyro_bias

        if self.dcm is None:
            # start with copter rotation matrix
            self.dcm = self.vehicle.dcm.copy()

        # update rotation of the gimbal
        self.dcm.rotate(self.gimbal_angular_rate*delta_t)
        self.dcm.normalize()

        # calculate copter -> gimbal rotation matrix
        rotmat_copter_gimbal = self.dcm.transposed() * self.vehicle.dcm

        # calculate joint angles (euler312 order)
        self.joint_angles = Vector3(*rotmat_copter_gimbal.to_euler312())

        # constrain joint angles
        need_new_dcm = False

        # constrain observed gyro (prevent observed rotation past gimbal limits)
        # NOTE: this needs to be replaced later with code that first converts rates
        # from 312 rates to body rates and back again
        if self.joint_angles.x >= self.max_yaw:
            need_new_dcm = True
            self.joint_angles.x = self.max_yaw
            if self.gimbal_angular_rate.z > 0:
                self.gimbal_angular_rate.z = 0
        if self.joint_angles.x <= self.min_yaw:
            need_new_dcm = True
            self.joint_angles.x = self.min_yaw
            if self.gimbal_angular_rate.z < 0:
                self.gimbal_angular_rate.z = 0
        if self.joint_angles.y >= self.max_roll:
            need_new_dcm = True
            self.joint_angles.y = self.max_roll
            if self.gimbal_angular_rate.x > 0:
                self.gimbal_angular_rate.x = 0
        if self.joint_angles.y <= self.min_roll:
            need_new_dcm = True
            self.joint_angles.y = self.min_roll
            if self.gimbal_angular_rate.x < 0:
                self.gimbal_angular_rate.x = 0
        if self.joint_angles.z >= self.max_pitch:
            need_new_dcm = True
            self.joint_angles.z = self.max_pitch
            if self.gimbal_angular_rate.y > 0:
                self.gimbal_angular_rate.y = 0
        if self.joint_angles.z <= self.min_pitch:
            need_new_dcm = True
            self.joint_angles.z = self.min_pitch
            if self.gimbal_angular_rate.y < 0:
                self.gimbal_angular_rate.y = 0

        if need_new_dcm:
            # when we hit the gimbal limits we need to recalc the matrix
            rotmat_copter_gimbal.from_euler312(self.joint_angles.x, self.joint_angles.y, self.joint_angles.z)
            self.dcm = self.vehicle.dcm * rotmat_copter_gimbal.transposed()

        # update observed gyro
        self.gyro = self.gimbal_angular_rate + self.true_gyro_bias

        # update delta_angle (integrate)
        self.delta_angle += self.gyro * delta_t

        # calculate accel in gimbal body frame
        copter_accel_earth = self.vehicle.dcm * self.vehicle.accel_body
        accel = self.dcm.transposed() * copter_accel_earth

        # integrate velocity
        self.delta_velocity += accel * delta_t

        # see if we should do a report
        if now - self.last_report_t >= 1.0 / self.reporting_rate:
            self.send_report()
            self.last_report_t = now

    def send_report(self):
        '''send a report to the vehicle control code over MAVLink'''
        from pymavlink import mavutil
        if self.connection is None:
            try:
                self.connection = mavutil.mavlink_connection(self.mavlink_url, retries=0)
            except Exception as e:
                return
            print("Gimbal connected to %s" % self.mavlink_url)
            
        # check for incoming control messages
        while True:
            m = self.connection.recv_match(type=['GIMBAL_CONTROL','HEARTBEAT'], blocking=False)
            if m is None:
                break
            if m.get_type() == 'GIMBAL_CONTROL':
                self.demanded_angular_rate = Vector3(m.demanded_rate_x,
                                                     m.demanded_rate_y,
                                                     m.demanded_rate_z)
                self.supplied_gyro_bias = Vector3(m.gyro_bias_x,
                                                  m.gyro_bias_y,
                                                  m.gyro_bias_z)
                self.seen_gimbal_control = True
                print("rate=%s" % self.demanded_angular_rate)
            if m.get_type() == 'HEARTBEAT' and not self.seen_heartbeat:
                self.seen_heartbeat = True
                self.connection.mav.srcSystem = m.get_srcSystem()
                self.connection.mav.srcComponent = mavutil.mavlink.MAV_COMP_ID_GIMBAL
                print("Gimbal using srcSystem %u" % self.connection.mav.srcSystem)

        if self.seen_heartbeat:
            now = time.time()
            if now - self.last_heartbeat_t >= 1:
                self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GIMBAL,
                                                   mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                                   0, 0, 0)
                self.last_heartbeat_t = now

        if self.seen_heartbeat:
            self.connection.mav.gimbal_report_send(self.counter,
                                                   self.delta_angle.x,
                                                   self.delta_angle.y,
                                                   self.delta_angle.z,
                                                   self.delta_velocity.x,
                                                   self.delta_velocity.y,
                                                   self.delta_velocity.z,
                                                   self.joint_angles.x,
                                                   self.joint_angles.y,
                                                   self.joint_angles.z)
            self.delta_velocity.zero()
            self.delta_angle.zero()
            self.counter += 1
