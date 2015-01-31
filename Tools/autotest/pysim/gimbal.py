#!/usr/bin/env python

from aircraft import Aircraft
import util, time, math
from math import degrees, radians, cos, sin, tan
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

        # physical constraints on joint angles in (roll, pitch, azimuth) order
        self.lower_joint_limits = Vector3(radians(-40), radians(-135), radians(-7.5))
        self.upper_joint_limits = Vector3(radians(40),  radians(45),   radians(7.5))
        self.travelLimitGain = 20

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
        self.demanded_angular_rate = Vector3(radians(0), radians(0), radians(0))

        # gyro bias provided by EKF on vehicle. In rad/s.
        # Should be subtracted from the gyro readings to get true body rotatation rates
        self.supplied_gyro_bias = Vector3()

        ###################################
        # communication variables
        self.connection = None
        self.last_report_t = time.time()
        self.last_heartbeat_t = time.time()
        self.seen_heartbeat = False
        self.seen_gimbal_control = False
        self.last_print_t = time.time()
        self.vehicle_component_id = None
        self.last_report_t = time.time()

    def update(self):
        '''update the gimbal state'''

        # calculate delta time in seconds
        now = time.time()
        delta_t = now - self.last_update_t
        self.last_update_t = now

        if self.dcm is None:
            # start with copter rotation matrix
            self.dcm = self.vehicle.dcm.copy()

        # take a copy of the demanded rates to bypass the limiter function for testing
        demRateRaw = self.demanded_angular_rate

        # 1)  Rotate the copters rotation rates into the gimbals frame of reference
        # copterAngRate_G = transpose(DCMgimbal)*DCMcopter*copterAngRate
        copterAngRate_G = self.dcm.transposed()*self.vehicle.dcm*self.vehicle.gyro
        #print("copterAngRate_G ", copterAngRate_G)

        # 2) Subtract the copters body rates to obtain a copter relative rotational
        # rate vector (X,Y,Z) in gimbal sensor frame
        # relativeGimbalRate(X,Y,Z) = gimbalRateDemand - copterAngRate_G
        relativeGimbalRate = self.demanded_angular_rate - copterAngRate_G
        #print("relativeGimbalRate ", relativeGimbalRate)

        # calculate joint angles (euler312 order)
        # calculate copter -> gimbal rotation matrix
        rotmat_copter_gimbal = self.dcm.transposed() * self.vehicle.dcm
        self.joint_angles = Vector3(*rotmat_copter_gimbal.transposed().to_euler312())
        #print("joint_angles ", self.joint_angles)
        
        # 4)  For each of the three joints, calculate upper and lower rate limits
        # from the corresponding angle limits and current joint angles
        #
        # upperRatelimit = (jointAngle - lowerAngleLimit) * travelLimitGain
        # lowerRatelimit = (jointAngle - upperAngleLimit) * travelLimitGain
        #
        # travelLimitGain is equal to the inverse of the bump stop time constant and
        # should be set to something like 20 initially. If set too high it can cause
        # the rates to 'ring' when they the limiter is in force, particularly given
        # we are using a first order numerical integration.
        upperRatelimit = -(self.joint_angles - self.upper_joint_limits) * self.travelLimitGain
        lowerRatelimit = -(self.joint_angles - self.lower_joint_limits) * self.travelLimitGain

        # 5) Calculate the gimbal joint rates (roll, elevation, azimuth)
        # 
        # gimbalJointRates(roll, elev, azimuth) = Matrix*relativeGimbalRate(X,Y,Z)
        #
        # where matrix =
        #
        # +-                                                                  -+
        # |          cos(elevAngle),        0,         sin(elevAngle)          |
        # |                                                                    |
        # |  sin(elevAngle) tan(rollAngle), 1, -cos(elevAngle) tan(rollAngle)  |
        # |                                                                    |
        # |           sin(elevAngle)                   cos(elevAngle)          |
        # |         - --------------,       0,         --------------          |
        # |           cos(rollAngle)                   cos(rollAngle)          |
        # +-                                                                  -+
        rollAngle = self.joint_angles.x
        elevAngle = self.joint_angles.y
        matrix = Matrix3(Vector3(cos(elevAngle), 0, sin(elevAngle)),
                         Vector3(sin(elevAngle)*tan(rollAngle), 1, -cos(elevAngle)*tan(rollAngle)),
                         Vector3(-sin(elevAngle)/cos(rollAngle), 0, cos(elevAngle)/cos(rollAngle)))
        gimbalJointRates = matrix * relativeGimbalRate
        #print("lowerRatelimit ", lowerRatelimit)
        #print("upperRatelimit ", upperRatelimit)
        #print("gimbalJointRates ", gimbalJointRates)

        # 6) Apply the rate limits from 4)
        gimbalJointRates.x = constrain(gimbalJointRates.x, lowerRatelimit.x, upperRatelimit.x)
        gimbalJointRates.y = constrain(gimbalJointRates.y, lowerRatelimit.y, upperRatelimit.y)
        gimbalJointRates.z = constrain(gimbalJointRates.z, lowerRatelimit.z, upperRatelimit.z)

        # 7) Convert the modified gimbal joint rates to body rates (still copter
        # relative)
        # relativeGimbalRate(X,Y,Z) = Matrix * gimbalJointRates(roll, elev, azimuth)
        #
        # where Matrix =
        #
        #   +-                                                   -+
        #   |  cos(elevAngle), 0, -cos(rollAngle) sin(elevAngle)  |
        #   |                                                     |
        #   |         0,       1,         sin(rollAngle)          |
        #   |                                                     |
        #   |  sin(elevAngle), 0,  cos(elevAngle) cos(rollAngle)  |
        #   +-                                                   -+
        matrix = Matrix3(Vector3(cos(elevAngle), 0, -cos(rollAngle)*sin(elevAngle)),
                         Vector3(0,       1,         sin(rollAngle)),
                         Vector3(sin(elevAngle), 0,  cos(elevAngle)*cos(rollAngle)))
        relativeGimbalRate = matrix * gimbalJointRates

        # 8) Add to the result from step 1) to obtain the demanded gimbal body rates
        # in an inertial frame of reference
        # demandedGimbalRatesInertial(X,Y,Z)  = relativeGimbalRate(X,Y,Z) + copterAngRate_G
        demandedGimbalRatesInertial = relativeGimbalRate + copterAngRate_G
        #if now - self.last_print_t >= 1.0:
        #    print("demandedGimbalRatesInertial ", demandedGimbalRatesInertial)
            
        # for the moment we will set gyros equal to demanded_angular_rate
        self.gimbal_angular_rate = demRateRaw #demandedGimbalRatesInertial + self.true_gyro_bias - self.supplied_gyro_bias

        # update rotation of the gimbal
        self.dcm.rotate(self.gimbal_angular_rate*delta_t)
        self.dcm.normalize()

        # calculate copter -> gimbal rotation matrix
        rotmat_copter_gimbal = self.dcm.transposed() * self.vehicle.dcm

        # calculate joint angles (euler312 order)
        self.joint_angles = Vector3(*rotmat_copter_gimbal.transposed().to_euler312())

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

        if now - self.last_print_t >= 1.0:
            self.last_print_t = now
            # calculate joint angles (euler312 order)
            Euler312 = Vector3(*self.dcm.to_euler312())
            print("Euler Angles 312 %6.1f %6.1f %6.1f" % (degrees(Euler312.z), degrees(Euler312.x), degrees(Euler312.y)))

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
            if m.get_type() == 'HEARTBEAT' and not self.seen_heartbeat:
                self.seen_heartbeat = True
                self.vehicle_component_id = m.get_srcComponent()
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
            delta_time = now - self.last_report_t
            self.last_report_t = now
            self.connection.mav.gimbal_report_send(self.connection.mav.srcSystem,
                                                   self.vehicle_component_id,
                                                   delta_time,
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
