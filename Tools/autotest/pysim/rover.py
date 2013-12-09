#!/usr/bin/env python
'''
simple rover simulator core
'''

from aircraft import Aircraft
import util, time, math
from math import degrees, radians, sin, cos, pi, asin
from rotmat import Vector3, Matrix3

class Rover(Aircraft):
    '''a simple rover'''
    def __init__(self,
                 max_speed=20,
                 max_accel=30,
                 wheelbase=0.335,
                 wheeltrack=0.296,
                 max_wheel_turn=35,
                 turning_circle=1.8,
                 skid_steering=False):
        Aircraft.__init__(self)
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.turning_circle = turning_circle
        self.wheelbase = wheelbase
        self.wheeltrack = wheeltrack
        self.max_wheel_turn = max_wheel_turn
        self.last_time = time.time()
        self.skid_steering = skid_steering

    def turn_circle(self, steering):
        '''return turning circle (diameter) in meters for steering angle proportion in degrees
        '''
        if abs(steering) < 1.0e-6:
            return 0
        return self.turning_circle * sin(radians(35)) / sin(radians(steering*35))

    def yaw_rate(self, steering, speed):
        '''return yaw rate in degrees/second given steering_angle and speed'''
        if abs(steering) < 1.0e-6 or abs(speed) < 1.0e-6:
            return 0
        d = self.turn_circle(steering)
        c = pi * d
        t = c / speed
        rate = 360.0 / t
        return rate

    def lat_accel(self, steering_angle, speed):
        '''return lateral acceleration in m/s/s'''
        yaw_rate = self.yaw_rate(steering_angle, speed)
        accel = radians(yaw_rate) * speed
        return accel

    def lat_accel2(self, steering_angle, speed):
        '''return lateral acceleration in m/s/s'''
        mincircle = self.wheelbase/sin(radians(35))
        steer = steering_angle/35
        return steer * (speed**2) * (2/mincircle)

    def steering_angle(self, lat_accel, speed):
        '''return steering angle to achieve the given lat_accel'''
        mincircle = self.wheelbase/sin(radians(35))
        steer = 0.5 * lat_accel * mincircle / (speed**2)
        return steer * 35

    def update(self, state):

        # if in skid steering mode the steering and throttle values are used for motor1 and motor2
        if self.skid_steering:
            motor1 = state.steering # left motor
            motor2 = state.throttle # right motor
            steering = motor1 - motor2
            throttle = 0.5*(motor1 + motor2)
        else:
            steering = state.steering
            throttle = state.throttle

        # how much time has passed?
        t = time.time()
        delta_time = t - self.last_time
        self.last_time = t

        # speed in m/s in body frame
        velocity_body = self.dcm.transposed() * self.velocity

        # speed along x axis, +ve is forward
        speed = velocity_body.x

        # yaw rate in degrees/s
        yaw_rate = self.yaw_rate(steering, speed)

        # target speed with current throttle
        target_speed = throttle * self.max_speed

        # linear acceleration in m/s/s - very crude model
        accel = self.max_accel * (target_speed - speed) / self.max_speed

#        print('speed=%f throttle=%f steering=%f yaw_rate=%f accel=%f' % (speed, state.throttle, state.steering, yaw_rate, accel))
        
        self.gyro = Vector3(0,0,radians(yaw_rate))

        # update attitude
        self.dcm.rotate(self.gyro * delta_time)
        self.dcm.normalize()

        # accel in body frame due to motor
        accel_body = Vector3(accel, 0, 0)

        # add in accel due to direction change
        accel_body.y += radians(yaw_rate) * speed

        # now in earth frame
        accel_earth = self.dcm * accel_body
        accel_earth += Vector3(0, 0, self.gravity)

        # if we're on the ground, then our vertical acceleration is limited
        # to zero. This effectively adds the force of the ground on the aircraft
        accel_earth.z = 0

        # work out acceleration as seen by the accelerometers. It sees the kinematic
        # acceleration (ie. real movement), plus gravity
        self.accel_body = self.dcm.transposed() * (accel_earth + Vector3(0, 0, -self.gravity))

        # new velocity vector
        self.velocity += accel_earth * delta_time

        # new position vector
        old_position = self.position.copy()
        self.position += self.velocity * delta_time

        # update lat/lon/altitude
        self.update_position(delta_time)

if __name__ == "__main__":
    r = Rover()
    d1 = r.turn_circle(r.max_wheel_turn)
    print("turn_circle=", d1)
    steer = 0.4*35
    speed = 2.65

    yrate = r.yaw_rate(steer, speed)
    yaccel = r.lat_accel(steer, speed)
    yaccel2 = r.lat_accel2(steer, speed)
    print yaccel, yaccel2
    sangle = r.steering_angle(yaccel, speed)
    print steer, sangle

    yrate2 = degrees(yaccel / speed)
    t = 360.0 / yrate2
    c = speed * t
    d2 = c / pi
    steer2 = degrees(asin(r.wheelbase / (d2 - (r.wheeltrack/2))))
    
    print steer, steer2
