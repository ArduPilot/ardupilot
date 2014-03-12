#!/usr/bin/env python
'''
simple antenna tracker simulator core
'''

from aircraft import Aircraft
import util, time, math
from math import degrees, radians
from rotmat import Vector3

class Tracker(Aircraft):
    '''a simple antenna tracker'''
    def __init__(self,
                 rate_controlled=False,
                 pitch_range = 45,
                 yaw_range = 180,
                 zero_yaw = 270, # yaw direction at startup
                 zero_pitch = 10, # pitch at startup
                 turn_rate=90 # servo max turn rate in degrees/sec
                 ):
        Aircraft.__init__(self)
        self.rate_controlled = rate_controlled
        self.turn_rate = turn_rate
        self.last_time = time.time()
        self.pitch_range = pitch_range
        self.yaw_range = yaw_range
        self.zero_yaw = zero_yaw
        self.zero_pitch = zero_pitch
        self.verbose = False
        self.last_debug = time.time()
        self.pitch_current = 0
        self.yaw_current = 0

    def slew_limit(self, current, target, range, delta_time):
        '''limit speed of servo movement'''
        dangle = self.turn_rate * delta_time
        dv = dangle / range
        if target - current > dv:
            return current + dv
        if target - current < -dv:
            return current - dv
        return target
        

    def update(self, state):
        # how much time has passed?
        t = time.time()
        delta_time = t - self.last_time
        self.last_time = t

        self.pitch_current = self.slew_limit(self.pitch_current, state.pitch_input, self.pitch_range, delta_time)
        self.yaw_current = self.slew_limit(self.yaw_current, state.yaw_input, self.yaw_range, delta_time)

        pitch_target = self.zero_pitch + self.pitch_current*self.pitch_range
        yaw_target = self.zero_yaw + self.yaw_current*self.yaw_range
        while yaw_target > 180:
            yaw_target -= 360

        (r,p,y) = self.dcm.to_euler()
        pitch_current = degrees(p)
        yaw_current = degrees(y)
        roll_current = degrees(r)

        pitch_rate = pitch_target - pitch_current
        pitch_rate = min(self.turn_rate, pitch_rate)
        pitch_rate = max(-self.turn_rate, pitch_rate)

        yaw_diff = yaw_target - yaw_current
        if yaw_diff > 180:
            yaw_diff -= 360
        if yaw_diff < -180:
            yaw_diff += 360            
        yaw_rate = yaw_diff
        yaw_rate = min(self.turn_rate, yaw_rate)
        yaw_rate = max(-self.turn_rate, yaw_rate)

        # keep it level
        roll_rate = 0 - roll_current

        if time.time() - self.last_debug > 2:
            self.last_debug = time.time()
            print("roll=%.1f/%.1f pitch=%.1f/%.1f yaw=%.1f/%.1f rates=%.1f/%.1f/%.1f" % (
                roll_current, 0,
                pitch_current, pitch_target,
                yaw_current, yaw_target,
                roll_rate, pitch_rate, yaw_rate))

        self.gyro = Vector3(radians(roll_rate),radians(pitch_rate),radians(yaw_rate))

        # update attitude
        self.dcm.rotate(self.gyro * delta_time)
        self.dcm.normalize()

        accel_earth = Vector3(0, 0, -self.gravity)
        self.accel_body = self.dcm.transposed() * accel_earth

        # new velocity vector
        self.velocity = Vector3()
        self.update_position(delta_time)
