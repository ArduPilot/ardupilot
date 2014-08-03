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
                 onoff=False,
                 yawrate=9.0,
                 pitchrate=1.0,
                 pitch_range = 45,
                 yaw_range = 170,
                 zero_yaw = 270, # yaw direction at startup
                 zero_pitch = 10 # pitch at startup
                 ):
        Aircraft.__init__(self)
        self.onoff = onoff
        self.yawrate = yawrate
        self.pitchrate = pitchrate
        self.last_time = time.time()
        self.pitch_range = pitch_range
        self.yaw_range = yaw_range
        self.zero_yaw = zero_yaw
        self.zero_pitch = zero_pitch
        self.verbose = False
        self.last_debug = time.time()
        self.pitch_current = 0
        self.yaw_current = 0

    def slew_limit(self, current, target, range, delta_time, turn_rate):
        '''limit speed of servo movement'''
        dangle = turn_rate * delta_time
        dv = dangle / range
        if target - current > dv:
            return current + dv
        if target - current < -dv:
            return current - dv
        return target


    def update_position_servos(self, state, delta_time):
        '''update function for position (normal) servos.
        Returns (yaw_rate,pitch_rate) tuple'''
        self.pitch_current = self.slew_limit(self.pitch_current, state.pitch_input, self.pitch_range, delta_time, self.yawrate)
        self.yaw_current = self.slew_limit(self.yaw_current, state.yaw_input, self.yaw_range, delta_time, self.pitchrate)

        pitch_target = self.zero_pitch + self.pitch_current*self.pitch_range
        yaw_target = self.zero_yaw + self.yaw_current*self.yaw_range
        while yaw_target > 180:
            yaw_target -= 360

        (r,p,y) = self.dcm.to_euler()
        pitch_current = degrees(p)
        yaw_current = degrees(y)
        roll_current = degrees(r)

        pitch_rate = pitch_target - pitch_current
        pitch_rate = min(self.pitchrate, pitch_rate)
        pitch_rate = max(-self.pitchrate, pitch_rate)

        yaw_diff = yaw_target - yaw_current
        if yaw_diff > 180:
            yaw_diff -= 360
        if yaw_diff < -180:
            yaw_diff += 360            
        yaw_rate = yaw_diff
        yaw_rate = min(self.yawrate, yaw_rate)
        yaw_rate = max(-self.yawrate, yaw_rate)

        return (yaw_rate, pitch_rate)

    def update_onoff_servos(self, state):
        '''update function for onoff servos.
        These servos either move at a constant rate or are still 
        Returns (yaw_rate,pitch_rate) tuple'''
        if abs(state.yaw_input) < 0.1:
            yaw_rate = 0
        elif state.yaw_input >= 0.1:
            yaw_rate = self.yawrate
        else:
            yaw_rate = -self.yawrate

        if abs(state.pitch_input) < 0.1:
            pitch_rate = 0
        elif state.pitch_input >= 0.1:
            pitch_rate = self.pitchrate
        else:
            pitch_rate = -self.pitchrate
        return (yaw_rate, pitch_rate)

    def update(self, state):
        # how much time has passed?
        t = time.time()
        delta_time = t - self.last_time
        self.last_time = t

        if self.onoff:
            (yaw_rate,pitch_rate) = self.update_onoff_servos(state)
        else:
            (yaw_rate,pitch_rate) = self.update_position_servos(state, delta_time)

        # implement yaw and pitch limits
        (r,p,y) = self.dcm.to_euler()
        pitch_current = degrees(p)
        yaw_current = degrees(y)
        roll_current = degrees(r)

        if yaw_rate > 0 and yaw_current >= self.yaw_range:
            yaw_rate = 0
        if yaw_rate < 0 and yaw_current <= -self.yaw_range:
            yaw_rate = 0

        if pitch_rate > 0 and pitch_current >= self.pitch_range:
            pitch_rate = 0
        if pitch_rate < 0 and pitch_current <= -self.pitch_range:
            pitch_rate = 0

        # keep it level
        roll_rate = 0 - roll_current

        if time.time() - self.last_debug > 2 and not self.onoff:
            self.last_debug = time.time()
            print("roll=%.1f/%.1f pitch=%.1f yaw=%.1f rates=%.1f/%.1f/%.1f" % (
                roll_current, 0,
                pitch_current, 
                yaw_current, 
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
