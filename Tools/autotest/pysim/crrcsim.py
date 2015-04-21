#!/usr/bin/env python
'''
Python interface to CRRCSIM simulator
'''

from aircraft import Aircraft
import util, time, struct
import socket, select
from math import degrees, radians, sin, cos
from rotmat import Vector3, Matrix3

class CRRCSim(Aircraft):
    '''a CRRCCIM shim'''
    def __init__(self, frame="generic"):
        Aircraft.__init__(self)

        self.sim = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sim.connect(('127.0.0.1', 9002))
        self.sim.setblocking(0)
        self.accel_body = Vector3(0, 0, -self.gravity)
        self.frame = frame
        self.last_timestamp = 0
        if self.frame.find("heli") != -1:
            self.decode_servos = self.decode_servos_heli
        else:
            self.decode_servos = self.decode_servos_fixed_wing

    def decode_servos_heli(self, servos):
        '''decode servos for heli'''
        swash1 = servos[0]
        swash2 = servos[1]
        swash3 = servos[2]
        tail_rotor = servos[3]
        rsc = servos[7]

        col_pitch = (swash1+swash2+swash3)/3.0 - 0.5
        roll_rate = (swash1 - swash2)/2
        pitch_rate = -((swash1 + swash2)/2.0 - swash3)/2
        yaw_rate = -(tail_rotor - 0.5)
        rsc_speed = rsc

        buf = struct.pack('<fffff',
                          util.constrain(roll_rate, -0.5, 0.5),
                          util.constrain(pitch_rate, -0.5, 0.5),
                          util.constrain(rsc, 0, 1),
                          util.constrain(yaw_rate, -0.5, 0.5),
                          util.constrain(col_pitch, -0.5, 0.5))
        self.sim.send(buf)

    def decode_servos_fixed_wing(self, servos):
        '''decode servos for fixed wing'''
        roll_rate = util.constrain(servos[0]-0.5, -0.5, 0.5)
        pitch_rate = util.constrain(servos[1]-0.5, -0.5, 0.5)
        throttle = util.constrain(servos[2], 0, 1)
        yaw_rate = util.constrain(servos[3]-0.5, -0.5, 0.5)

        buf = struct.pack('<fffff',
                          roll_rate,
                          pitch_rate,
                          throttle,
                          yaw_rate,
                          0)
        self.sim.send(buf)
        
    def recv_fdm(self):
        '''receive FDM packet'''
        try:
            select.select([self.sim.fileno()], [], [], 1)
            buf = self.sim.recv(1024)
        except Exception as ex:
            return False

        (timestamp,
         latitude, longitude,
         altitude,
         heading,
         speedN, speedE, speedD,
         xAccel, yAccel, zAccel,
         rollRate, pitchRate, yawRate,
         roll, pitch, yaw,
         airspeed) = struct.unpack('<dddddddddddddddddd', buf)

        self.accel_body = Vector3(xAccel, yAccel, zAccel)
        self.gyro = Vector3(rollRate, pitchRate, yawRate)
        self.velocity = Vector3(speedN, speedE, speedD)
        dist = util.gps_distance(0, 0, latitude, longitude)
        bearing = util.gps_bearing(0, 0, latitude, longitude)
        self.position.x = dist * cos(radians(bearing))
        self.position.y = dist * sin(radians(bearing))
        self.position.z = -altitude
        self.dcm.from_euler(roll, pitch, yaw)
        self.time_now = timestamp + self.time_base

        # auto-adjust to crrcsim frame rate
        deltat = timestamp - self.last_timestamp
        if deltat < 0.01 and deltat > 0:
            self.adjust_frame_time(1.0/deltat)
        self.last_timestamp = timestamp

    def update(self, actuators):
        '''update model'''
        self.decode_servos(actuators)
        self.recv_fdm()
        self.update_position()
