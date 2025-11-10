#!/usr/bin/env python3

# flake8: noqa

import os, inspect, sys

import socket
import struct
import json
import math
import pybullet_data
import pybullet as p
# use pymavlink for ArduPilot convention transformations
from pymavlink.rotmat import Vector3, Matrix3
from pymavlink.quaternion import Quaternion

import time

import argparse
from math import degrees, radians

parser = argparse.ArgumentParser(description="pybullet robot")
parser.add_argument("--fps", type=float, default=1000.0, help="physics frame rate")

args = parser.parse_args()

RATE_HZ = args.fps
TIME_STEP = 1.0 / RATE_HZ
GRAVITY_MSS = 9.80665

# Create simulator
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
FixedBase = False #if fixed no plane is imported
if (FixedBase == False):
    p.loadURDF("plane.urdf")
p.changeDynamics(FixedBase,-1,lateralFriction=1.,
                 spinningFriction=0., rollingFriction=0., contactStiffness=-1, contactDamping=-1)

last_angles = [0.0] * 12
force = [500] * 12
servo_direction = [1,-1, 1,
                   1, 1,-1,
                   1,-1, 1,
                   1, 1,-1]
def control_joints(pwm):
    '''control a joint based bot'''
    global last_angles
    joint_speed = radians(250)
    joints = [0,1,2,4,5,6,8,9,10,12,13,14]
    pwm = pwm[0:len(joints)]
    angles = [radians(((v-1500.0)*90)/1000) for v in pwm ]
    for i in range(len(angles)):
      angles[i] = angles[i] * servo_direction[i]
    current = last_angles
    max_change = joint_speed * TIME_STEP
    for i in range(len(angles)):
        angles[i] = constrain(angles[i], current[i]-max_change, current[i]+max_change)
    p.setJointMotorControlArray(robot, joints, p.POSITION_CONTROL, angles,forces = force)
    last_angles = angles

#spawn robot
position = [0, 0, 1.6]
robot = p.loadURDF("models/quadruped/quadruped.urdf",position, useFixedBase=FixedBase)
control_pwm = control_joints

p.setTimeStep(TIME_STEP)
time_now = 0
last_velocity = None

def quaternion_to_AP(quaternion):
    '''convert pybullet quaternion to ArduPilot quaternion'''
    return Quaternion([quaternion[3], quaternion[0], -quaternion[1], -quaternion[2]])

def vector_to_AP(vec):
    '''convert pybullet vector tuple to ArduPilot Vector3'''
    return Vector3(vec[0], -vec[1], -vec[2])

def quaternion_from_AP(q):
    '''convert ArduPilot quaternion to pybullet quaternion'''
    return [q.q[1], -q.q[2], -q.q[3], q.q[0]]

def to_tuple(vec3):
    '''convert a Vector3 to a tuple'''
    return (vec3.x, vec3.y, vec3.z)

def init():
  global time_now
  time_now = 0
  position = [0,0,0]
  orientation = [0,0,0,1]
  p.reset_Base_Position_And_Orientations(robot,position,orientation)


def constrain(v,min_v,max_v):
    '''constrain a value'''
    if v < min_v:
        v = min_v
    if v > max_v:
        v = max_v
    return v

#robot.position = [ 0, 0, 2]
#robot.orientation = quaternion_from_AP(Quaternion([math.radians(0), math.radians(0), math.radians(50)]))
def step(sleep_dt=None):
    # # call the step method for each body
    # for body in self.bodies.values():
    #     if isinstance(body, Body):
    #         body.step()

    # call simulation step
    p.stepSimulation()

    # sleep
    if sleep_dt is not None:
        time.sleep(sleep_dt)

def physics_step(pwm_in):

  control_pwm(pwm_in)
  
  step(sleep_dt=0)
  # p.setRealTimeSimulation(1)
  global time_now
  time_now += TIME_STEP

  # get the position orientation and velocity
  pos,ori = p.getBasePositionAndOrientation(robot)
  quaternion = quaternion_to_AP(ori)
  roll, pitch, yaw = quaternion.euler
  linear,angular = p.getBaseVelocity(robot)
  velocity = vector_to_AP(linear)
  position = vector_to_AP(pos)

  # get ArduPilot DCM matrix (rotation matrix)
  dcm = quaternion.dcm

  # get gyro vector in body frame
  gyro = dcm.transposed() * vector_to_AP(angular)
  
  # calculate acceleration
  global last_velocity
  if last_velocity is None:
      last_velocity = velocity

  accel = (velocity - last_velocity) * (1.0 / TIME_STEP)
  last_velocity = velocity

  # add in gravity in earth frame
  accel.z -= GRAVITY_MSS

  # convert accel to body frame
  accel = dcm.transposed() * accel

  # convert to tuples
  accel = to_tuple(accel)
  gyro = to_tuple(gyro)
  position = to_tuple(position)
  velocity = to_tuple(velocity)
  euler = (roll, pitch, yaw)

  return time_now,gyro,accel,position,euler,velocity

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 9002))
sock.settimeout(0.1)

last_SITL_frame = -1
connected = False
frame_count = 0
frame_time = time.time()
print_frame_count = 1000

move_accel = 0.0
last_move = time.time()

while True:

  py_time = time.time()

  try:
      data,address = sock.recvfrom(100)
  except Exception as ex:
      time.sleep(0.01)
      continue

  parse_format = 'HHI16H'
  magic = 18458

  if len(data) != struct.calcsize(parse_format):
    print("got packet of len %u, expected %u" % (len(data), struct.calcsize(parse_format)))
    continue


  decoded = struct.unpack(parse_format,data)

  if magic != decoded[0]:
      print("Incorrect protocol magic %u should be %u" % (decoded[0], magic))
      continue

  frame_rate_hz = decoded[1]
  frame_count = decoded[2]
  pwm = decoded[3:]

  if frame_rate_hz != RATE_HZ:
      print("New frame rate %u" % frame_rate_hz)
      RATE_HZ = frame_rate_hz
      TIME_STEP = 1.0 / RATE_HZ
      p.setTimeStep(TIME_STEP)

  # Check if the fame is in expected order
  if frame_count < last_SITL_frame:
    # Controller has reset, reset physics also
    init()
    print('Controller reset')
  elif frame_count == last_SITL_frame:
    # duplicate frame, skip
    print('Duplicate input frame')
    continue
  elif frame_count != last_SITL_frame + 1 and connected:
    print('Missed {0} input frames'.format(frame_count - last_SITL_frame - 1))
  last_SITL_frame = frame_count

  if not connected:
    connected = True
    print('Connected to {0}'.format(str(address)))
  frame_count += 1

  # physics time step
  phys_time,gyro,accel,pos,euler,velo = physics_step(pwm)

  # build JSON format
  IMU_fmt = {
    "gyro" : gyro,
    "accel_body" : accel
  }
  JSON_fmt = {
    "timestamp" : phys_time,
    "imu" : IMU_fmt,
    "position" : pos,
    "attitude" : euler,
    "velocity" : velo
  }
  JSON_string = "\n" + json.dumps(JSON_fmt,separators=(',', ':')) + "\n"

  # Send to AP
  sock.sendto(bytes(JSON_string,"ascii"), address)

  # Track frame rate
  if frame_count % print_frame_count == 0:
    now = time.time()
    total_time = now - frame_time
    print("%.2f fps T=%.3f dt=%.3f" % (print_frame_count/total_time, phys_time, total_time))
    frame_time = now
