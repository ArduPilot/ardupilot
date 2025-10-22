#!/usr/bin/env python3
'''
example vehicles using pyBullet
'''

import os
import sys
import time
import math
import socket
import struct
import json
import argparse

import pybullet as p
import pybullet_data

from pymavlink.rotmat import Vector3
from pymavlink.quaternion import Quaternion

# --- Argument parsing ---
parser = argparse.ArgumentParser(description="pybullet robot (no pyrobolearn)")
parser.add_argument("--vehicle", required=True, choices=['racecar', 'iris'], default='iris', help="vehicle type")
parser.add_argument("--fps", type=float, default=1200.0, help="physics frame rate")
parser.add_argument("--stadium", default=False, action='store_true', help="use stadium for world")
parser.add_argument("--nogui", default=False, action='store_true', help="disable GUI")
args = parser.parse_args()

# --- Constants ---
RATE_HZ = args.fps
TIME_STEP = 1.0 / RATE_HZ
GRAVITY_MSS = 9.80665

# --- PyBullet initialization ---
physicsClient = p.connect(p.DIRECT if args.nogui else p.GUI)
p.setTimeStep(TIME_STEP)
p.setGravity(0, 0, -GRAVITY_MSS)
p.setRealTimeSimulation(0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# --- Load environment ---
if args.stadium:
    p.loadSDF("stadium.sdf")
else:
    p.loadURDF("plane.urdf")

script_dir = os.path.dirname(os.path.abspath(__file__))

# --- State ---
time_now = 0.0
last_velocity = None
vehicle = None
robot_id = None


def constrain(v, min_v, max_v):
    '''constrain a value to a range'''
    return max(min_v, min(v, max_v))


class Iris(object):
    '''Iris quadcopter'''
    def __init__(self):
        global robot_id
        iris_path = os.path.join(script_dir, "models/iris/iris.urdf")
        robot_id = p.loadURDF(iris_path, [0, 0, 0.2])

        self.motor_indices = [1, 2, 3, 4]
        self.motor_dir = [1, 1, -1, -1]
        self.motor_speed = 5 # visual speed
        self.thrust_scale = 0.01

        # positive for CCW, negative for CW (quad-X layout)
        self.rotor_torque_dirs = [1, 1, -1, -1]
        self.torque_coef = 0.001  # Nm per unit thrust (tunable)

        # physical layout
        L = 0.2  # arm length
        self.rotor_positions = [
            [L, -L, 0],   # motor 1, Front-Right
            [-L, L, 0],   # motor 2, Rear-Left
            [L, L, 0],   # motor 3, Front-Left
            [-L, L, 0],   # motor 4, Rear-Right
        ]
        self.reset()
        print("Created Iris vehicle")

    def update(self, pwm):
        '''update Iris simulation'''
        num_motors = 4
        motors = pwm[:num_motors]

        # scale PWM to thrust (N) and torque (Nm)
        thrusts = [constrain(p - 1000, 0, 1000) * self.thrust_scale for p in motors]

        total_yaw_torque = 0.0

        for i in range(num_motors):
            force = [0, 0, thrusts[i]]
            p.applyExternalForce(
                objectUniqueId=robot_id,
                linkIndex=self.motor_indices[i],
                forceObj=force,
                posObj=[0, 0, 0],
                flags=p.LINK_FRAME
                )

            # accumulate torque (about Z axis)
            total_yaw_torque += self.rotor_torque_dirs[i] * thrusts[i] * self.torque_coef

        # Apply yaw torque to body
        p.applyExternalTorque(
            objectUniqueId=robot_id,
            linkIndex=-1,
            torqueObj=[0, 0, -total_yaw_torque],
            flags=p.LINK_FRAME
        )

        # animate motor spinning
        for i in range(num_motors):
            speed = constrain(motors[i] - 1000.0, 0, 1000) * self.motor_dir[i] * self.motor_speed
            p.setJointMotorControl2(robot_id, self.motor_indices[i], p.VELOCITY_CONTROL, targetVelocity=speed)

    def reset(self):
        '''reset time and location'''
        p.resetBasePositionAndOrientation(robot_id, [0, 0, 0.2], [0, 0, 0, 1])


class RaceCar(object):
    '''racing car'''
    def __init__(self):
        global robot_id
        robot_id = p.loadURDF("racecar/racecar.urdf", [0, 0, 0.2])

        self.steering_joints = [4, 6]
        self.wheel_joints = [2, 3, 5, 7]
        self.steer_max = 45.0
        self.throttle_max = 200.0

        self.reset()

        print("Created RaceCar vehicle")

    def update(self, pwm):
        '''update RaceCar simulation'''
        steering = constrain((pwm[0] - 1500.0)/500.0, -1, 1) * math.radians(self.steer_max) * -1
        throttle = constrain((pwm[2] - 1500.0)/500.0, -1, 1) * self.throttle_max

        for joint in self.wheel_joints:
            p.setJointMotorControl2(robot_id, joint,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=throttle)
        for joint in self.steering_joints:
            p.setJointMotorControl2(robot_id, joint,
                                    p.POSITION_CONTROL,
                                    targetPosition=steering)

    def reset(self):
        '''reset time and location'''
        p.resetBasePositionAndOrientation(robot_id, [0, 0, 0.2], [0, 0, 0, 1])


def vector_to_AP(vec):
    return Vector3(vec[0], -vec[1], -vec[2])


def to_tuple(v3):
    return (v3.x, v3.y, v3.z)


def quaternion_to_AP(q):
    '''convert pybullet quaternion to ArduPilot quaternion'''
    return Quaternion([q[3], q[0], -q[1], -q[2]])


def physics_step(pwm_in):
    global time_now, last_velocity
    vehicle.update(pwm_in)
    p.stepSimulation()
    time_now += TIME_STEP

    pos, orn = p.getBasePositionAndOrientation(robot_id)
    lin_vel, ang_vel = p.getBaseVelocity(robot_id)

    q_ap = quaternion_to_AP(orn)
    roll, pitch, yaw = q_ap.euler
    velocity = vector_to_AP(lin_vel)
    position = vector_to_AP(pos)

    dcm = q_ap.dcm
    gyro = dcm.transposed() * vector_to_AP(ang_vel)

    if last_velocity is None:
        last_velocity = velocity

    accel = (velocity - last_velocity) * (1.0 / TIME_STEP)
    last_velocity = velocity
    accel.z -= GRAVITY_MSS
    accel = dcm.transposed() * accel

    return time_now, to_tuple(gyro), to_tuple(accel), to_tuple(position), (roll, pitch, yaw), to_tuple(velocity)


# --- UDP communication setup ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 9002))
sock.settimeout(0.1)

last_SITL_frame = -1
connected = False
frame_count = 0
frame_time = time.time()
print_frame_count = 1000

vehicles = {
    "iris" : Iris,
    "racecar" : RaceCar
}

if args.vehicle not in vehicles:
    print(f"Unknown vehicle {args.vehicle}")
    sys.exit(1)
vehicle = vehicles[args.vehicle]()

# show the joints
print("Vehicle joints:")
number_of_joints = p.getNumJoints(robot_id)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(robot_id, joint_number)
    print(" %s : %s" % (info[0], info[1]))

# --- Main loop ---
while True:
    try:
        data, address = sock.recvfrom(100)
    except Exception:
        time.sleep(0.01)
        continue

    parse_format = 'HHI16H'
    if len(data) != struct.calcsize(parse_format):
        print(f"Bad packet size: {len(data)}")
        continue

    decoded = struct.unpack(parse_format, data)
    magic = 18458
    if decoded[0] != magic:
        print(f"Incorrect magic: {decoded[0]}")
        continue

    frame_rate_hz = decoded[1]
    frame_number = decoded[2]
    pwm = decoded[3:]

    if frame_rate_hz != RATE_HZ:
        RATE_HZ = frame_rate_hz
        TIME_STEP = 1.0 / RATE_HZ
        p.setTimeStep(TIME_STEP)
        print(f"Updated rate to {RATE_HZ} Hz")

    if frame_number < last_SITL_frame:
        vehicle.reset()
        time_now = 0.0
        print("Controller reset")
    elif frame_number != last_SITL_frame + 1 and connected:
        print(f"Missed {frame_number - last_SITL_frame - 1} frames")

    last_SITL_frame = frame_number

    if not connected:
        connected = True
        print(f"Connected to {address}")

    frame_count += 1

    phys_time, gyro, accel, pos, euler, velo = physics_step(pwm)

    json_data = {
        "timestamp": phys_time,
        "imu": {
            "gyro": gyro,
            "accel_body": accel
        },
        "position": pos,
        "attitude": euler,
        "velocity": velo
    }

    sock.sendto((json.dumps(json_data, separators=(',', ':')) + "\n").encode("ascii"), address)

    if frame_count % print_frame_count == 0:
        now = time.time()
        total_time = now - frame_time
        print(f"{print_frame_count/total_time:.2f} fps T={phys_time:.3f} dt={total_time:.3f}")
        frame_time = now
