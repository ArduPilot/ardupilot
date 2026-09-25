#!/usr/bin/env python3
"""JSON backend for a vehicle that is commanded by velocity, not by joint angles.

Many walking robots (and a lot of tracked/differential platforms) ship with their
own locomotion controller.  ArduPilot does not drive their joints; it asks for a
forward speed and a turn rate, and the on-board controller works out how to
achieve it.  This example models that class of vehicle so Rover's navigation and
mission code can be exercised without any physics engine.

Contrast with ``../pybullet/walking_robot.py``, which drives twelve leg joints
directly from twelve servo outputs.

Only the Python standard library is used.

Usage:
    python3 velocity_controlled_robot.py
    ardurover --model JSON:127.0.0.1 --defaults default.parm

See readme.md for the parameters Rover needs for this vehicle model.
"""

import argparse
import json
import math
import socket
import struct

MAGIC = 18458
PACKET = struct.Struct("<HHI16H")

# ArduPilot sends 0 on servo channels it is not driving (for example while
# disarmed).  Treating 0 as a PWM value would scale to full reverse, so anything
# outside a sane band is taken as neutral.
PWM_MIN = 800
PWM_MAX = 2200
PWM_NEUTRAL = 1500


def clamp(value, low, high):
    return max(low, min(high, value))


def normalise(pwm):
    """Scale one servo output to -1..1, treating undriven channels as neutral."""
    if pwm < PWM_MIN or pwm > PWM_MAX:
        return 0.0
    return clamp((pwm - PWM_NEUTRAL) / 500.0, -1.0, 1.0)


def pwm_to_command(pwm, max_speed, max_turn_rate):
    """Convert skid steering outputs to a forward speed and a turn rate.

    The skid mixer in AP_MotorsUGV computes::

        motor_left  = throttle + steering
        motor_right = throttle - steering

    so a positive turn rate (clockwise, NED) means the left motor runs faster.
    Getting this sign backwards is not obvious from the servo output alone: the
    vehicle settles pointing 180 degrees away from the target and yaws forever.
    """
    left = normalise(pwm[0])    # SERVO1_FUNCTION = 73, throttleLeft
    right = normalise(pwm[2])   # SERVO3_FUNCTION = 74, throttleRight
    speed = (left + right) * 0.5 * max_speed
    turn_rate = (left - right) * 0.5 * max_turn_rate
    return speed, turn_rate


class Vehicle(object):
    """Planar unicycle with a first order lag on the commanded velocity.

    The lag stands in for however long the on-board locomotion controller takes
    to reach a newly commanded velocity.
    """

    def __init__(self, tau):
        self.tau = tau
        self.reset()

    def reset(self):
        self.timestamp = 0.0
        self.north = 0.0
        self.east = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.turn_rate = 0.0
        self.accel_forward = 0.0

    def update(self, speed_cmd, turn_rate_cmd, dt):
        previous_speed = self.speed
        gain = 1.0 if dt >= self.tau else dt / self.tau
        self.speed += (speed_cmd - self.speed) * gain
        self.turn_rate += (turn_rate_cmd - self.turn_rate) * gain

        self.accel_forward = (self.speed - previous_speed) / dt
        self.yaw += self.turn_rate * dt
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        self.north += self.speed * math.cos(self.yaw) * dt
        self.east += self.speed * math.sin(self.yaw) * dt
        self.timestamp += dt

    def state(self):
        return {
            "timestamp": self.timestamp,
            "imu": {
                "gyro": [0.0, 0.0, self.turn_rate],
                # x is forward acceleration, y is centripetal, z is the
                # reaction to gravity for a vehicle sitting level.
                "accel_body": [
                    self.accel_forward,
                    self.speed * self.turn_rate,
                    -9.80665,
                ],
            },
            "position": [self.north, self.east, 0.0],
            "attitude": [0.0, 0.0, self.yaw],
            "velocity": [
                self.speed * math.cos(self.yaw),
                self.speed * math.sin(self.yaw),
                0.0,
            ],
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=9002)
    parser.add_argument("--max-speed", type=float, default=1.5,
                        help="forward speed at full throttle (m/s)")
    parser.add_argument("--max-turn-rate", type=float, default=1.2,
                        help="turn rate at full steering (rad/s)")
    parser.add_argument("--tau", type=float, default=0.2,
                        help="first order lag of the locomotion controller (s)")
    args = parser.parse_args()

    vehicle = Vehicle(args.tau)
    last_frame = None
    reply = ("\n" + json.dumps(vehicle.state()) + "\n").encode()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print("listening for ArduPilot on UDP %d" % args.port)

    while True:
        data, address = sock.recvfrom(2048)
        if len(data) != PACKET.size:
            continue
        values = PACKET.unpack(data)
        if values[0] != MAGIC:
            continue
        frame_rate, frame_count, pwm = values[1], values[2], values[3:]
        if frame_rate == 0:
            continue

        # A frame count that goes backwards means SITL restarted.
        if last_frame is not None and frame_count < last_frame:
            print("ArduPilot restarted, resetting vehicle")
            vehicle.reset()
            last_frame = None

        # Repeated frame counts are retransmissions; do not advance time twice.
        if frame_count != last_frame:
            speed, turn_rate = pwm_to_command(
                pwm, args.max_speed, args.max_turn_rate)
            vehicle.update(speed, turn_rate, 1.0 / frame_rate)
            reply = ("\n" + json.dumps(vehicle.state()) + "\n").encode()
            last_frame = frame_count

        sock.sendto(reply, address)


if __name__ == "__main__":
    main()
