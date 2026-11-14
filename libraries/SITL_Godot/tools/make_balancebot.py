#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/balancebot.gltf: a two wheeled balancing rover sized like
SITL's own (libraries/SITL/SIM_BalanceBot.cpp), so that the gains in
Tools/autotest/default_params/balancebot.parm suit it.

  ./make_balancebot.py

AP_FLAKE8_CLEAN
'''

from gltf_model import body, collider, part, write

# SIM_BalanceBot's pendulum, which this one is built to match
MASS = 3.0             # kg, the body
WHEEL_MASS = 0.113     # kg each, carried in the body's mass below
COM_HEIGHT = 0.10      # m, centre of mass above the axle
PITCH_INERTIA = 0.0125  # kg.m^2 about the axle

WHEEL_RADIUS = 0.05    # m
TRACK = 0.15           # m between the wheels
BODY_WIDTH = 0.065     # m
BODY_HEIGHT = 0.24     # m
BODY_DEPTH = 0.08      # m

# SIM_BalanceBot's motors at stall: 12 V through 3 ohm at a torque
# constant of 0.24 is 0.96 N.m, which on a 50 mm wheel is this
DRIVE_FORCE = 19.0
# and their free speed: 12 V / 0.24 is 50 rad/s, which on the same wheel
# is this. The drive falls off in a straight line between the two.
FREE_SPEED = 2.5

# glTF axes: +Z forward, +X left, +Y up.  The origin is the centre of
# mass, COM_HEIGHT above the axle: the one dimension the controller's
# gains depend on.
collider("body", [BODY_WIDTH, BODY_HEIGHT, BODY_DEPTH], [0, 0.5 * BODY_HEIGHT - COM_HEIGHT, 0])

# A wheel each side, skid steered: SERVO1 is the left throttle (+X) and
# SERVO3 the right. The suspension is short and hard, or the controller
# would chase the springs rather than its own lean.
for name, sign, servo in (("left", 1, 1), ("right", -1, 3)):
    part("wheel_" + name, "wheel",
         {"radius": WHEEL_RADIUS, "travel": 0.002,
          "spring": 60000.0, "damping": 900.0,
          "grip": 1.3, "rollingResistance": 0.02,
          "driveServo": servo, "maxDriveForce": DRIVE_FORCE,
          "freeSpeed": FREE_SPEED},
         [round(sign * 0.5 * TRACK, 4), -COM_HEIGHT, 0])

# Pitch is about X, yaw about Y and roll about Z; the pitch figure is
# SIM_BalanceBot's own. Yaw is taken over the track and includes the
# wheels: as a column the width of the body it is too small to steer.
IYY = (MASS / 12.0 * (TRACK ** 2 + BODY_DEPTH ** 2)
       + 2 * WHEEL_MASS * (0.5 * TRACK) ** 2)
IZZ = MASS / 12.0 * (TRACK ** 2 + BODY_HEIGHT ** 2)

body("balancebot", [0, COM_HEIGHT + WHEEL_RADIUS, 0], round(MASS + 2 * WHEEL_MASS, 3),
     [PITCH_INERTIA, round(IYY, 5), round(IZZ, 5)],
     {"chassis": {"style": "balancebot",
                  "size": [BODY_WIDTH, BODY_HEIGHT, BODY_DEPTH],
                  "centre": [0, 0.5 * BODY_HEIGHT - COM_HEIGHT, 0]},
      "drag": {"area": [0.012, 0.004, 0.009]}})
write("balancebot", [(1.0, 0.9, 0.05)])
