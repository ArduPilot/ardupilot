#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/quadx.gltf: a generic 1.5 kg X quadcopter for ArduCopter's
default frame.

  ./make_quadx.py

AP_FLAKE8_CLEAN
'''

from gltf_model import Y, add, body, collider_ext, part, qaxis, shape, write

FRAME = shape([0.12, 0.05, 0.16])
BOOM = shape([0.34, 0.015, 0.02])
LEG = shape({"type": "cylinder", "cylinder": {"height": 0.12, "radiusTop": 0.006, "radiusBottom": 0.006}})
FOOT = shape({"type": "sphere", "sphere": {"radius": 0.012}})


def solid(name, shape_index, **where):
    add({"name": name, **where, "extensions": collider_ext(shape_index)})


# the booms lie along the diagonals, 45 degrees either way about Y
_, s, _, c = qaxis(Y, 45)
solid("frame", FRAME)
solid("arm_fr_bl", BOOM, rotation=[0, s, 0, c])
solid("arm_fl_br", BOOM, rotation=[0, -s, 0, c])
for name, x, z in [("fl", 0.08, 0.08), ("fr", -0.08, 0.08), ("bl", 0.08, -0.08), ("br", -0.08, -0.08)]:
    solid("leg_" + name, LEG, translation=[x, -0.06, z])
    solid("foot_" + name, FOOT, translation=[x, -0.12, z])

# ArduPilot quad X motor order. glTF axes: +Z forward, -X right, +Y up
arm = 0.12
motors = [
    (1, -arm, arm, "ccw"),   # front right
    (2, arm, -arm, "ccw"),   # back left
    (3, arm, arm, "cw"),     # front left
    (4, -arm, -arm, "cw"),   # back right
]
for servo, x, z, direction in motors:
    part("motor%d" % servo, "rotor",
         {"servo": servo, "pwmMin": 1000, "pwmMax": 2000, "maxThrust": 9.8, "thrustExpo": 0.65,
          "torqueRatio": 0.016, "direction": direction, "timeConstant": 0.03, "diameter": 0.254,
          "pitchSpeed": 20.0, "inplaneDrag": 0.02},
         [x, 0.025, z])

# the backend draws the airframe from the chassis: a centre with the
# booms, motors and legs its own rotor positions imply
body("quadx", [0, 0.135, 0], 1.5, [0.015, 0.028, 0.015],
     {"drag": {"area": [0.05, 0.08, 0.05]},
      "chassis": {"style": "multirotor", "size": [0.16, 0.07, 0.16]}})
write("quadx", [(0.9, 0.8, 0.05)])
