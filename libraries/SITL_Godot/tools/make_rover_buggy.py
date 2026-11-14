#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/buggy.gltf: a generic four wheeled rover, front wheels
steered together and rear wheels driven, sized so that the Rover tests'
speeds and turn radii suit it.

  ./make_rover_buggy.py

AP_FLAKE8_CLEAN
'''

from gltf_model import Z, body, collider, part, qaxis, write

# the vehicle, in metres. glTF axes: +Z forward, +X left, +Y up, and the
# origin is the centre of the chassis at axle height
MASS = 3.0
TRACK = 0.30                # between the wheel centres, side to side
WHEELBASE = 0.44            # front axle to rear axle
WHEEL_RADIUS = 0.06
CHASSIS = [0.26, 0.08, 0.5]
CAGE = [0.2, 0.12, 0.22]
STEER_ANGLE = 30            # degrees either way
DRIVE_SERVO = 3
DRIVE_FORCE = 14            # newtons at each driven wheel
SUSPENSION = {"travel": 0.06, "spring": 900, "damping": 35}


def wheel(driven):
    p = {"radius": WHEEL_RADIUS}
    p.update(SUSPENSION)
    p["grip"] = 1.0 if driven else 0.9
    p["rollingResistance"] = 0.03
    if driven:
        p.update({"driveServo": DRIVE_SERVO, "maxDriveForce": DRIVE_FORCE})
    return p


collider("chassis", CHASSIS, [0, 0, 0])
collider("cage", CAGE, [0, 0.1, -0.05])

# the front wheels hang below their steering pivots, which turn about
# the vertical; SERVO1 above trim steers right
half_track = 0.5 * TRACK
half_base = 0.5 * WHEELBASE
for side, sign in (("fl", 1), ("fr", -1)):
    hub = part("wheel_" + side, "wheel", wheel(False), [-0.06, 0, 0], qaxis(Z, -90))
    part("steer_" + side, "hinge",
         {"servo": 1, "angleMin": STEER_ANGLE, "angleMax": -STEER_ANGLE, "maxRate": 400},
         [round(sign * half_track, 4), -0.02, round(half_base, 4)], qaxis(Z, 90), [hub])

# the rear wheels are driven, and sit at the bottom of their travel
for side, sign in (("rl", 1), ("rr", -1)):
    part("wheel_" + side, "wheel", wheel(True),
         [round(sign * half_track, 4), -0.08, round(-half_base, 4)])

body("buggy", [0, 0.13, 0], MASS, [0.07, 0.09, 0.03],
     {"drag": {"area": [0.04, 0.05, 0.03]},
      "chassis": {"style": "rover", "size": CHASSIS}}, com=[0, -0.02, 0])
write("buggy", [(0.9, 0.8, 0.05)])  # tyres grip the ground and do not bounce off it
