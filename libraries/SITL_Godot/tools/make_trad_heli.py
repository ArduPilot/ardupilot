#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/heli.gltf: a generic single rotor helicopter on skids, with
a tail rotor on a boom, sized so that the Copter tests' helicopter
collective and headspeed settings suit it.

  ./make_trad_heli.py

AP_FLAKE8_CLEAN
'''

from gltf_model import Z, body, collider, part, qaxis, write

# the helicopter, in metres. glTF axes: +Z forward, +X left, +Y up, and
# the origin is the centre of mass, under the rotor shaft
MASS = 4.54
ROTOR_DIAMETER = 1.4
ROTOR_Y = 0.3               # the head, above the centre of mass
TAIL_Z = -0.85              # the tail rotor, behind it
TAIL_DIAMETER = 0.22
TAIL_THRUST = 8.0           # newtons, either way, against the main torque
BOOM = [0.04, 0.04, 0.7]
FUSELAGE = [0.2, 0.25, 0.55]
SKID_X = 0.15               # the skids out from the centreline
SKID_Y = -0.32              # and below the centre of mass
COLLECTIVE = 25             # degrees either way
HEADSPEED = 1500            # rpm

collider("fuselage", FUSELAGE, [0, 0, 0])
collider("tail_boom", BOOM, [0, 0.08, -0.55])
for side, sign in (("left", 1), ("right", -1)):
    collider("skid_" + side, [0.03, 0.03, 0.6], [sign * SKID_X, SKID_Y, 0.05])
collider("fin_box", [0.01, 0.2, 0.12], [0, 0.1, TAIL_Z - 0.03])

# the head: an H3-120 swashplate on servos 1 to 3, with the rotor speed
# controller on servo 8
part("main_rotor", "heliRotor",
     {"swashServo1": 1, "swashServo2": 2, "swashServo3": 3, "rscServo": 8,
      "collectiveMin": -COLLECTIVE, "collectiveMax": COLLECTIVE,
      "cyclicScale": 10, "nominalRpm": HEADSPEED, "spoolTime": 5,
      "thrustScale": 9.0, "torqueRatio": 0.035, "hubStiffness": 18,
      "flapTimeConstant": 0.04, "direction": "cw",
      "diameter": ROTOR_DIAMETER},
     [0, ROTOR_Y, 0])

# the tail rotor pushes sideways against the main rotor's torque, so it
# runs both ways and its thrust does not depend on the airspeed
part("tail_rotor", "rotor",
     {"servo": 4, "bidirectional": True, "maxThrust": TAIL_THRUST,
      "thrustExpo": 0.2, "torqueRatio": 0.0, "direction": "ccw",
      "timeConstant": 0.02, "diameter": TAIL_DIAMETER, "pitchSpeed": 0.0},
     [0.06, 0.1, TAIL_Z], qaxis(Z, -90))

# a stabiliser and fin on the boom, which steady it in forward flight
part("stabiliser", "surface", {"span": 0.16, "chord": 0.1, "cd0": 0.01, "stallAngle": 20},
     [0, 0.12, TAIL_Z + 0.1])
part("fin", "surface", {"span": 0.18, "chord": 0.1, "cd0": 0.01, "stallAngle": 20},
     [0, 0.2, TAIL_Z], qaxis(Z, 90))

body("heli", [0, 0.34, 0], MASS, [0.35, 0.25, 0.12],
     {"drag": {"area": [0.1, 0.12, 0.05]},
      "chassis": {"style": "helicopter", "size": FUSELAGE}})
write("heli", [(0.6, 0.5, 0.05)])   # skids settle rather than bounce
