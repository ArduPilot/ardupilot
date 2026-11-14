#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/quadplane.gltf: a 3.2 kg quadplane on a twin tail boom, with
four lift motors on the booms and a puller on the nose, sized so that the
QuadPlane tests' transition airspeeds and hover throttle suit it.

  ./make_quadplane.py

AP_FLAKE8_CLEAN
'''

import math

from gltf_model import X, Z, body, collider, part, qaxis, write

# the airframe, in metres. glTF axes: +Z forward, +X left, +Y up, and the
# origin is the centre of mass, a little ahead of the wing's quarter chord
MASS = 3.2
SPAN = 1.6                  # tip to tip
CHORD = 0.25
DIHEDRAL = 3.0              # degrees, the outer panels only
WING_Y = 0.07               # above the centre of mass
BOOM_X = 0.35               # each boom out from the centreline
BOOM_Y = 0.05
TAIL_Z = -0.65              # tailplane quarter chord, behind the origin
TAIL_SPAN = 2 * BOOM_X + 0.06
FIN_SPAN = 0.2
VTOL_Z = 0.35               # lift motors fore and aft along the booms
VTOL_Y = 0.12
VTOL_THRUST = 16.0          # newtons each: about two to one in the hover
PROP_THRUST = 36.0
NOSE_WHEEL_Z = 0.3
MAIN_WHEEL_Z = -0.12

collider("fuselage", [0.1, 0.1, 0.9], [0, 0, 0])

# the wing in two panels a side: the outer pair carry the ailerons, and
# sit a little higher on the dihedral
panel = SPAN / 4
wing = {"span": panel, "chord": CHORD, "aspectRatio": round(SPAN / CHORD, 2),
        "cl0": 0.25, "cm0": -0.04, "cd0": 0.012, "stallAngle": 14}
rise = math.tan(math.radians(DIHEDRAL))
for side, sign in (("left", 1), ("right", -1)):
    for panel_name, mid in (("inner", 0.5 * panel), ("outer", 1.5 * panel)):
        params = dict(wing)
        if panel_name == "outer":
            # SERVO1 above trim rolls right: the left aileron goes down
            params.update({"flapServo": 1, "flapAngleMin": -20 * sign,
                           "flapAngleMax": 20 * sign, "flapChordFraction": 0.25})
        y = WING_Y + (mid - 0.5 * panel) * rise
        part("wing_%s_%s" % (side, panel_name), "surface", params,
             [round(sign * mid, 4), round(y, 4), 0.0],
             qaxis(Z, DIHEDRAL * sign if panel_name == "outer" else 0.0))
        collider("wing_box_%s_%s" % (side, panel_name), [panel, 0.02, CHORD],
                 [round(sign * mid, 4), round(y, 4), -0.25 * CHORD])

# twin booms from the wing back to the tail, with a fin on each
for side, sign in (("left", 1), ("right", -1)):
    collider("boom_" + side, [0.03, 0.04, 0.85], [sign * BOOM_X, BOOM_Y, -0.33])
    part("fin_" + side, "surface",
         {"span": FIN_SPAN, "chord": 0.15, "aspectRatio": 2.5, "cd0": 0.015,
          "stallAngle": 18, "flapServo": 4, "flapAngleMin": 25,
          "flapAngleMax": -25, "flapChordFraction": 0.35},
         [sign * BOOM_X, 0.15, TAIL_Z - 0.01], qaxis(Z, 90))
    collider("fin_box_" + side, [0.01, FIN_SPAN, 0.15], [sign * BOOM_X, 0.15, TAIL_Z - 0.05])

# the tailplane spans the booms; SERVO2 above trim pitches up
part("tailplane", "surface",
     {"span": round(TAIL_SPAN, 4), "chord": 0.16, "cd0": 0.015, "stallAngle": 15,
      "downwash": 0.45, "flapServo": 2, "flapAngleMin": 15, "flapAngleMax": -15,
      "flapChordFraction": 0.35},
     [0, BOOM_Y, TAIL_Z], qaxis(X, 1.7))
collider("tail_box", [round(TAIL_SPAN, 4), 0.01, 0.16], [0, BOOM_Y, TAIL_Z - 0.04])

part("propeller", "rotor",
     {"servo": 3, "maxThrust": PROP_THRUST, "thrustExpo": 0.3,
      "torqueRatio": 0.012, "direction": "ccw", "timeConstant": 0.1,
      "diameter": 0.3, "pitchSpeed": 95.0},
     [0, 0, 0.45], qaxis(X, 90))

# the lift motors, in ArduPilot's quad X order: 1 front right, 2 back
# left, 3 front left, 4 back right, with 1 and 2 turning the same way
for servo, sx, sz, spin in ((5, -1, 1, "ccw"), (6, 1, -1, "ccw"),
                            (7, 1, 1, "cw"), (8, -1, -1, "cw")):
    part("vtol_motor%u" % (servo - 4), "rotor",
         {"servo": servo, "maxThrust": VTOL_THRUST, "thrustExpo": 0.65,
          "torqueRatio": 0.016, "direction": spin, "timeConstant": 0.03,
          "diameter": 0.33, "pitchSpeed": 20.0, "inplaneDrag": 0.02},
         [sx * BOOM_X, VTOL_Y, sz * VTOL_Z])

# tricycle gear: a steered nose wheel, and a main wheel under each boom
nose_wheel = part("wheel_nose", "wheel",
                  {"radius": 0.035, "travel": 0.04, "spring": 500, "damping": 18,
                   "grip": 0.9, "rollingResistance": 0.12},
                  [-0.12, 0, 0], qaxis(Z, -90))
part("steering_nose", "hinge", {"servo": 4, "angleMin": 10, "angleMax": -10, "maxRate": 200},
     [0, -0.05, NOSE_WHEEL_Z], qaxis(Z, 90), [nose_wheel])
for side, sign in (("left", 1), ("right", -1)):
    part("wheel_" + side, "wheel",
         {"radius": 0.04, "travel": 0.04, "spring": 900, "damping": 30,
          "grip": 0.8, "rollingResistance": 0.12},
         [sign * BOOM_X, -0.17, MAIN_WHEEL_Z])

# the backend draws the airframe from the chassis: a body with a boom
# each side carrying the lift motors, square to the wing
body("quadplane", [0, 0.22, 0], MASS, [0.45, 0.6, 0.35],
     {"drag": {"area": [0.07, 0.08, 0.035]},
      "chassis": {"style": "quadplane", "size": [0.1, 0.1, 0.9]}})
write("quadplane")
