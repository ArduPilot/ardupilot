#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/plane3d.gltf: an aerobatic plane matching SITL's plane-3d,
2 kg with a 1.88 m span and 0.45 m^2 of wing, for 3D and scripted
aerobatics.

  ./make_plane3d.py

AP_FLAKE8_CLEAN
'''

from gltf_model import X, Z, body, collider, part, qaxis, write

# glTF axes: +Z forward, +X left, +Y up. The body origin is the centre of
# mass, 3 cm behind the wing's quarter chord for near-neutral stability.
collider("fuselage", [0.1, 0.12, 1.1], [0, 0, -0.1])
collider("wing_box", [1.88, 0.03, 0.24], [0, 0, -0.03])
collider("tail_box", [0.6, 0.01, 0.167], [0, 0, -0.78])
collider("fin_box", [0.01, 0.3, 0.2], [0, 0.15, -0.79])

# symmetric aerofoil, full span ailerons
wing = {"span": 0.47, "chord": 0.24, "aspectRatio": 7.85, "cl0": 0.0, "cd0": 0.02, "stallAngle": 27,
        "flapServo": 1, "flapChordFraction": 0.3}
for side, sign in (("left", 1), ("right", -1)):
    for panel, x in (("inner", 0.235), ("outer", 0.705)):
        params = dict(wing)
        # SERVO1 above trim rolls right: left aileron trailing edge down
        params.update({"flapAngleMin": -30 * sign, "flapAngleMax": 30 * sign})
        part("wing_%s_%s" % (side, panel), "surface", params, [round(sign * x, 4), 0, 0.03])

# tailplane at -0.5 deg incidence; SERVO2 above trim pitches up: trailing edge up.
# The tail stalls at 35 deg, above the 25-29 deg a full control throw adds:
# the aerobatics schedules rely on full-throw authority, as SITL's plane-3d gives.
part("tailplane", "surface",
     {"span": 0.6, "chord": 0.167, "cd0": 0.02, "stallAngle": 35, "downwash": 0.45,
      "flapServo": 2, "flapAngleMin": 30, "flapAngleMax": -30, "flapChordFraction": 0.5},
     [0, 0, -0.75], qaxis(X, 0.5))
# fin: span up, lift to the right; SERVO4 above trim yaws right
part("fin", "surface",
     {"span": 0.3, "chord": 0.2, "aspectRatio": 2.5, "cd0": 0.02, "stallAngle": 35,
      "flapServo": 4, "flapAngleMin": 35, "flapAngleMax": -35, "flapChordFraction": 0.7},
     [0, 0.15, -0.76], qaxis(Z, 90))
# the fuselage as a lifting body in sideslip: without its side force a
# skidded turn cannot curve the path, and the aerobatics schedules fly
# skids. At the centre of mass, so it adds force but no yawing moment.
part("fuselage_side", "surface",
     {"span": 0.12, "chord": 1.1, "aspectRatio": 0.3, "cd0": 0.0, "stallAngle": 40},
     [0, 0, 0], qaxis(Z, 90))

# thrust as SITL's plane-3d: constant with airspeed, 42 N at full throttle
part("propeller", "rotor",
     {"servo": 3, "maxThrust": 42.0, "thrustExpo": 0.5, "torqueRatio": 0.003,
      "direction": "ccw", "timeConstant": 0.05, "diameter": 0.3, "pitchSpeed": 0.0},
     [0, 0, 0.5], qaxis(X, 90))

# Names avoid Godot's import hint suffixes (-col, -rigid, -wheel, ...).
nose_wheel = part("wheel_nose", "wheel",
                  {"radius": 0.035, "travel": 0.04, "spring": 400, "damping": 15, "grip": 0.9,
                   "rollingResistance": 0.03},
                  [-0.14, 0, 0], qaxis(Z, -90))
part("steering_nose", "hinge", {"servo": 4, "angleMin": 25, "angleMax": -25, "maxRate": 200},
     [0, -0.06, 0.32], qaxis(Z, 90), [nose_wheel])
for side, x in (("left", 0.16), ("right", -0.16)):
    part("wheel_" + side, "wheel",
         {"radius": 0.04, "travel": 0.04, "spring": 700, "damping": 25, "grip": 0.8,
          "rollingResistance": 0.03},
         [x, -0.2, -0.07])

body("plane3d", [0, 0.25, 0], 2.0, [0.18, 0.30, 0.13],
     {"chassis": {"style": "plane", "size": [0.1, 0.12, 1.1]},
      "drag": {"area": [0.06, 0.06, 0.045]}})
write("plane3d")
