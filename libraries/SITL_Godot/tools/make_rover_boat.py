#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/speedboat.gltf: a generic boat floating on four buoyancy
points, driven by a single steerable outboard at the stern, sized so
that the Rover tests' motorboat speeds and turn rates suit it.

  ./make_rover_boat.py

AP_FLAKE8_CLEAN
'''

from gltf_model import X, Z, body, collider, part, qaxis, write

# the boat, in metres. glTF axes: +Z forward, +X left, +Y up, and the
# origin is at the waterline on the hull's centreline
MASS = 3.0
HULL = [0.35, 0.12, 1.2]
CONSOLE = [0.2, 0.15, 0.3]
FLOATER_X = 0.14              # buoyancy points out from the centreline
FLOATER_Z = 0.45              # and fore and aft of the origin
FLOATER_VOLUME = 0.004        # cubic metres each: four of these float the hull
OUTBOARD_Z = -0.62          # the leg's pivot, at the transom
THRUST = 25.0               # newtons, ahead or astern
STEER_ANGLE = 35            # degrees either way

collider("hull", HULL, [0, -0.02, 0])
collider("console", CONSOLE, [0, 0.1, -0.1])

# buoyancy at the quarters: displacing more as the hull settles, which
# rights the boat and holds it at its waterline
for end, sz in (("bow", 1), ("stern", -1)):
    for side, sx in (("left", 1), ("right", -1)):
        part("floater_%s_%s" % (end, side), "floater",
             {"volume": FLOATER_VOLUME, "height": 0.2,
              "dragArea": [0.01, 0.004, 0.0003], "damping": 5.0},
             [sx * FLOATER_X, -0.05, sz * FLOATER_Z])

# the outboard: a propeller on a leg which the steering servo turns,
# so its thrust points where the leg does
prop = part("outboard_prop", "rotor",
            {"servo": 3, "bidirectional": True, "maxThrust": THRUST, "thrustExpo": 0.2,
             "torqueRatio": 0.01, "direction": "ccw", "timeConstant": 0.1,
             "diameter": 0.12, "pitchSpeed": 14.0},
            [-0.15, 0, -0.08], qaxis(X, 90))
part("outboard_steering", "hinge",
     {"servo": 1, "angleMin": -STEER_ANGLE, "angleMax": STEER_ANGLE, "maxRate": 200},
     [0, 0, OUTBOARD_Z], qaxis(Z, 90), [prop])

body("speedboat", [0, 0.11, 0], MASS, [0.3, 0.35, 0.08],
     {"drag": {"area": [0.05, 0.08, 0.02]},
      "water": {"level": 0.0, "density": 1000},
      "chassis": {"style": "boat", "size": HULL}}, com=[0, -0.03, 0])
write("speedboat", [(0.5, 0.4, 0.1)])   # a hull slides off what it touches
