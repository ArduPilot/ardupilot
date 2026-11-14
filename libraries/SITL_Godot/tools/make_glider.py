#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/glider.gltf: an engine-less glider sized like SITL's own
(libraries/SITL/SIM_Glider.h), so Tools/autotest/default_params/glider.parm
suits it. It has no motor, and is let go with --launcher=drop.

  ./make_glider.py

AP_FLAKE8_CLEAN
'''

import math

from gltf_model import X, Z, body, collider, part, qaxis, write

# SIM_Glider's model, which this one is built to match
MASS = 9.07441         # kg
SPAN = 1.827411        # m, tip to tip
CHORD = 0.507614       # m
AR = SPAN * SPAN / (SPAN * CHORD)   # 3.6, a stubby high-speed glider

PANEL = SPAN / 4.0     # two panels a side
DIHEDRAL = 3.0

# glTF axes: +Z forward, +X left, +Y up. The body origin is the centre of
# mass, at the wing's quarter chord.
collider("fuselage", [0.16, 0.16, 1.40], [0, 0, -0.15])
collider("tail_box", [0.55, 0.012, 0.28], [0, 0.06, -0.95])
collider("fin_box", [0.012, 0.35, 0.28], [0, 0.22, -0.95])

wing = {"span": round(PANEL, 4), "chord": round(CHORD, 4), "aspectRatio": round(AR, 3),
        "cl0": 0.1, "cm0": -0.03, "cd0": 0.015, "stallAngle": 14}
for side, sign in (("left", 1), ("right", -1)):
    for panel, x in (("inner", PANEL / 2), ("outer", PANEL * 1.5)):
        params = dict(wing)
        if panel == "outer":
            # SERVO1 above trim rolls right: left aileron trailing edge down
            params.update({"flapServo": 1, "flapAngleMin": -18 * sign, "flapAngleMax": 18 * sign,
                           "flapChordFraction": 0.22})
        y = 0.04 + x * math.tan(math.radians(DIHEDRAL))
        part("wing_%s_%s" % (side, panel), "surface", params,
             [round(sign * x, 4), round(y, 4), 0.0], qaxis(Z, sign * DIHEDRAL))
        collider("wing_box_%s_%s" % (side, panel), [round(PANEL, 4), 0.025, round(CHORD, 4)],
                 [round(sign * x, 4), round(y, 4), round(-CHORD / 4, 4)], qaxis(Z, sign * DIHEDRAL))

# tailplane at -1.5 deg incidence; SERVO2 above trim pitches up
part("tailplane", "surface",
     {"span": 0.55, "chord": 0.28, "cd0": 0.016, "stallAngle": 15, "downwash": 0.4,
      "flapServo": 2, "flapAngleMin": 18, "flapAngleMax": -18, "flapChordFraction": 0.35},
     [0, 0.06, -0.95], qaxis(X, 1.5))
# fin: span up, lift to the right; SERVO4 above trim yaws right
part("fin", "surface",
     {"span": 0.35, "chord": 0.28, "aspectRatio": 2.0, "cd0": 0.016, "stallAngle": 18,
      "flapServo": 4, "flapAngleMin": 22, "flapAngleMax": -22, "flapChordFraction": 0.35},
     [0, 0.22, -0.95], qaxis(Z, 90))

# a belly skid, not an undercarriage: this one is dropped, and arrives
# on its underside
part("skid", "wheel",
     {"radius": 0.03, "travel": 0.02, "spring": 2600, "damping": 120, "grip": 0.6,
      "rollingResistance": 0.5},
     [0, -0.10, -0.05])

# SIM_Glider's inertias are FRD: roll 0.234, pitch 1.85, yaw 2.04. Here X
# is the pitch axis, Y the yaw axis and Z the roll axis.
body("glider", [0, 0.18, 0], MASS, [1.85, 2.04, 0.234],
     {"chassis": {"style": "plane", "size": [0.16, 0.16, 1.40]},
      "drag": {"area": [0.09, 0.09, 0.014]}})
write("glider")
