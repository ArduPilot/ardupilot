#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/sailboat.gltf: a yacht which sails, on four buoyancy points,
with no motor: a rudder servo and a sheet servo. The mast is a hinge in
sheet mode: the servo sets how far the sail may swing, the wind which side.

  ./make_sailboat.py

AP_FLAKE8_CLEAN
'''

from gltf_model import Z, body, collider, part, qaxis, write

MASS = 4.0                  # kg
LENGTH = 1.20               # m, glTF Z
BEAM = 0.30                 # m, glTF X
DEPTH = 0.16                # m, glTF Y

# Four of these float the hull.  Buoyancy ramps over the floater's height,
# so a hull which displaces its own weight at about half draught sits on
# the surface rather than bobbing like a cork or wallowing under.
FLOATER_VOLUME = 0.0022
FLOATER_HEIGHT = 0.20

SAIL_HEIGHT = 1.00          # m up the mast
SAIL_CHORD = 0.42           # m
MAST_Z = 0.18               # m forward of the centre

KEEL_DEPTH = 0.30           # m below the hull
KEEL_CHORD = 0.26
# The keel is under the rig, forward of the middle, to hold the bow and
# leave the stern free to swing down wind: amidships, the hull rounds up
# into the wind and stays there with no flow over the rudder.
KEEL_Z = 0.22

RUDDER_SPAN = 0.18
RUDDER_CHORD = 0.09
RUDDER_Z = -0.52
STEER_ANGLE = 35.0          # SIM_Sailboat's steering_angle_max

# How far the sail may swing. ArduPilot's mainsail output is sheet let
# out: 0 is hauled in hard, and full travel is SAIL_ANGLE_MAX, ninety
# degrees. A sheet that runs out short of that leaves the sail stalled.
SHEET_OUT = 90.0
SHEET_IN = 0.0

# glTF axes: +Z forward, +X left, +Y up.
collider("hull", [BEAM, DEPTH, LENGTH], [0, 0, 0])

# Buoyancy at four corners, so that the water resists the hull turning
# as well as moving. The linear damping is small along the hull: about
# 5 N of sail must drive it, and a hull's resistance is mostly quadratic.
for end, pz in (("fore", 0.36 * LENGTH), ("aft", -0.36 * LENGTH)):
    for side, px in (("port", 0.3 * BEAM), ("stbd", -0.3 * BEAM)):
        part("floater_%s_%s" % (end, side), "floater",
             {"volume": FLOATER_VOLUME, "height": FLOATER_HEIGHT,
              "dragArea": [0.030, 0.006, 0.0003], "damping": 0.1},
             [round(px, 4), 0.0, round(pz, 4)])

# A lifting surface has its span along local X and its chord along local
# Z, and lifts towards local +Y. A quarter turn about Z stands the span
# up, for a sail or a keel; a hinge turns about local X, so it stands too.
UPRIGHT = qaxis(Z, 90)

# The sail in panels up the mast, each narrower than the one below, so
# that it stalls from the head down and not all at once. They hang under
# the mast's hinge, whose own X is what points at the sky.
SAIL_PANELS = 4
HEAD_CHORD = 0.22 * SAIL_CHORD
panels = []
for i in range(SAIL_PANELS):
    up = (i + 0.5) / SAIL_PANELS
    chord = SAIL_CHORD + (HEAD_CHORD - SAIL_CHORD) * up
    # the panel's quarter chord moves forward as it narrows, so the leech stays straight
    panels.append(part("mainsail_%d" % (i + 1), "surface",
                       {"span": round(SAIL_HEIGHT / SAIL_PANELS, 4),
                        "chord": round(chord, 4),
                        "aspectRatio": round(SAIL_HEIGHT / SAIL_CHORD, 2),
                        "cl0": 0.0, "cd0": 0.02, "stallAngle": 25, "oswald": 0.8},
                       [round(up * SAIL_HEIGHT, 4), 0, round(0.25 * (SAIL_CHORD - chord), 4)]))
# A telltale at the masthead, streaming with the apparent wind to show
# which tack the boat is on. It carries no force.
part("telltale", "streamer", {"length": 0.66, "width": 0.045, "whip": 0.5},
     [0, round(0.5 * DEPTH + SAIL_HEIGHT, 4), MAST_Z])

# SERVO4 is the mainsail sheet (k_mainsail_sheet)
part("mast", "hinge",
     {"servo": 4, "angleMin": SHEET_IN, "angleMax": SHEET_OUT, "maxRate": 60.0, "sheet": True},
     [0, round(0.5 * DEPTH, 4), MAST_Z], UPRIGHT, panels)

# The keel, standing down under the hull.  It is what stops the boat
# being blown sideways, and its weight is what keeps the mast up.
part("keel", "surface",
     {"span": KEEL_DEPTH, "chord": KEEL_CHORD, "fluid": "water",
      "aspectRatio": round(KEEL_DEPTH / KEEL_CHORD, 2),
      "cl0": 0.0, "cd0": 0.015, "stallAngle": 20},
     [0, round(-0.5 * DEPTH - 0.5 * KEEL_DEPTH, 4), KEEL_Z], UPRIGHT)
collider("ballast", [0.05, KEEL_DEPTH, KEEL_CHORD],
         [0, round(-0.5 * DEPTH - 0.5 * KEEL_DEPTH, 4), KEEL_Z])

# The rudder on SERVO1, the steering output. The blade hangs down from
# the pivot, along the hinge's own X.
blade = part("rudder_blade", "surface",
             {"span": RUDDER_SPAN, "chord": RUDDER_CHORD, "fluid": "water",
              "aspectRatio": round(RUDDER_SPAN / RUDDER_CHORD, 2),
              "cd0": 0.015, "stallAngle": 22},
             [round(-0.5 * RUDDER_SPAN, 4), 0, 0])
part("rudder", "hinge",
     {"servo": 1, "angleMin": -STEER_ANGLE, "angleMax": STEER_ANGLE, "maxRate": 180.0},
     [0, round(-0.5 * DEPTH, 4), RUDDER_Z], UPRIGHT, [blade])

IXX = MASS / 12.0 * (DEPTH ** 2 + LENGTH ** 2)
IYY = MASS / 12.0 * (BEAM ** 2 + LENGTH ** 2)
IZZ = MASS / 12.0 * (BEAM ** 2 + DEPTH ** 2)

# The centre of mass is near the bottom of the keel, where the lead is:
# higher, the sail lays the boat over past fifty degrees. It is not moved
# forward over the keel, which would leave the rudder as the only fin.
body("sailboat", [0, 0.05, 0], MASS, [round(IXX, 4), round(IYY, 4), round(IZZ, 4)],
     {"chassis": {"style": "boat", "size": [BEAM, DEPTH, LENGTH]},
      "water": {"level": 0.0}},
     com=[0, round(-0.5 * DEPTH - 0.85 * KEEL_DEPTH, 4), 0])
write("sailboat", [(0.4, 0.3, 0.1)])
