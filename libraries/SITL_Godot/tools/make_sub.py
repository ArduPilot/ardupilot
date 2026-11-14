#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/sub.gltf: a small ROV sized like SITL's own submarine, so that
Tools/autotest/default_params/sub.parm suits it. The frame is ArduSub's
SUB_FRAME_VECTORED: four flat thrusters at 45 degrees and two upright ones.

  ./make_sub.py

AP_FLAKE8_CLEAN
'''

from gltf_model import X, Y, body, collider, part, qaxis, qmul, write

# SIM_Submarine's hull, which this one is built to match
MASS = 10.5            # kg
WIDTH = 0.338          # m, glTF X
HEIGHT = 0.254         # m, glTF Y
LENGTH = 0.457         # m, glTF Z

# Fresh water, the backend's default. Trimmed half a newton light, as a
# real ROV is, so that it comes home by itself if it loses power.
WATER_DENSITY = 1000.0
VOLUME = round(MASS / WATER_DENSITY + 0.00005, 6)

# The buoyancy acts this far above the centre of mass.  That lever is
# the whole of the sub's pitch and roll stiffness, since no thruster
# drives pitch: tilt it and the couple rights it.
BUOYANCY_RISE = 0.04

THRUST = 40.0          # newtons, either way: a T200 class thruster
TIME_CONSTANT = 0.15   # a thruster takes a moment to spin its column of water up

HALF_W = 0.14          # thruster box, narrower than the hull
HALF_L = 0.17
VERT_X = 0.12


def thruster(servo, spin):
    return {"servo": servo, "maxThrust": THRUST, "bidirectional": True,
            "thrustExpo": 0.2, "timeConstant": TIME_CONSTANT,
            "torqueRatio": 0.01, "diameter": 0.076, "direction": spin}


# glTF axes: +Z forward, +X left, +Y up.  Starboard is -X and aft is -Z.
collider("hull", [WIDTH, HEIGHT, LENGTH], [0, 0, 0])

# The four flat thrusters: the first turn aims the rotor's +Y thrust axis
# forward, the second swings it to its corner. Servos 1 and 2 at the front
# thrust aft and outboard, 3 and 4 at the rear forward and outboard.
for servo, px, pz, swing, spin in (
        (1, -HALF_W, HALF_L, -135, "ccw"),
        (2, HALF_W, HALF_L, 135, "cw"),
        (3, -HALF_W, -HALF_L, -45, "cw"),
        (4, HALF_W, -HALF_L, 45, "ccw")):
    part("thruster_%d" % servo, "rotor", thruster(servo, spin),
         [round(px, 4), 0.0, round(pz, 4)], qmul(qaxis(X, 90), qaxis(Y, swing)))

# The two upright thrusters, which heave and roll, point down: ArduSub
# drives both with a negative throttle factor, so full output pushes the
# sub down. High up beside the buoyancy, as on a BlueROV2.
for servo, px, spin in ((5, -VERT_X, "ccw"), (6, VERT_X, "cw")):
    part("thruster_%d" % servo, "rotor", thruster(servo, spin),
         [round(px, 4), round(0.30 * HEIGHT, 4), 0.0], qaxis(X, 180))

# Buoyancy, above the centre of mass so that the hull hangs upright
part("hull_buoyancy", "floater", {"fluid": "water", "volume": VOLUME, "height": HEIGHT},
     [0, BUOYANCY_RISE, 0])

# Drag, level with the centre of mass: above it, driving forward would pitch
# up a frame with no pitch authority. Spread over four points so that it
# resists turning too. Each area is a quarter of the hull's, times a blunt Cd.
DRAG_AREA = [round(HEIGHT * LENGTH * 0.9 / 4, 5),
             round(WIDTH * LENGTH * 1.1 / 4, 5),
             round(WIDTH * HEIGHT * 0.9 / 4, 5)]
for name, px, pz in (("fore", 0.0, 0.4 * LENGTH), ("aft", 0.0, -0.4 * LENGTH),
                     ("port", 0.4 * WIDTH, 0.0), ("stbd", -0.4 * WIDTH, 0.0)):
    part("hull_drag_" + name, "floater",
         {"fluid": "water", "volume": 0.0, "height": HEIGHT, "dragArea": DRAG_AREA, "damping": 3.0},
         [round(px, 4), 0.0, round(pz, 4)])

# Box inertia about each axis, near enough doubled: a hull dragging this
# much water with it turns as though it were twice as heavy.
IXX = MASS / 12.0 * (HEIGHT ** 2 + LENGTH ** 2)
IYY = MASS / 12.0 * (WIDTH ** 2 + LENGTH ** 2)
IZZ = MASS / 12.0 * (WIDTH ** 2 + HEIGHT ** 2)

body("sub", [0, 0, 0], MASS, [round(IXX * 2, 3), round(IYY * 2, 3), round(IZZ * 2, 3)],
     {"chassis": {"style": "sub", "size": [WIDTH, HEIGHT, LENGTH]},
      "water": {"level": 0.0, "density": WATER_DENSITY}})
write("sub", [(0.5, 0.4, 0.1)])
