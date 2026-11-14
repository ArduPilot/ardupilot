#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/tracker.gltf: an antenna tracker in three bodies, a heavy base,
a post which turns on it (SERVO1, yaw) and a flat panel dish which tilts on
the post (SERVO2, pitch). The dish is the vehicle: its attitude is the aim.

  ./make_tracker.py

AP_FLAKE8_CLEAN
'''

from gltf_model import X, Z, add, body, box_inertia, collider, qaxis, write

# The yaw servo is continuous, with no end stop, so the dish can follow
# something round and round: SERVO_YAW_TYPE 2, which tracker.parm sets.
YAW_RATE = 120.0        # deg/s flat out
# The tilt is not SIM_Tracker's forty-five either way: the panel swings
# from upright, looking at the horizon, to flat on its back looking
# straight up, and is set up leaning back part of the way.
PITCH_MIN = 0.0         # deg above the horizon
PITCH_MAX = 90.0
PITCH_START = 30.0
PITCH_RATE = 30.0       # deg/s

BASE_SIZE = [0.30, 0.06, 0.30]      # m
BASE_MASS = 20.0                    # kg, so that it stays put
POST_HEIGHT = 0.50                  # m
POST_THICK = 0.05
POST_MASS = 0.5
DISH_SIZE = [0.40, 0.40, 0.04]      # m, a flat panel
DISH_MASS = 0.5
FEED_LENGTH = 0.25                  # m, the boom which shows where it points
TORQUE = 5.0                        # N.m

BASE_COLOUR = [0.16, 0.17, 0.20]
POST_COLOUR = [0.55, 0.57, 0.60]
DISH_COLOUR = [0.90, 0.90, 0.92]


def servo(name, at, rotation, joined_to, number, lo, hi, rate, continuous=False, built=None):
    '''the joint to the body below, and the hinge which drives it'''
    hinge = {"servo": number, "angleMin": lo, "angleMax": hi,
             "maxRate": rate, "maxTorque": TORQUE}
    if continuous:
        hinge["continuous"] = True
    if built is not None:
        hinge["builtAngle"] = built
    n = {"name": name, "translation": [round(v, 4) for v in at],
         "extensions": {
             "KHR_physics_rigid_bodies": {"joint": {"connectedNode": joined_to, "joint": 0}},
             "ARDUPILOT_sitl": {"hinge": hinge}}}
    if rotation:
        n["rotation"] = rotation
    return add(n)


def solid(name, at, mass, size, kids, colour, index=None, rotation=None):
    return body(name, [round(v, 4) for v in at], mass, box_inertia(mass, size),
                {"chassis": {"style": "solid", "colour": colour}},
                children=kids, rotation=rotation, index=index)


# glTF axes: +Z forward, +X left, +Y up. The dish is node 0, the vehicle.
pivot_height = BASE_SIZE[1] + POST_HEIGHT

base = solid("base", [0, 0.5 * BASE_SIZE[1], 0], BASE_MASS, BASE_SIZE,
             [collider("base_box", BASE_SIZE, [0, 0, 0])], BASE_COLOUR)

# The post turns on the base about the upright axis: a hinge turns about
# its own X, which a quarter turn about Z stands up. ArduPilot drives a
# continuous yaw servo below mid travel to go clockwise seen from above.
post_size = [POST_THICK, POST_HEIGHT, POST_THICK]
post = solid("post", [0, BASE_SIZE[1] + 0.5 * POST_HEIGHT, 0], POST_MASS, post_size,
             [collider("post_box", post_size, [0, 0, 0]),
              servo("yaw", [0, -0.5 * POST_HEIGHT, 0], qaxis(Z, 90), base,
                    1, -1.0, 1.0, YAW_RATE, continuous=True)],
             POST_COLOUR)

# The dish tilts about its own X on the top of the post. A turn about +X
# puts the nose down, so more PWM is the negative angle, and the dish is
# built already tilted back. It hangs on its own centre of mass.
solid("tracker", [0, pivot_height, 0], DISH_MASS, DISH_SIZE,
      [collider("dish_panel", DISH_SIZE, [0, 0, 0]),
       collider("dish_feed", [0.02, 0.02, FEED_LENGTH], [0, 0, 0.5 * FEED_LENGTH]),
       servo("pitch", [0, 0, 0], None, post, 2, -PITCH_MIN, -PITCH_MAX, PITCH_RATE,
             built=-PITCH_START)],
      DISH_COLOUR, index=0, rotation=qaxis(X, -PITCH_START))

write("tracker", [(1.0, 0.9, 0.0)], joints=[{}], roots=[0, post, base])
