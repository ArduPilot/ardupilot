#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write the walking robots: models/hexapod.gltf, quadruped.gltf and
three-legged-bot.gltf, each with its gait in a .lua and its parameters in
a .parm. Every leg segment is a rigid body, joined on by a hinged joint.

  ./make_hexapod.py

AP_FLAKE8_CLEAN
'''

import math
import os

from gltf_model import MODELS, Y, Z, add, body, box_inertia, collider, qaxis, qmul, write

# ---------------------------------------------------------------- settings
# what to write: (name, legs, servos a leg). Legs are 3, 4 or 6; servos are
# 2 (swing, lift) or 3 (+ knee), numbered clockwise from the front left leg
ROBOTS = (("hexapod", 6, 3),
          ("quadruped", 4, 3),
          ("three-legged-bot", 3, 3))

BODY_RADIUS = 0.11      # m, centre to a corner, where a leg is hung
BODY_HEIGHT = 0.05      # m
BODY_MASS = 1.0         # kg

SEGMENT = 0.08          # m, every leg segment: hip, thigh and shin alike
# With three servos the thigh slopes down from the hip to the knee and
# the shin goes on down from there, so the leg is bent at rest and not
# laid out straight: a joint already bent has travel both ways.
THIGH_RISE = -20.0      # deg above level; negative slopes down
LINK_MASS = 0.05        # kg, each leg segment
LINK_THICK = 0.022      # m

SWING = 40.0            # deg either way, fore and aft
LIFT = 40.0             # deg either way
KNEE = 40.0             # deg either way
TORQUE = 1.5            # N.m, a standard size hobby servo
RATE = 500.0            # deg/s
# --------------------------------------------------------------------------

BODY_COLOUR = [0.16, 0.17, 0.20]
THIGH_COLOUR = [0.95, 0.55, 0.10]
SHIN_COLOUR = [0.90, 0.90, 0.92]
COXA_COLOUR = [0.75, 0.10, 0.10]

# a hinge turns about its own X: stand that up for a joint which swings
# a leg fore and aft, and lay it along the body for one which lifts it
UP_AXIS = qaxis(Z, 90)
FORE_AXIS = qaxis(Y, -90)

roots = []
# how high the body rides, which is set for each robot as it is built
RIDE = 0.0


def r4(v):
    return [round(x, 4) for x in v]


def turn(phi, v):
    '''v turned phi degrees about the upright axis'''
    c, n = math.cos(math.radians(phi)), math.sin(math.radians(phi))
    return [c * v[0] + n * v[2], v[1], -n * v[0] + c * v[2]]


def link(name, phi, start, rise, length, axis, joined_to, servo, lo, hi, foot=False):
    '''a leg segment: a strut from start, running rise degrees above level
    for length, jointed and hinged at its inboard end to joined_to. Returns
    the body and where the strut ends. Set out in the +X, +Y plane, then
    turned phi to the corner it hangs from.'''
    c, n = math.cos(math.radians(rise)), math.sin(math.radians(rise))
    end = [start[0] + length * c, start[1] + length * n, 0.0]
    centre = [0.5 * (start[0] + end[0]), 0.5 * (start[1] + end[1]), 0.0]
    # the strut lies along the body's own X, so that is where its ends are
    size = [length, LINK_THICK, LINK_THICK]
    kids = [collider(name + "_box", size, [0, 0, 0])]
    if foot:
        kids.append(collider(name + "_foot", [0.016, 0.03, 0.03],
                             [0.5 * length, 0, 0], material=1))
    kids.append(add({
        "name": name + "_servo",
        "translation": [round(-0.5 * length, 4), 0, 0],
        # the hinge is set square to the leg, not to the tipped strut
        "rotation": qmul(axis, qaxis(Z, -rise)),
        "extensions": {
            "KHR_physics_rigid_bodies": {"joint": {"connectedNode": joined_to, "joint": 0}},
            "ARDUPILOT_sitl": {"hinge": {"servo": servo, "angleMin": lo, "angleMax": hi,
                                         "maxRate": RATE, "maxTorque": TORQUE}},
        }}))
    at = turn(phi, centre)
    idx = add({
        "name": name,
        "translation": [round(at[0], 4), round(at[1] + RIDE, 4), round(at[2], 4)],
        "rotation": qmul(qaxis(Z, rise), qaxis(Y, phi)),
        "children": kids,
        "extensions": {
            "KHR_physics_rigid_bodies": {"motion": {
                "mass": LINK_MASS, "inertiaDiagonal": box_inertia(LINK_MASS, size, 1e-4)}},
            # drawn as what it is, and in a colour that shows against
            # the body and the ground: a gait is watched at the legs
            "ARDUPILOT_sitl": {"chassis": {"style": "solid",
                                           "colour": (SHIN_COLOUR if foot else
                                                      COXA_COLOUR if name.endswith("coxa") else THIGH_COLOUR)}}},
    })
    roots.append(idx)
    return idx, end


# The gait has to know how many legs there are, how many servos each
# has and where each stands, so it is written beside them.
LUA = """-- Copyright (C) 2026 David Buzz
-- SPDX-License-Identifier: GPL-3.0-or-later
-- Part of ArduPilot and released under its licence: see COPYING.txt.

-- Walk models/%(name)s.gltf. Written by tools/make_hexapod.py: change the
-- legs there and write both again, not here. It walks at the throttle
-- ArduPilot asks for, and steers by a longer stride on one side.
local LEGS = %(legs)d
local JOINTS = %(joints)d
-- which side each leg is on: 1 left, -1 right, 0 on the centreline
local SIDE = {%(sides)s}
-- and for a leg on the centreline, which cannot swing fore and aft,
-- whether it is ahead of the body or behind it: 1 or -1
local AHEAD = {%(ahead)s}
-- when in the stride each leg's turn comes, as a fraction of it
local TURN = {%(turns)s}
-- how much of the stride a foot spends on the ground
local STANCE = %(stance).2f
local SWING_DEG = 32        -- reach fore and aft at full throttle
local REACH_DEG = 18        -- and in and out, for a leg on the centreline
local LIFT_DEG = %(lift)d         -- how far a foot is picked up
local RANGE_DEG = %(range).1f    -- servo travel either side of centre
local KNEE_FOLD = 1.2       -- knee bend for each degree of lift
local STEPS_HZ = %(steps).1f        -- strides a second at full throttle
local PERIOD_MS = 20

local THROTTLE = 3
local STEERING = 4
local phase = 0.0

local function pwm(deg)
    local v = 1500 + deg / RANGE_DEG * 500
    return math.floor(math.max(1000, math.min(2000, v)))
end

function update()
    local thr = vehicle:get_control_output(THROTTLE) or 0
    local steer = vehicle:get_control_output(STEERING) or 0
    if not arming:is_armed() then
        thr = 0
        steer = 0
    end
    local drive = math.max(math.abs(thr), math.abs(steer))
    phase = (phase + STEPS_HZ * math.min(drive, 1.0) * PERIOD_MS * 0.001) %% 1.0
    for leg = 0, LEGS - 1 do
        -- where this leg is in its own stride: on the ground and
        -- sweeping back from 1 to -1, then in the air and coming
        -- forward again
        local u = (phase + TURN[leg + 1]) %% 1.0
        local reach
        local lift = 0
        if u < STANCE then
            reach = 1.0 - 2.0 * u / STANCE
        else
            local s = (u - STANCE) / (1.0 - STANCE)
            reach = 2.0 * s - 1.0
            if drive > 0.03 then
                lift = math.sin(math.pi * s) * LIFT_DEG
            end
        end
        -- steering to the right is a longer stride on the left
        local side = SIDE[leg + 1]
        local stride = thr + side * steer
        stride = math.max(-1.0, math.min(1.0, stride))
        local swing = 0
        local knee = lift * KNEE_FOLD
        if side ~= 0 then
            swing = reach * SWING_DEG * stride
        elseif JOINTS == 3 then
            -- a leg on the centreline pushes by reaching out and
            -- drawing in, which is the knee's work
            knee = knee + reach * REACH_DEG * thr * AHEAD[leg + 1]
        end
        local chan = leg * JOINTS
        SRV_Channels:set_output_pwm_chan_timeout(chan, pwm(swing), 1000)
        SRV_Channels:set_output_pwm_chan_timeout(chan + 1, pwm(lift), 1000)
        if JOINTS == 3 then
            SRV_Channels:set_output_pwm_chan_timeout(chan + 2, pwm(knee), 1000)
        end
    end
    return update, PERIOD_MS
end

return update()
"""


def build(name, legs, joints):
    """write one robot: its model, its gait and its parameters"""
    global RIDE
    roots[:] = [0]
    # the shin hangs straight down at rest, from a knee which with three
    # servos is off the hip's level by the thigh's rise
    stand = SEGMENT * (1.0 - math.sin(math.radians(THIGH_RISE))) if joints == 3 else SEGMENT
    RIDE = stand + 0.5 * LINK_THICK

    # Six legs and three hang from the corners of a hexagon, four from the
    # corners of a square; a hexagon is three slabs laid across each other.
    if legs == 4:
        side = BODY_RADIUS * math.sqrt(2.0)
        body_kids = [collider("body_box", r4([side, BODY_HEIGHT, side]), [0, 0, 0])]
        # heading of each corner, clockwise from the front left
        headings = [-45.0, 45.0, 135.0, -135.0]
    else:
        slab = r4([BODY_RADIUS, BODY_HEIGHT, BODY_RADIUS * math.sqrt(3.0)])
        body_kids = [collider("body_slab%d" % (n + 1), slab, [0, 0, 0], qaxis(Y, a))
                     for n, a in enumerate((0.0, 60.0, 120.0))]
        headings = [-30.0, 30.0, 90.0, 150.0, -150.0, -90.0]
        if legs == 3:
            headings = [-60.0, 60.0, 180.0]

    servo = 1
    sides = []
    ahead = []
    for leg, heading in enumerate(headings):
        # a corner at this heading from dead ahead, positive to the
        # right; a leg is built pointing along +X, which is to the left,
        # so it is turned a quarter turn less than its heading
        phi = -heading - 90.0
        leg_name = "leg%d" % (leg + 1)
        # More PWM always means the foot further forward, and higher: a
        # turn about the upright axis is forwards on one side of the body
        # and backwards on the other, so the swing's sign goes by side.
        right = math.sin(math.radians(heading))
        sides.append(0 if abs(right) < 0.1 else (-1 if right > 0 else 1))
        ahead.append(1 if math.cos(math.radians(heading)) > 0 else -1)
        swing = (-SWING, SWING) if right > 0 else (SWING, -SWING)

        parent = 0
        start = [BODY_RADIUS, 0.0, 0.0]
        if joints == 3:
            parent, start = link(leg_name + "_coxa", phi, start, 0.0, SEGMENT, UP_AXIS, parent,
                                 servo, swing[0], swing[1])
            parent, start = link(leg_name + "_thigh", phi, start, THIGH_RISE, SEGMENT, FORE_AXIS, parent,
                                 servo + 1, -LIFT, LIFT)
            # straight down from the knee to the ground
            link(leg_name + "_shin", phi, start, -90.0, SEGMENT, FORE_AXIS, parent,
                 servo + 2, -KNEE, KNEE, foot=True)
        else:
            parent, start = link(leg_name + "_thigh", phi, start, 0.0, SEGMENT, UP_AXIS, parent,
                                 servo, swing[0], swing[1])
            link(leg_name + "_shin", phi, start, -90.0, SEGMENT, FORE_AXIS, parent,
                 servo + 1, -LIFT, LIFT, foot=True)
        servo += joints

    body_size = [2 * BODY_RADIUS, BODY_HEIGHT, 2 * BODY_RADIUS]
    body(name, [0, round(RIDE, 4), 0], BODY_MASS, box_inertia(BODY_MASS, body_size, 1e-4),
         {"chassis": {"style": "solid", "colour": BODY_COLOUR}}, children=body_kids)
    # the second material is the feet: rubber, the only reason it goes anywhere
    write(name, [(0.6, 0.5, 0.05), (1.6, 1.3, 0.0)], joints=[{"limits": []}], roots=roots)

    # Six legs go three and three, every other one round the body. Fewer go
    # one at a time, across the body from the one before. The fewer feet
    # are left standing, the less the lifted one is picked up, or it tips.
    if legs == 6:
        turns = [0.5 * (n % 2) for n in range(legs)]
        stance, steps, lift_deg = 0.5, 2.0, 28
    elif legs == 4:
        turns = [0.0, 0.5, 0.25, 0.75]
        stance, steps, lift_deg = 0.75, 1.5, 22
    else:
        turns = [n / 3.0 for n in range(legs)]
        stance, steps, lift_deg = 0.9, 1.2, 10
    with open(os.path.join(MODELS, name + ".lua"), "w") as f:
        f.write(LUA % {"name": name, "legs": legs, "joints": joints, "range": SWING,
                       "sides": ", ".join(str(v) for v in sides),
                       "ahead": ", ".join(str(v) for v in ahead),
                       "turns": ", ".join("%.3f" % v for v in turns),
                       "stance": stance, "steps": steps, "lift": lift_deg})

    # Every leg servo is given to scripting, of which there are sixteen
    # outputs; the gait drives outputs by number whatever they are for, so
    # the next two are the steering and throttle Rover will not arm without.
    servos = legs * joints
    scripted = min(servos, 16)
    lines = ["# Parameters for models/%s.gltf, applied on top of Rover's. Written" % name,
             "# by tools/make_hexapod.py: change the legs there, not here.",
             "# models/%s.lua turns Rover's steering and throttle into the leg servos." % name,
             "SCR_ENABLE      1",
             "SCR_HEAP_SIZE   200000",
             ""]
    if scripted + 2 > 16:
        lines += ["SERVO_32_ENABLE 1", ""]
    lines.append("# the legs: %d servos, %d legs of %d" % (servos, legs, joints))
    for n in range(scripted):
        lines.append("SERVO%d_FUNCTION %d" % (n + 1, 94 + n))
    lines += ["",
              "# Steering and throttle outputs, which Rover will not arm without.",
              "# Nothing is plugged into them: the gait reads what ArduPilot would",
              "# have sent.",
              "SERVO%d_FUNCTION 26" % (scripted + 1),
              "SERVO%d_FUNCTION 70" % (scripted + 2),
              "",
              "# it walks at a few tenths of a metre a second, not at a car's pace",
              "CRUISE_SPEED    0.25",
              "CRUISE_THROTTLE 60",
              "WP_SPEED        0.25",
              "WP_RADIUS       0.5",
              "",
              "SIM_RATE_HZ     400",
              ""]
    with open(os.path.join(MODELS, name + ".parm"), "w") as f:
        f.write("\n".join(lines))


for robot in ROBOTS:
    build(*robot)
