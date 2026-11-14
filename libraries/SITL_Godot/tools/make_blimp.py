#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Write models/blimp.gltf: a generic blimp, an envelope of lifting gas
over a gondola, driven by a motor pod each side and trimmed by a lift
fan and a side fan, sized for the Blimp vehicle's four motor frame.

  ./make_blimp.py

AP_FLAKE8_CLEAN
'''

import math

from gltf_model import X, Z, body, collider, part, qaxis, write

# the blimp, in metres. glTF axes: +Z forward, +X left, +Y up, and the
# origin is the gondola, which hangs below the envelope
ENVELOPE_RADIUS = 1.0
ENVELOPE_LENGTH = 8.0       # the straight section
OVERALL = ENVELOPE_LENGTH + 2 * ENVELOPE_RADIUS    # ten metres, nose to tail
ENVELOPE_Y = 1.5            # the envelope's centre, above the gondola
# the gas: the volume of the football the envelope makes, which at the
# density of air lifts its own displacement
LIFT_VOLUME = round(4.0 / 3.0 * math.pi * (0.5 * OVERALL) * ENVELOPE_RADIUS ** 2, 2)
MASS = round(1.225 * LIFT_VOLUME, 2)   # floats level, neither rising nor sinking
# a motor pod each side, level with the envelope's centre: a small
# football with a propeller no bigger than itself right behind it
POD_LENGTH = 0.9            # about a tenth of the envelope
POD_DIAMETER = 0.4          # and of the propeller
# A metre off the skin, on outriggers: the pods yaw the airship, driven
# against each other, and close in to the hull the arm is too short to
# turn 166 kg.m^2 against fins big enough to keep it pointing ahead.
POD_X = ENVELOPE_RADIUS + 1.0
POD_Z = round(-0.5 * POD_LENGTH, 4)   # the propeller sits behind the pod
POD_THRUST = 45.0           # newtons, ahead or astern
FAN_THRUST = 15.0
TAIL_Z = -0.5 * OVERALL + 0.5


def pod(servo, thrust, pitch_speed, spin):
    return {"servo": servo, "bidirectional": True, "maxThrust": thrust, "thrustExpo": 0.2,
            "torqueRatio": 0.002, "direction": spin, "timeConstant": 0.05,
            "diameter": POD_DIAMETER, "pitchSpeed": pitch_speed}


# a capsule stands on its end; turn it to lie along the hull
collider("envelope", {"type": "capsule", "capsule": {
    "height": ENVELOPE_LENGTH, "radiusTop": ENVELOPE_RADIUS, "radiusBottom": ENVELOPE_RADIUS}},
    [0, ENVELOPE_Y, 0], qaxis(X, 90))
collider("gondola", [0.6, 0.5, 1.2], [0, 0, 0])

# the gas: buoyancy in air rather than water, so gravity still acts on
# the airframe and the autopilot's accelerometers read as they should
part("envelope_lift", "floater", {"fluid": "air", "volume": LIFT_VOLUME}, [0, ENVELOPE_Y, 0])

# a motor pod each side drives it forward and turns it; both run either
# way, so the pair can yaw it on the spot
for side, sign, servo, spin in (("left", 1, 1, "ccw"), ("right", -1, 2, "cw")):
    part("pod_" + side, "rotor", pod(servo, POD_THRUST, 8.0, spin),
         [sign * POD_X, ENVELOPE_Y, POD_Z], qaxis(X, 90))

# Blimp's four-motor frame (Fins.cpp, setup_motors) wants motor 3 straight
# up and motor 4 straight out to the right: fixed, reversible propellers on
# the hull centreline, so neither pitches or yaws the airship as it pushes.
part("lift_fan", "rotor", pod(3, FAN_THRUST, 6.0, "ccw"),    # no rotation: thrust along +Y
     [0, ENVELOPE_Y + ENVELOPE_RADIUS + 0.35, 0])            # above the skin
# motor 4, at the envelope's centre: +Y turned onto -X, which is to starboard
part("side_fan", "rotor", pod(4, FAN_THRUST, 6.0, "cw"), [0, ENVELOPE_Y, 0], qaxis(Z, 90))

# tail surfaces to keep it pointing where it is going
part("tail_stabiliser", "surface", {"span": 1.8, "chord": 0.9, "cd0": 0.01, "stallAngle": 20},
     [0, ENVELOPE_Y, TAIL_Z])
part("tail_fin", "surface", {"span": 1.6, "chord": 0.9, "cd0": 0.01, "stallAngle": 20},
     [0, ENVELOPE_Y + 0.8, TAIL_Z], qaxis(Z, 90))

# a shell of gas and fabric: heavy to pitch or yaw, light to roll
PITCH_INERTIA = round(MASS * (0.25 * OVERALL ** 2 + ENVELOPE_RADIUS ** 2) / 4, 1)
body("blimp", [0, 0.3, 0], MASS,
     [PITCH_INERTIA, PITCH_INERTIA, round(2 * MASS * ENVELOPE_RADIUS ** 2 / 3, 1)],
     {"drag": {"area": [8.0, 8.0, 0.6]},
      "chassis": {"style": "blimp", "size": [2 * ENVELOPE_RADIUS, 2 * ENVELOPE_RADIUS, OVERALL],
                  "podLength": POD_LENGTH, "podDiameter": POD_DIAMETER}})
write("blimp", [(0.6, 0.5, 0.2)])   # an envelope bounces gently off what it meets
