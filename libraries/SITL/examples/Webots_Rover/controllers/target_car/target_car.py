#!/usr/bin/env python3
"""Moving radar target: a car-sized vehicle that drives back and forth past the
rover so the rover's LD2451-style radars have a real *moving* target to detect.

It sweeps a straight line along its own X axis, SPAWN-RELATIVE: from its start X
it drives TRAVEL metres in -X, then back, forever. Being spawn-relative, the same
controller works in any world regardless of where the rover sits - just place the
target's spawn so the rover falls inside the [x0-TRAVEL, x0] sweep, offset a few
metres to one side so it never collides.

Because it approaches then recedes, it produces a strong radial (Doppler) speed -
which the radar's minAbsoluteRadialSpeed filter requires. A stationary or purely
side-passing target would be rejected, exactly as a real vehicle radar rejects
static clutter.
"""
from controller import Robot

robot = Robot()
ts = int(robot.getBasicTimeStep())

wheels = [robot.getDevice(n) for n in ("tw_fl", "tw_fr", "tw_rl", "tw_rr")]
for w in wheels:
    w.setPosition(float("inf"))
    w.setVelocity(0.0)

gps = robot.getDevice("tgps")
gps.enable(ts)

WHEEL_SPEED = 12.0   # rad/s -> ~3.6 m/s at r=0.30
TRAVEL = 50.0        # metres to drive away from spawn before turning back
sx = sy = None       # spawn position, captured on the first step
direction = -1       # start driving backwards (body -X) away from spawn
returning = False

# Distance-based (not axis-based) so the same controller works whatever the
# target's heading is: it drives TRAVEL m away from spawn, returns, and repeats.
# Orient the target (its rotation field) along the rover's heading so the sweep
# passes through the front and rear radar arcs.
while robot.step(ts) != -1:
    v = gps.getValues()
    if sx is None:
        sx, sy = v[0], v[1]
    d = ((v[0] - sx) ** 2 + (v[1] - sy) ** 2) ** 0.5
    if not returning and d >= TRAVEL:
        direction, returning = 1, True
    elif returning and d <= 1.0:
        direction, returning = -1, False
    for w in wheels:
        w.setVelocity(direction * WHEEL_SPEED)
