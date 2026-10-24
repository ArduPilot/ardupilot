# Quadcopter calibration: how the numbers were calculated

This is the quadcopter counterpart to `TRICOPTER_TUNING.md`, worked out the
same way, for `webots_quadX.wbt` (the reference X-frame world; `webots_quadPlus.wbt`,
`webots_two_quadX.wbt`, and the `pyramidMap*.wbt` variants share the same
per-arm mass and `tests/migrate_worlds.py` calibration, just a different
arm layout / vehicle count). Unlike the tricopter, this airframe was already
correctly calibrated in the repository -- this document derives *why* those
numbers are what they are and shows the geometry is worth double-checking
numerically rather than by eye, since one wrinkle here (below) is easy to get
wrong by inspection.

## 1. Mass budget

Unlike the tricopter (which puts its main body mass on a child `Solid`), the
quad puts the body mass directly on the `Robot` node's own `Physics` block,
alongside a `boundingObject Box { size 0.1 0.1 0.1 }` on the Robot itself --
which is exactly the thing the tricopter's Robot node was missing (see
`TRICOPTER_TUNING.md` section 2). That's why the quad never had the
identity-inertia bug: the Robot node always had geometry for Webots to derive
a real inertia tensor from.

| Component | Mass (kg) |
|---|---|
| Robot's own `Physics` (the body) | 1.00 |
| 4x arm nacelle (`Solid`, 0.05 kg each) | 0.20 |
| **Total** | **1.20** |

Matches `tests/migrate_worlds.py`'s reported `vehicle mass 1.200 kg (summed
from the world)` for this file.

## 2. Inertia -- and a geometry trap

Each arm is built as:

```text
Pose { translation <axis offset, 0.2 m>  rotation <Q>
  children [
    Solid { translation 0 0.2 0   # <-- a SECOND, local offset
      children [ Propeller {...} ]
      physics Physics { mass 0.05  inertiaMatrix [3.2e-05 3.2e-05 3.2e-05  0 0 0] }
    }
  ]
}
```

Reading only the outer `Pose`'s translation (`+-0.2` on X or Z) suggests a
0.2 m arm radius. That is wrong: the inner `Solid` has its *own* local offset
`0 0.2 0`, which gets rotated by the Pose's quaternion `Q` before being added
to the Pose's translation. For all four arms, `Q` happens to map that local
`+Y` offset onto the *same* axis as the outer translation, so the two offsets
add instead of being perpendicular. Verified numerically (Rodrigues' rotation
formula, not by eye):

```python
import numpy as np
def rot(axis, angle):
    axis = np.array(axis)/np.linalg.norm(axis)
    c, s = np.cos(angle), np.sin(angle)
    x, y, z = axis
    K = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3) + s*K + (1-c)*(K@K)

# one arm: Pose translation (-0.2,0,0), rotation axis/angle, Solid translation (0,0.2,0)
R = rot((-0.5773502691896258, 0.5773502691896258, 0.5773502691896258), 2.094395)
world = np.array((-0.2, 0, 0)) + R @ np.array((0, 0.2, 0))
# -> [-0.4, 0, 0]: radius 0.4 m, not 0.2 m
```

All four arms land at **radius 0.4 m** from the center, on the X and Z axes
(a "+" position layout -- `FRAME_TYPE 1` (X) refers to the motor-mixing
convention ArduPilot uses, not necessarily the visual strut position in the
world). This matters because inertia scales with the *square* of that radius
-- getting it wrong by 2x here would have meant a 4x error in the arms'
contribution.

Per-axis inertia, nacelles as `I = 0.4 * m * r_nacelle^2` (same sphere
convention as the tricopter, `r_nacelle = 0.04 m`) for the nacelle's own spin,
plus `m * r_arm^2` parallel-axis for its distance from the CG:

| Axis | Arms contributing (parallel-axis) | Arms' own spin | Body (0.1^3 box, `I=m*s^2/6`) | Total |
|---|---|---|---|---|
| Roll (X) | 2 arms at z=+-0.4: `2 * 0.05 * 0.4^2 = 0.016` | `4 * 3.2e-05 = 1.28e-4` | 0.001667 | **~0.0178 kg m^2** |
| Pitch (Z) | 2 arms at x=+-0.4: `2 * 0.05 * 0.4^2 = 0.016` | 1.28e-4 | 0.001667 | **~0.0178 kg m^2** |
| Yaw (Y) | all 4 arms, r=0.4: `4 * 0.05 * 0.4^2 = 0.032` | 1.28e-4 | 0.001667 | **~0.0338 kg m^2** |

(All four arms sit at the same 0.4 m radius, so every arm contributes to yaw,
but only the two arms on a given axis's perpendicular plane contribute to
roll or pitch.) These are sanity-check totals; Webots derives the real numbers
from geometry -- but they are the reason the quad, with 4 arms at 0.4 m and
0.05 kg each, ends up with noticeably *more* rotational inertia than the
tricopter's ~0.02 kg m^2 (2 arms at ~0.3 m plus a lighter tail at 0.34 m):
more mass, further out.

## 3. Rotor thrust/torque constants

Same formula as the tricopter (`tests/migrate_worlds.py calibrate`), with this
world's values: 4 rotors, thrust-to-weight 2.2, torque ratio 0.02.

```text
thrust_per_rotor = tw * mass * g / rotors
                  = 2.2 * 1.2 * 9.80665 / 4
                  = 6.4724 N

kt (thrustConstants) = thrust_per_rotor / omega_max^2
                      = 6.4724 / 400^2
                      = 4.0452e-05

kq (torqueConstants) = kt * torque_ratio
                      = 4.0452e-05 * 0.02
                      = 8.0905e-07

hover_throttle = 1 / tw = 1 / 2.2 = 0.4545  (~45%)
```

Matches the values already committed in `webots_quadX.wbt`
(`4.04524e-05` / `8.09049e-07`) and reported by `migrate_worlds.py` as
`unchanged` -- this world is already at its calibrated values.

`torque_ratio = 0.02` here (vs. the tricopter's near-zero `8.74e-7`) is
deliberate and physical: `migrate_worlds.py`'s own comment notes a real
propeller's torque/thrust ratio is "nearer 0.02 m" -- the quad's yaw
authority is meant to come from genuine propeller reaction torque (each rotor
alternating CW/CCW), not a vectored thrust mechanism, so this ratio is not
supposed to be tiny the way the tricopter's is.

## 4. Why yaw didn't need retuning

The tricopter's yaw axis needed a lower `ATC_RAT_YAW_P` (see
`TRICOPTER_TUNING.md` section 4) because its yaw authority passes through a
mechanical tail servo with real slew lag. The quad has no such stage: yaw
torque is the *difference* in reaction torque between the CW and CCW rotor
pairs, and that reaction torque follows rotor speed directly -- the same
`RotationalMotor` velocity loop that produces thrust. There's no separate,
slower actuator in the yaw path to introduce phase lag, so the stock
`ATC_RAT_YAW_P` doesn't fight anything unexpected.

The one real actuator lag on this airframe is rotor spin-up itself. Per the
main `README.md`: Webots' `Propeller` shaft slews at a constant `maxTorque`
rad/s^2 (measured; unrelated to any `Physics` node on the fastHelix/slowHelix
graphics), so `maxTorque` on each motor is set to 5000 purely to make that
spin-up fast (about 0.05 s to hover speed) rather than to represent a real
motor's torque. That's what keeps rotor response -- and with it, yaw
response -- fast enough that stock gains are adequate.

## 5. What's already validated (pre-existing, not re-tested for this document)

`quadX.parm`'s own header comment already records validated flight behavior
for the stock ATC gains on this calibration:

> a 20 m GUIDED step settles with 0.6 m overshoot and 0.01 m final error, and
> hover holds within 0.2 deg of tilt.

This document doesn't re-run that flight test -- it's cited here as the
existing evidence that the mass/inertia/thrust numbers above already produce
a well-behaved airframe, in contrast to the tricopter, where the same kind of
check (done fresh, see `TRICOPTER_TUNING.md` section 6) was what caught the
identity-inertia and massless-arm bugs in the first place. If this world's
mass, arm count, or arm radius changes, re-run
`tests/flight_test.py --webots webots_quadX.wbt` (needs `./waf configure
--board sitl && ./waf copter` first) to re-validate, and run
`tests/migrate_worlds.py calibrate webots_quadX.wbt` to recalculate `kt`/`kq`
for the new mass per section 3.
