# Tricopter stability: how the numbers were calculated

This documents the calibration behind the `webots_tricopter.wbt` /
`webots_two_tricopter.wbt` fix, so it can be redone if the airframe changes
(mass, arm length, rotor count) without re-discovering the method by trial and
error. It splits the work into two kinds:

- **Calculated** -- mass, inertia and rotor thrust/torque constants. These
  follow directly from formulas already used by `tests/migrate_worlds.py`, and
  are reproducible: change the world's mass and re-run the script.
- **Empirically tuned** -- the tail servo's actuator limits and
  `ATC_RAT_YAW_P/D`. These were found by flying the real simulator and
  measuring the result; the calculations below explain *why* they needed
  tuning, not what value to use.

Background for the underlying bugs this fixes: identity-inertia fallback on
the Robot node, and massless rotor arms. See the "Notes on the world files"
section of `README.md` and the comments in `tricopter.parm` /
`worlds/webots_tricopter.wbt`.

## 1. Mass budget

Webots treats every `Solid`/`Physics` node rigidly attached to the vehicle's
`Robot` node as part of one rigid body, so the airframe's total mass is just
the sum of all of them:

| Component | Mass (kg) |
|---|---|
| Main body `Solid` (box, unchanged) | 1.000 |
| Robot's own placeholder `Physics` (unchanged) | 0.001 |
| Front arm nacelle, motor1 (added) | 0.050 |
| Front arm nacelle, motor2 (added) | 0.050 |
| Tail assembly, motor3 + servo (0.001 -> 0.06) | 0.060 |
| **Total** | **1.161** |

This is exactly what `tests/migrate_worlds.py`'s `vehicle_mass()` computes: it
regex-scans every `mass` field under the Robot block and sums them (dividing
by vehicle count for the two-vehicle world). That total mass is the sole input
to the thrust recalibration in step 3 -- change any mass above and re-running
`migrate_worlds.py` recalculates everything downstream automatically.

The front-arm nacelle mass (0.05 kg) is not arbitrary: it is the exact value
already used for every arm on `webots_quadX.wbt`, chosen so the two airframes
share the same nacelle-inertia convention (step 2).

## 2. Inertia

`migrate_worlds.py`'s `fix_solid_inertia()` treats each rigidly-attached
nacelle as a solid sphere of radius `NACELLE_RADIUS = 0.04 m` and assigns it
the sphere moment of inertia about its own center:

```text
I = 0.4 * m * r^2
```

For the front arms:

```text
I = 0.4 * 0.05 * 0.04^2 = 3.2e-05 kg m^2   (per axis)
```

identical to the quad's arms, since it is the same formula and the same
nacelle mass.

The dominant contribution to the *airframe's* rotational inertia is not a
nacelle's own inertia but the parallel-axis term from its distance to the
center of mass:

```text
I_parallel = m * r^2
```

For the tail assembly (0.06 kg at a 0.34 m boom):

```text
I_parallel = 0.06 * 0.34^2 = 6.94e-03 kg m^2
```

which is ~200x the nacelle's own 3.2e-05 kg m^2 term. This is why getting the
*mass* of a far-out component right matters far more than its shape or its own
`inertiaMatrix` -- and why the two front arms, which previously had **no**
`Solid`/`Physics` node at all (zero mass), were contributing zero inertia
regardless of how correct everything else was.

The main body's `Robot` node had a `Physics` block with no `boundingObject`
and no `inertiaMatrix` of its own, so Webots silently fell back to the
*identity* tensor (1 kg m^2 per axis) -- see the WARNING Webots prints:
`Undefined inertia matrix: using the identity matrix.` One kg m^2 is roughly
50-300x any of the real per-axis totals below, and it was added on top of
whatever the child Solids contributed. This was the dominant bug: adding a
`boundingObject Box { size 0.1 0.1 0.1 }` directly on the Robot node (matching
the quad, which already had one) let Webots derive a correct inertia from
that box instead, and is what took the flight test from a 194 deg flip to a
0.36 deg peak tilt.

Approximate total per-axis inertia after the fix, summing nacelle
parallel-axis terms plus the (now-correct) main-body contribution:

| Axis | Contributions | Total (kg m^2) |
|---|---|---|
| Roll | 2x front arms, `z`-offset only: `2 * 0.05 * 0.3^2` | ~0.0107 |
| Pitch | 2x front arms (`x`-offset) + tail (`x`-offset) | ~0.0102 |
| Yaw | 2x front arms (`x^2+z^2`) + tail (`x^2`) | ~0.0187 |

(plus ~0.0017 kg m^2 from the main body box on every axis, and negligible
per-nacelle own-inertia terms). These are order-of-magnitude sanity checks,
not values written into any file -- Webots computes the real numbers from
geometry.

## 3. Rotor thrust/torque constants

`tests/migrate_worlds.py calibrate` sets a world's `Propeller` constants for
the mass the world has now. For the tricopter it offers these defaults, read
from the world itself: 3 rotors, `omega_max` 400 rad/s, torque ratio 8.74e-7,
and thrust-to-weight 2.2.

`torque_ratio` (torqueConstants / thrustConstants) was deliberately kept at
its original, near-zero value: the tricopter's yaw authority is meant to come
entirely from the tilting tail rotor, with the propellers' own reaction torque
kept negligible by design. Only the thrust *scale* changes when mass changes.

With `mass = 1.161 kg` (step 1), `g = 9.80665`, thrust-to-weight `tw = 2.2`,
`omega_max = 400 rad/s`:

```text
thrust_per_rotor = tw * mass * g / rotors
                 = 2.2 * 1.161 * 9.80665 / 3
                 = 8.349 N

kt (thrustConstants) = thrust_per_rotor / omega_max^2
                      = 8.349 / 400^2
                      = 5.2184e-05

kq (torqueConstants) = kt * torque_ratio
                      = 5.2184e-05 * 8.74e-7
                      = 4.5608e-11

hover_throttle = 1 / tw = 1 / 2.2 = 0.4545  (~45%)
```

These match exactly what `migrate_worlds.py` wrote into the world files
(`5.21836e-05` / `4.56085e-11`). To redo this after a mass change, edit the
mass in Webots, then:

```bash
cd libraries/SITL/examples/Webots
python3 tests/migrate_worlds.py calibrate webots_tricopter.wbt webots_two_tricopter.wbt
```

It shows the new mass, asks for the thrust-to-weight, rotor speed and torque
ratio (defaulting to the values above), shows the resulting constants, and
changes nothing but the rotor constants, and only once you confirm. Add
`--check` to see the result without writing it.

## 4. Yaw torque vs. yaw inertia -- why the rate gain needed retuning

This calculation doesn't get written into any file, but it is what explains
the yaw oscillation that showed up once roll/pitch were fixed, and why the
fix was a *lower* `ATC_RAT_YAW_P`, not a higher one.

Static hover moment balance about the pitch axis (front motors' thrust times
their arm must equal the tail rotor's thrust times its boom, since collective
thrust must also sum to the vehicle's weight) gives, for this geometry (front
arms at x ~= 0.165 m average, tail boom at x = 0.34 m):

```text
F_front * 0.165 = F_tail * 0.34
F_front + F_tail = weight = mass * g = 1.161 * 9.80665 = 11.386 N

=> F_tail ~= 3.72 N   (at hover)
```

Maximum yaw torque available is the tail rotor's thrust vectored through the
servo's maximum deflection (`MOT_YAW_SV_ANGLE`, default 30 deg):

```text
yaw_torque_max = F_tail * sin(30 deg) * boom_length
               = 3.72 * 0.5 * 0.34
               ~= 0.63 N m
```

Against the corrected yaw inertia (~0.0187-0.02 kg m^2 from step 2):

```text
yaw_accel_max = yaw_torque_max / yaw_inertia
              = 0.63 / 0.02
              ~= 31 rad/s^2   (~1800 deg/s^2)
```

That is a large amount of torque authority relative to inertia, applied
through a tail servo with real mechanical lag (unlike a quad's near-instant
reaction-torque yaw). That combination -- high authority, added phase lag --
is a textbook setup for a rate-loop resonance, which is exactly what was
observed: a slowly-growing ~1 Hz yaw oscillation with roll/pitch staying
clean throughout. This estimate is what motivated looking for a *gain*
problem instead of another mass/inertia problem once the flip was fixed but
the yaw wobble remained.

## 5. What was tuned empirically, not calculated

The following were found by running the real simulator (headless Webots +
SITL + a scripted MAVLink flight, arm/takeoff/hover/measure) and iterating,
not derived from the formulas above:

- **Tail servo actuator limits**, in `worlds/webots_tricopter.wbt` /
  `worlds/webots_two_tricopter.wbt`:

  ```text
  RotationalMotor {
    name "servo_tail"
    controlPID 4 0 0.4
    maxVelocity 15
    maxTorque 20
  }
  ```

  `maxVelocity 15` (~860 deg/s) is in the range of a fast digital servo.
  `maxTorque 20` N m is about 27x the ~0.73 N m a rough slew-time estimate
  (accelerating the tail's 6.94e-03 kg m^2 inertia to a 0.1 s half-slew) says
  is needed -- generous headroom without being the original `maxTorque 1000`,
  `maxVelocity 50000` (a near-instantaneous, essentially teleporting
  actuator, which is what let a whole-vehicle mass added at that boom start
  injecting real momentum kicks into the airframe every control step).

  This value is used by `ardupilot_SITL_TRICOPTER.c` too: it used to hard-code
  `wb_motor_set_velocity(servo, 1000)`, which silently exceeded a lowered
  world `maxVelocity` and spammed a Webots warning on every control step
  (~32,000 times in one flight test, and enough load in the two-vehicle world
  to crash Webots outright). The controller now reads the servo's actual
  `maxVelocity` via `wb_motor_get_max_velocity()` at startup, the same way it
  already did for the three thrust motors.

- **`ATC_RAT_YAW_P` / `ATC_RAT_YAW_D`**, in `tricopter.parm` / `tricopter2.parm`:

  ```text
  ATC_RAT_YAW_P    0.1
  ATC_RAT_YAW_D    0.01
  ```

  The autotest default (`ATC_RAT_YAW_P 0.3`, from
  `Tools/autotest/default_params/copter.parm`) resonated on the corrected
  airframe (yaw-rate RMS ~85 deg/s over a 20 s hover). Lowering P to 0.1 and
  adding a small D term brought that down to ~0.1-0.13 deg/s RMS with no
  crash, in both the single- and two-tricopter worlds. Section 4's estimate
  explains why the default was too hot; it does not by itself hand you 0.1 --
  that number came from testing 1.9 (the old, even-more-aggressive value this
  file used to carry) -> 0.3 (stock) -> 0.1+D, and measuring each.

## 6. How it was verified

Each change was checked against a real flight, not just read from the world
file, using a short scripted test: headless Webots (`xvfb-run ... --batch
--mode=fast --no-rendering`) plus the real `arducopter` SITL binary
(`--model webots-tri:127.0.0.1:<port>`), driven over MAVLink (pymavlink) to
arm, take off to 10 m, hold for ~20 s, and measure attitude RMS/max and
yaw-rate RMS. This mirrors `tests/flight_test.py`'s approach (built for the
quad) but targets the tricopter's `webots-tri` model and `tricopter.parm`.
The two-vehicle world was verified the same way but needs *both* vehicles'
SITL instances connected -- Webots' synchronized stepping stalls if either
robot's controller is left waiting for a socket connection that never comes,
which is also why `run_two_tricopter.sh` always launches both instances
together.

Before / after, single tricopter, 20 s hover after a 10 m takeoff:

| Metric | Before (identity inertia, massless arms) | After |
|---|---|---|
| Tilt RMS | 99.1 deg | 0.25 deg |
| Tilt max | 193.8 deg (flipped, crash-disarmed) | 0.36 deg |
| Yaw-rate RMS | 31.7 deg/s | 0.10 deg/s |
| Still armed at end | No | Yes |
