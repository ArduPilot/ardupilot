# Using SITL with Webots

[Webots](https://www.cyberbotics.com/#webots "Webots") is an open source robot simulator that provides a complete development environment to model, program and simulate robots. Thousands of institutions worldwide use it for R&D and teaching. Webots has been codeveloped by the Swiss Federal Institute of Technology in Lausanne, thoroughly tested, well documented and continuously maintained since 1996.

This directory holds the C/C++ integration: `libraries/SITL/SIM_Webots.cpp` on the
ArduPilot side, and the Webots controllers under `controllers/` on the simulator
side. (There is a separate Python integration under
`libraries/SITL/examples/Webots_Python`, used by the `webots-python` model.)

## Why this backend

Webots drives each propeller from a real `RotationalMotor` inside a `Propeller`
node, so lift comes from an actual rotor speed rather than a lumped thrust term:

```text
Thrust = thrustConstants[0] * |omega| * omega
Torque = torqueConstants[0] * |omega| * omega
```

The controllers report those rotor speeds back to ArduPilot, which feeds them to
AP_RPM's SITL backend, so RPM logging and the RPM-driven harmonic notch can be
exercised. Webots cannot report a propeller's actual shaft speed, so these are
estimates that follow the rotor's measured spin-up. See "Rotor model" below.

## Installing Webots

Please check this [page](https://www.cyberbotics.com/download "page"). The steps are very easy and straight forward.

The worlds and controllers here are saved as R2025a and were verified against
that release.  On Ubuntu the .deb may leave `libsndio7.0` unsatisfied, which
makes `webots` fail to start with

```text
error while loading shared libraries: libsndio.so.7
```

`sudo apt install libsndio7.0` fixes it.

The first time you open one of these worlds Webots downloads the PROTO assets
named in the `EXTERNPROTO` lines, which takes a little while; later runs are
cached.

## Building the controllers

Set `WEBOTS_HOME` and run `make` in each controller directory, or let Webots
build them when it loads a world.

To check that the controllers still compile on a machine with no Webots
installed:

```bash
libraries/SITL/examples/Webots/tests/build_controllers.sh
```

That builds each controller against `tests/webots_stub`, a header-and-link-only
stand-in for the Webots C API, with `-Wall -Wextra -Werror`.

## Running the simulator

1. open webots and open file `libraries/SITL/examples/Webots/worlds/webots_quadX.wbt`
2. press the "run" button
3. run `./libraries/SITL/examples/Webots/run_quadX.sh`

You can stop and re-run ArduPilot SITL without touching Webots: the controller
waits for the next SITL to connect and puts the vehicle back where the world
started it (this uses the robot's `supervisor TRUE`).  While no SITL is
connected the simulation is paused, since it only advances in step with SITL.

Each `worlds/<name>.wbt` has a launch script named after it, `run_<name>.sh`
(the `webots_` prefix is dropped), run from the repository root:

| World | Script | Controller port(s) |
|---|---|---|
| `webots_quadX.wbt` | `run_quadX.sh` | 5577 |
| `webots_quadPlus.wbt` | `run_quadPlus.sh` | 5599 |
| `webots_tricopter.wbt` | `run_tricopter.sh` | 5599 |
| `webots_rover.wbt` | `run_rover.sh` | 5599 |
| `pyramidMap.wbt` | `run_pyramidMap.sh` | 5599 |
| `webots_two_quadX.wbt` | `run_two_quadX.sh` | 5599, 5598 |
| `webots_two_tricopter.wbt` | `run_two_tricopter.sh` | 5599, 5598 |
| `pyramidMap_two_quads.wbt` | `run_pyramidMap_two_quads.sh` | 5599, 5598 |

The two-vehicle scripts open one xterm per vehicle through
`multi_vehicle_launch.sh`, and send MAVLink to UDP 14450 (vehicle 1) and
14550 (vehicle 2).  They pass `-N` so the two instances do not race `waf` in
the same build directory, so build once first with `./waf copter`.

If MAVProxy's console fails to start under Anaconda with an undefined
`libssh2_session_callback_set2`, Anaconda's `libssh2` is shadowing the
system one; preloading the system library works around it, e.g. on
Debian/Ubuntu x86_64:

```bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libssh2.so.1
```

Each world's `controllerArgs` sets the port its controller listens on, and the
matching script passes the same port to `--model webots-<type>:IP:PORT`.
The multirotor controllers also accept `-df <drag factor>` and `-mv <rad/s>` to
cap the rotor speed below the world's `maxVelocity`.  The rover controller
accepts `-ms <m/s>` (speed at full throttle, default 27) and `-sa <rad>`
(steering angle at full steering, default 0.7); keep `rover.parm`'s
`CRUISE_THROTTLE` and steering gains in step if you change them.

Code shared by all controllers (sensor packing, sockets, and the SITL
connection in `sitl_link.c`) lives in `controllers/common`.

## Simulation using Map Street

You can use [OpenStreetMaps](https://www.openstreetmap.org/ "OpenStreetMaps") with [Webots](https://cyberbotics.com/doc/automobile/openstreetmap-importer "Webots"), it is fairly straight forward. This is a very nice sample  **./libraries/SITL/examples/Webots/worlds/pyramidMap.wbt**, and you need to run SITL using **./libraries/SITL/examples/Webots/run_pyramidMap.sh**
[![Watch the video] Flying at Giza Pyramids](https://www.youtube.com/embed/c5CJaRH9Pig)

## Rotor model

Because thrust is quadratic in rotor speed, the controllers command

```text
omega = sqrt(throttle) * maxVelocity
```

so that thrust is linear in ArduPilot's 0..1 output. The bundled `.parm` files
therefore set `MOT_THST_EXPO 0`; raising it asks ArduPilot to compensate for a
propeller curve the controller has already removed.

The worlds are sized by `tests/migrate_worlds.py calibrate`, which picks
`thrustConstants` and `maxVelocity` for a chosen thrust-to-weight ratio, summing
the vehicle's mass from the world itself. The quad worlds use a 400 rad/s
(~3800 rpm) rotor at a thrust-to-weight of 2.2, which puts hover near 45%
throttle and about 2575 rpm on a 1.2 kg airframe. After changing a vehicle's
mass or rotor count, run it on that world; it asks before changing anything,
and changes only the rotor constants:

```bash
python3 libraries/SITL/examples/Webots/tests/migrate_worlds.py calibrate webots_quadX.wbt
```

`migrate_worlds.py status` summarises every world (version, mass, rotors,
thrust-to-weight), and `migrate_worlds.py migrate` converts a world saved by
an older Webots, such as your own R2021 world, deciding what to convert from
its `#VRML_SIM` version line. The bundled worlds are already R2025a, so
migrate leaves them alone. Run it with no arguments for a menu.

`maxTorque` is deliberately large (5000). Measured in Webots R2025a, a
`Propeller`'s shaft slews towards its commanded speed at a constant `maxTorque`
rad/s^2, as if it had a unit moment of inertia: `maxTorque` 90, 1000 and 5000
gave 90, 1000 and 5000 rad/s^2. With the original `maxTorque 90` a rotor takes
about 3 s to reach hover speed and the vehicle simply falls; 5000 gets there
in about 0.05 s (0.08 s to 400 rad/s). The `fastHelix`/`slowHelix` Solids are
purely graphical, so giving them a Physics node does not change this.
`maxTorque` has no physical meaning in this model; it only sets how quickly the
rotor reaches its commanded speed. This is what the original worlds were really
working around by running their rotors at 3.6 rad/s (about 34 rpm), where even
a small torque is enough.

Webots cannot report a `Propeller`'s actual shaft speed, so the rotor speeds
the controllers send to SITL (`RPM1_TYPE 10`) are estimates: the setpoints
slewed at each motor's `maxTorque` rad/s^2 (`controllers/common/rotor_speed.h`).
That reproduced the measured shaft speed to within 1 rpm, through spool-ups,
steps and 2-10 Hz throttle modulation. They are sent in servo-channel order,
so the first entry is the rotor on SERVO1, as AP_RPM and ESC telemetry
expect.

## Notes on the world files

Two things in these worlds broke on modern Webots and are worth knowing about if
you author your own:

* **`EXTERNPROTO` is mandatory.** R2025a does not silently resolve PROTO nodes;
  it reports `Missing declaration for 'X'` and then *skips the node*. A world
  missing them loads without its scenery — including its terrain, so the vehicle
  falls forever. Webots prints the exact line to add.
* **`UnevenTerrain` is authored for ENU.** Its `size` is (x, y, height) with the
  grid in the XY plane. In these NUE worlds the original `size 500 1 500` gave a
  terrain one metre wide and 500 m tall, standing on edge. It is now rotated flat
  and given `flatCenter TRUE` so a takeoff does not begin on the side of a random
  dune.

`Transform` nodes were renamed to `Pose` (required since R2023b for anything
containing a `Solid`), and the Solids that had a Physics node but no
`boundingObject` or `inertiaMatrix` now carry an explicit inertia — without one
Webots falls back to the *identity* tensor, which made the airframe's rotational
inertia roughly 600x too high.

## Testing without Webots

`tests/mock_webots.py` stands in for Webots: it speaks the same line-delimited
JSON protocol as the controllers and integrates a rigid body whose rotors follow
the `Propeller` equations above. `tests/flight_test.py` flies ArduCopter SITL
against it and checks hover quality, a 20 m position step, and the reported
rotor speeds:

```bash
./waf configure --board sitl && ./waf copter
libraries/SITL/examples/Webots/tests/flight_test.py
```

This is not a substitute for running the real simulator — it does not exercise
Webots' own physics, contact handling, or sensor models — but it catches
protocol, framing and calibration regressions. To run the same checks against
Webots itself:

```bash
WEBOTS_HOME=/usr/local/webots \
  libraries/SITL/examples/Webots/tests/flight_test.py --webots webots_quadX.wbt
```

Note that `webots_quadX.wbt` runs at roughly 0.6x real time on a typical
desktop, and SITL takes its clock from the simulator, so anything that times a
manoeuvre against the wall clock needs to account for that — `flight_test.py`
measures the ratio and scales its own budget.
