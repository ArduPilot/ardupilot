# Native FixedWing AirShow demonstration

This Rust applet ports
`AP_Scripting/applets/Aerobatics/FixedWing/plane_aerobatics.lua`. The original
controller and trajectories are by Matthew Hampsey, Andy Palmer and Andrew
Tridgell, with controller assistance from Paul Riseborough and testing by Henry
Wurzburg. This port is AI-assisted (Codex, with independent Claude reviews).
It is experimental and not established as safe for hardware flight.

The ten-maneuver AirShow runs from RC or `NAV_SCRIPT_TIME` in Plane SITL. Tests
also exercise the controller and full runner on H743/ChibiOS under Renode.
It is a single-aircraft, numeric-schedule demonstration, not a complete
replacement for every feature of the Lua applet.

## Build and install

Build Plane with `--enable-modules`. Use Rust 1.98.1 or newer and install
`thumbv7em-none-eabihf` for STM32H7. Small-board resource limits may prevent
using this applet on F405; F405 is not an acceptance target for AirShow.

```sh
Tools/modules/build.py libraries/AP_Module/applets/aerobatics/main.rs \
    build/modules/airshow.apmod --target thumbv7em-none-eabihf --cpu cortex-m7 \
    --stack-size 16384 --heap-size 32768
Tools/modules/apmod_info.py build/modules/airshow.apmod
```

For x86-64 SITL/Linux, use `--target x86_64-unknown-linux-gnu` and omit `--cpu`.
Images are target-specific. See the [runtime guide](../../README.md) for
firmware options, ROMFS installation and memory accounting.

Install `airshow.apmod` in `/APM/modules/` on microSD (`./modules/` in the
SITL working directory). Put this directory's **numeric** `AirShow.txt` in
`/APM/scripts/trick72.txt` (`./scripts/trick72.txt` in SITL). Do not install
modules in the repository's `modules/` submodule directory or substitute the
original executable-Lua schedule, which the numeric parser rejects.

Set `MOD_ENABLE=1`, `MOD_HEAP_MAX=32768`, `MOD_CODE_MAXSIZE=131072` and
`MOD_STACK_MAX=32768`, then reboot. The build command declares a 32 KiB initial
heap and a 16 KiB module stack; the latter excludes the host-call margin.
The supplied schedule's regression fixtures use these requests, not a guarantee
for arbitrary schedules or workloads. To request a larger initial heap, change
`--heap-size` when building and raise `MOD_HEAP_MAX` accordingly; changing the
parameter alone does not change the module's initial request.

`MOD_DEBUG_OPTS` bit 6 disables emergency heap expansion while armed. Without
it, an undersized heap may expand up to `MOD_HEAP_MAX`, with a warning and
latched pre-arm error. Use representative peak/stack measurements before
considering hardware use, not historical results from a different SDK/toolchain.

## RC and mission operation

All three native implementations require firmware built with
`AP_MODULE_SHARED_STATE_ENABLED=1`, even when only one language is installed.
The shared-state service coordinates activation as well as live handoff.

`AEROM_LANG` selects the native implementation: `0` runs Rust (default), `1`
runs Ada, and `2` runs Fortran. It is a shared dynamic parameter at `AEROM_`
index 35; existing parameter indices are unchanged. All three modules may be
loaded together. Each checks the live selector every 25 ms and sleeps without
updating control when it is not selected. Other values leave all three idle.
On selection, the module sends `Aerobatics RUST selected`, `Aerobatics ADA
selected`, or `Aerobatics F90 selected` once, including initial selection at startup.
Changing the selector during a running RC or AUTO schedule transfers the controller
to the selected language. Install all three modules and use the same schedule file.
The outgoing worker completes its current tick, writes a versioned snapshot to
shared block `0x4145524f` (2048 bytes), yields ownership and sleeps. The incoming
worker claims ownership with a conditional write, rebuilds the path using the
original load speed/entry rate/box width, verifies the schedule fingerprint and
restores the controller and path caches. Schedule position, origin, orientation,
previous rates, XY shifts, throttle integrator and maneuver message cursors all
continue from the outgoing worker. No language-specific object layout or pointer
is shared. See [the snapshot format](handoff.md) for the wire layout.

The block is reset when first opened after boot or a complete module restart;
joining workers do not reset it. Starting a new schedule initializes a fresh
controller. A snapshot older than Plane's one-second command deadline is not
resumed. A changed schedule file, incompatible snapshot or failed restore aborts
control through the existing watchdog. Selecting an invalid or unloaded language
also lets that watchdog expire. A worker that fails before yielding cannot be
replaced by another worker until `MOD_RESTART` restarts the modules; ownership
is never stolen from a worker that could still issue commands.

After the module registers its parameters, set `TRIK_ENABLE=1` and change
`MOD_RESTART` to reload. Set `TRIK1_ID=72`, `RC7_OPTION=300` (action) and
`RC9_OPTION=301` (selection). Channel 9 low selects trick 1; channel 7 low
aborts, middle selects and high executes. A fresh switch transition in a
supported Plane scripting mode is required; a held execute switch must not
launch a maneuver after reload.

For AUTO missions, use `NAV_SCRIPT_TIME` with command 72 and timeout 0
(no mission-item time limit). `TRIK_ENABLE` and RC scripting options are not
needed. Completion acknowledges the mission command ID. As in Lua,
`AEROM_MIS_ANGLE=0` selects the bearing between surrounding waypoints,
reversed when appropriate for the entry course; a nonzero value fixes the heading.
Waypoint searches skip non-location commands and are limited to 256 items,
including forward `DO_JUMP` traversal, to reject malformed/cyclic missions.

The next script item is preloaded while approaching it. Jumping directly to an
uncached schedule may require loading inside Plane's one-second watchdog;
slow storage can cause that item to be skipped, with a diagnostic.

Ported built-in IDs also accept Lua's four mission/RC arguments:
2 (Loop), 7 (Axial Roll), 8 (Rolling Circle), 9 (Half Cuban Eight),
10 (Half Reverse Cuban Eight), 12 (Humpty Bump), 14 (Scale Figure Eight),
15 (Immelmann Turn), 16 (Split-S), and 26 (Barrel Roll).
Loaded schedules use their file's arguments; mission arguments do not scale
or select a subsection of a loaded schedule.

Do not run the original Lua aerobatics applet concurrently: both use the same
parameter names and vehicle-control interface. Other Lua scripts may coexist.
Parameters share AP_Param tables, keys, defaults, live reads and saved-value
semantics. Dynamic table metadata persists until firmware reboot; identical
registrations reuse it across module reloads. Speed PI gains are captured at
startup, as in Lua.

## Schedule and controller limits

`AirShow.txt` translates the Lua schedule through Barrel Roll, omitting
CrossBoxTopHat and TriangularLoop. The ten maneuvers, nested paths, alignment,
roll references, orientation changes and throttle-boost attributes are compiled
in Rust. The parser accepts at most 16 KiB and 256 commands; these input bounds
do not guarantee that every accepted schedule fits the module heap. Rust and
Fortran reject schedules requiring more than 112 composite cache records before
starting, so the running state always fits the handoff block. Ada also imposes
its 128-node arena limit.

Schedule numbers are finite decimal tokens of at most 255 bytes. Conversion
uses firmware `strtof()` through a hashed binding rather than embedding Rust's
decimal parser in each module. Whole-token validation rejects trailing junk,
hexadecimal and non-finite values. Rounding and locale follow the board's C
library. Conversion may allocate firmware-owned memory outside the module heap.

The retained path tree uses fallible allocation at construction and no
allocation during sampling. Re-entry models Lua's single-entry factory cache,
including refreshing barrel-roll banking from live speed. Maneuver messages are
best effort. Dual-aircraft time sync, unlisted built-in maneuvers, custom Lua
functions, timestamps, rate-controller overrides, optional `@` message prefixes
and debug messages are unsupported. Custom geometry requires rebuilding.

Brief pre-step or command-delivery contention waits for fresh observations on
the next tick. A one-second local command deadline prevents resuming after
prolonged starvation. Ordinary errors release scripting control. On module
stop, host bindings reject further calls and Plane's existing one-second
NavScript watchdog releases control, as with Lua restart. The runner discards
its controller on mode change or mission-item expiry/replacement. Invalid-input
checks can be stricter than Lua's NaN checks; no new disarm/failsafe policy is added.

These modules are trusted native code, not a sandbox. Rust checks and simulation
do not establish instruction budgets, worst-case stack usage, cache correctness
or flight safety.

## Validation

The tests live in `Tools/modules/rust/tests/`:

- `test_aerobatics.py`: primitives and roll profiles against Lua, host Rust
  unit tests, and H743 replay of the oracle with explicit cross-platform tolerances.
- `test_aerobatics_path.py`: complete numeric schedule, lookahead/re-entry,
  fallible allocation and reload cleanup.
- `test_aerobatics_controller.py`: 512 observations for nominal, sweep and
  mirrored/scaled variants, comparing 33 float32 outputs per step against Lua
  on the same platform, including signed zero.
- `test_aerobatics_parameters.py`: parameter defaults, dynamic table expansion,
  legacy repairs and saved values across engine changes.
- `test_aerobatics_languages.py`: idle selection with individual/all workers,
  and a complete AirShow with repeated live handoffs between every pair of languages,
  checking maneuver order, continuous progress, control gaps and position error.
- `test_aerobatics_flight.py`: sequential Lua/Rust SITL flights, maneuver order,
  path error, repeat/abort and restart while controlling.
- `test_aerobatics_faults.py`: test-only Busy injection at read/delivery
  boundaries, timeout and abort recovery. Ordinary builds omit the injection code.
- `test_aerobatics_runner_h743.py`: full runner, RC/mission requests and reloads
  with stationary emulated sensors; this is not a H743 flight simulation.

Build SITL Plane with both engines before the comparison tests. For example:

```sh
./waf configure --board sitl --enable-modules --enable-lua-scripting
./waf plane
MAVLINK20=1 python3 -m pytest -q -s Tools/modules/rust/tests/test_aerobatics_flight.py
Tools/autotest/autotest.py test.Plane.AerobaticsScripting_rust
```

Plane autotests `AerobaticsScripting_rust_mission` and
`AerobaticsScripting_rust_mission_control` cover AUTO completion, built-in
arguments, timeout, replacement and mode exit with RC tricks disabled.

For H743 path replay, build the dedicated harness:

```sh
./waf configure --board MatekH743 --enable-modules \
    --extra-hwdef libraries/AP_Module/examples/ModuleMath/aerobatics.hwdef
./waf --targets examples/ModuleMath
RENODE=/path/to/renode \
    python3 -m pytest -q -s Tools/modules/rust/tests/test_aerobatics_path.py
```

Controller comparisons also need the target-local Lua oracle Plane. Build it
with `Tools/modules/aerobatics-oracle.parm` and
`Tools/modules/aerobatics-oracle.hwdef`, then build `examples/ModuleMath` with
`libraries/AP_Module/examples/ModuleMath/controller.hwdef` without replacing
the Plane binary. Oracle firmware has embedded test defaults and must not be
used for flight. Reconfigure normally before building deployable firmware.

Harness tests verify their exported memory/deadline settings and reload cleanup.
Use their current reports and `apmod_info.py` for resource comparisons. Include
code (RAM-loaded for SD modules, flash-backed for eligible ROMFS modules),
worker/manager stacks and firmware-owned metadata, not just module heap usage.
SITL CPU-time and Renode results are not physical-H7 timing benchmarks.
