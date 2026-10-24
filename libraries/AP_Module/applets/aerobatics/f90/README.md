# Fortran aerobatics applet

This AI-assisted (Codex) translation retains the adjacent Rust applet's geometry,
14 named maneuver constructors, ten built-in command IDs, numeric schedules,
controller, parameters, RC/mission ownership, watchdog behavior and DataFlash
records. Original aerobatics authors and operating instructions are credited in
the [parent README](../README.md). This is experimental native flight-control
software; simulation and packaging checks do not establish hardware flight safety.

The `.f90` sources use free-form Fortran with the GNU Fortran backend's Fortran
2008 C interoperability, including `ISO_C_BINDING`; they are not strict ISO
Fortran 90. They require no libgfortran, formatted I/O, or Fortran heap runtime.
A small [C++ bridge](memory.cpp) accesses the existing module allocator. The
[local builder](build.py) concatenates the source units and reuses the shared
Fortran backend with that bridge; shared SDK and tooling files are unchanged.

## Build and use

From the repository root, with GNU Fortran, Clang 21+ and standalone LLD 21+
selected as described in [the SDK guide](../../../../../Tools/modules/fortran/README.md):

```sh
python3 libraries/AP_Module/applets/aerobatics/f90/build.py \
    build/modules/airshow-f90.apmod --target x86_64-unknown-linux-gnu

FC=arm-linux-gnueabihf-gfortran \
python3 libraries/AP_Module/applets/aerobatics/f90/build.py \
    build/modules/airshow-f90-h7.apmod --target thumbv7em-none-eabihf --cpu cortex-m7
```

Use `--cpu cortex-m4` for M4 packaging. Defaults request a 16 KiB module stack
and a 32 KiB initial heap. The supplied AirShow used 28,774 bytes of peak module
heap in the SITL RC acceptance test. Larger accepted schedules may need a larger
`--heap-size` and corresponding firmware limits. The path tree is allocated
fallibly in one arena sized for the loaded schedule and never allocates during
sampling; temporary schedule text is freed after construction.

Install target-appropriate images and the parent's numeric `AirShow.txt` as
`trick72.txt`, following the parent README's RC and AUTO setup. Set
`AEROM_LANG=2` to select Fortran; Rust (`0`, the default) and Ada (`1`) can remain
loaded and sleep while Fortran is selected. See the parent README for switching
behavior. The original Lua aerobatics applet must remain disabled because it
does not check this selector. Parameters are registered with the same
keys, indices and defaults. Reads resolve live firmware values by name, and the
speed PI gains are captured at startup. The Fortran runner announces its language
in the startup message; operational maneuver messages retain the Rust wording.
The optional Rust fault-injection compilation mode is not implemented.

## Validation

Build Plane with `--enable-modules`, then run:

```sh
MAVLINK20=1 python3 -m pytest -q -s \
    libraries/AP_Module/applets/aerobatics/f90/tests/test_replay.py
```

`AP_MODULE_SITL_BINARY` can select a separate module-enabled SITL binary. The
replay also builds the existing Rust reference and therefore needs its Rust
build prerequisites. `CXX`, `LLD` and `FC` select the shared backend tools.

Executed validation:

- Runtime-free x86-64, Cortex-M4 and Cortex-M7 image builds and packer checks.
- Three 512-observation SITL controller replays (nominal, sweep and mirror),
  comparing all 30 recorded float fields with Rust, including signed zero;
  every value matched bit for bit.
- The existing full RC AirShow SITL scenario, including maneuver order, held
  execute after reload, repeat, abort, and restart during control. It completed
  with 241 path-error samples and a 6.486 m peak error in that run.
- Parser/roll edge checks, invalid inputs, waiting, altitude abort, completion,
  restart count, backwards time, degenerate-tangent/lookahead fallback, mission
  waypoint skipping, `DO_JUMP` traversal and bounded-cycle checks are part of
  the replay probe.

ARM packaging is not ARM execution. No physical-aircraft or Renode run has been
performed for this translation. Timing and peak-memory observations are specific
to these workloads and the toolchain used; they are not worst-case guarantees.

Live `AEROM_LANG` changes transfer the full controller and path caches through the
shared-memory binding, as described in the [parent guide](../README.md).
The replay checks also compare canonical snapshot hashes with Rust at every
observation and periodically rebuild the path before restoring the snapshot.
