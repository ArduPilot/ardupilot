# Ada FixedWing AirShow

This is an AI-assisted (Codex) Ada translation of the adjacent Rust aerobatics
applet, derived from the Lua controller and trajectories by Matthew Hampsey,
Andy Palmer and Andrew Tridgell, with controller assistance from Paul Riseborough
and testing by Henry Wurzburg. It remains experimental and requires human review
and vehicle-specific validation before hardware flight.

The port includes the 40 Hz controller, all ten built-in maneuver IDs and all
fourteen named factories, nested numeric schedules, roll and orientation
attributes, speed-dependent barrel banking, mission headings and preloading,
RC selection/abort, parameter registration and repairs, watchdogs and DataFlash
records. Float32 arithmetic and the audited firmware math/vector/quaternion
bindings retain the Rust calculation order. It uses the same parameter keys,
indices, defaults and saved values. Set `AEROM_LANG=1` to select Ada; the Rust
and Fortran modules can remain loaded and sleep while Ada is selected. The
default `AEROM_LANG=0` selects Rust. See the parent README for switching behavior.

## Build and use

Install the toolchain described in the [Ada backend guide](../../../../../Tools/modules/ada/README.md).
From the repository root:

```sh
python3 libraries/AP_Module/applets/aerobatics/ada/build_airshow.py \
    build/modules/airshow-ada.apmod --target thumbv7em-none-eabihf --cpu cortex-m7
python3 Tools/modules/apmod_info.py build/modules/airshow-ada.apmod
```

Use `--target x86_64-unknown-linux-gnu` without `--cpu` for SITL. The wrapper uses
the standard Ada backend and adds `clock.cpp`, the applet's adapter for the
SDK's monotonic microsecond clock.
The controller, trajectories, parser and runner are implemented in Ada.

The defaults request a 16 KiB worker stack and 4 KiB heap. Configure firmware
limits to admit the resulting image; `MOD_CODE_MAXSIZE=262144`,
`MOD_STACK_MAX=32768` and `MOD_HEAP_MAX=4096` admit the checked build requests.
These are build limits, not measured worst-case stack requirements. Native code
is trusted and has no instruction budget or isolation guarantee.

Install only the resulting applet in the writable module directory. Use the
parent directory's numeric `AirShow.txt` as `scripts/trick72.txt` in SITL or
`/APM/scripts/trick72.txt` on the vehicle. RC and mission setup follow the
[parent README](../README.md). Set `TRIK_ENABLE=1` and reload to register the RC
trick tables; AUTO mission operation does not require RC tricks enabled.

## Resource and behavior limits

The runtime-free Ada backend prohibits allocators, exceptions, tasking and
secondary stacks. Paths therefore use a checked static arena of 128 nodes,
including composite nodes and resolved alignments. AirShow fits; larger schedules
can return `NO_MEMORY` before reaching the parser's 256-command bound. Increasing
`Max_Nodes` requires rebuilding and increasing the firmware image limit. No path
allocation occurs while sampling. This replaces Rust's fallible heap allocation
and its module heap bound.

The retained tree, message cursor and re-entry cache follow the Rust applet.
Accepted schedule text is bounded to 16 KiB, validates UTF-8 and finite decimal
numbers using firmware `strtof`, and rejects unsupported directives. Diagnostics
use the same maneuver/start/finish messages, with shorter generic error details.
The Rust-only fault-injection build option is not implemented. All other feature
limits in the parent README continue to apply, including the absence of
multi-aircraft synchronization and unlisted Lua maneuvers.

## Checks

With `ADA`, `LLVM_LLC`, `CXX`, `LLD` and the Ada runtime source include paths set:

```sh
MAVLINK20=1 AP_MODULE_SITL_BINARY=/path/to/module-enabled/arduplane \
    python3 -m pytest -q -s libraries/AP_Module/applets/aerobatics/ada/tests
```

`ADA_NATIVE_INCLUDE_PATH` and `ADA_ARM_INCLUDE_PATH` select the runtime source
specifications per target. Tests compile and pack production x86-64, M4 and M7
images. A test entry is compiled from the actual production body and runs the
original Rust controller replay beside Ada in SITL, comparing every float32 bit
(including signed zero) for nominal, sweep and mirrored/scaled traces: 512
observations and 33 outputs per trace. It also exercises roll conventions,
parser rejection/UTF-8 handling, arena exhaustion, invalid attitude, waiting,
altitude abort and completion. A production-runner check covers dynamic table
expansion, parameter repairs/persistence, reload with the RC execute switch held,
selection, start and abort with stationary SITL observations. The existing full
RC AirShow flight scenario also runs with the production Ada image: all ten
maneuver messages, completion, retained-schedule repeat/abort and reload while
controlling with execute held.

Validation performed for this translation: x86-64/M4/M7 production builds and
packing passed; all three controller traces matched Rust with zero difference;
the parameter/reload/RC checks passed; the full AirShow SITL flight passed with
240 position-error samples and a 6.807 m peak error. The module heap reported
zero bytes used/peak. Flight metrics vary with simulation scheduling.

These numerical, lifecycle and SITL flight checks do not establish hardware
flight safety. Target packaging does not establish ARM numerical replay,
worst-case timing or stack use.

Live `AEROM_LANG` changes transfer the full controller and path caches through the
shared-memory binding, as described in the [parent guide](../README.md).
The replay checks also compare canonical snapshot hashes with Rust at every
observation and periodically rebuild the path before restoring the snapshot.
