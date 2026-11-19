# Four-language sine demo

These AI-assisted examples run together and send `NAMED_VALUE_FLOAT` at a
nominal 10 Hz. Each computes `sin(2*pi*SINE_FREQ*t + phase)` using seconds since
vehicle boot, so starting or reloading one implementation does not reset its
phase relative to the others.

| Directory | Message name | Phase |
| --- | --- | --- |
| `lua/` | `Sine_lua` | 0 degrees |
| `rust/` | `Sine_rust` | 90 degrees |
| `ada/` | `Sine_ada` | 180 degrees |
| `f90/` | `Sine_f90` | 270 degrees |

All four share the same dynamic parameter table: key **120**, prefix
`SINE_`, one entry at index **1** named `FREQ`. `SINE_FREQ` is the wave frequency
in **Hz**, defaults to **0.2** (a five-second period), and changes live without
reloading. Zero gives four constant phase values. Frequencies below 5 Hz suit
the 10 Hz sampling rate. Reserve table key 120 for this demo while it is loaded.
The first instance creates the table; later instances bind the existing
parameter, respecting Lua's and native modules' different registration lifecycles.

The native examples use the firmware's sine function and require no language
runtime. The `.f90` implementation uses the SDK's Fortran 2008 C interoperability.
Short-lived binding contention skips a sample; registration retries contention.

## Build and install

Use the [module build guide](../../../../Tools/modules/README.md), including its
[Ada](../../../../Tools/modules/ada/README.md) and
[Fortran](../../../../Tools/modules/fortran/README.md) toolchain setup. From the
repository root, with ARM Ada source specifications selected in
`ADA_INCLUDE_PATH` and an ARM GNU Fortran compiler selected in `FC`:

```sh
for source in rust/sine.rs ada/sine.adb f90/sine.f90; do
    language=${source%%/*}
    python3 Tools/modules/build.py \
        "libraries/AP_Module/examples/sine/$source" \
        "build/modules/sine_$language.apmod" \
        --target thumbv7em-none-eabihf --cpu cortex-m7 \
        --stack-size 8192 --heap-size 4096
done
```

Use `--target x86_64-unknown-linux-gnu` without `--cpu` for SITL, selecting native
Ada specifications and native GNU Fortran. All three modules use the standard
builder, including the Ada example's core `Time_Us` clock accessor.

Copy the three `.apmod` files into `/APM/modules/` and copy `lua/sine.lua` into
`/APM/scripts/` on the vehicle. In SITL, use `modules/` and `scripts/` beneath
the working directory. Enable `MOD_ENABLE=1` and `SCR_ENABLE=1`, set
`MOD_STACK_MAX=32768` to allow the requested stack plus host callback margin,
and reboot. The firmware needs both native
modules and Lua enabled. No arming or vehicle control is involved.

## MAVProxy graph

Run these commands in MAVProxy; the four expressions create one graph:

```text
module load graph
graph NAMED_VALUE_FLOAT['Sine_lua'].value NAMED_VALUE_FLOAT['Sine_rust'].value NAMED_VALUE_FLOAT['Sine_ada'].value NAMED_VALUE_FLOAT['Sine_f90'].value
param set SINE_FREQ 0.4
```

The quoted dictionary keys preserve the message names' case. The longest name,
`Sine_rust`, fits MAVLink's ten-byte name field.

The integration check in `Tools/modules/tests/test_sine.py` runs all four on a
module- and Lua-enabled SITL build, checks names, phases, sampling intervals,
live frequency changes and native reloads while Lua keeps running.
