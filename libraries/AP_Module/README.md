# Native modules

`AP_Module` loads trusted native modules from SD card or ROMFS. Each module
has its own worker thread, heap and stack and uses generated bindings to call
ArduPilot APIs. Rust, freestanding C++ and
[Fortran](../../Tools/modules/fortran/README.md) share one loader and ABI on
x86-64 Linux/SITL and STM32. Lua can be
enabled independently or alongside native modules.
An [experimental Ada backend](../../Tools/modules/ada/README.md) uses GNAT LLVM
and template-generated bindings with the same loader and ABI, without linking an Ada runtime.

See [DESIGN.md](DESIGN.md) for implementation contracts and the
[module tools guide](../../Tools/modules/README.md) for packaging and tests.

## Build the firmware

Native module support is included by default when `HAL_PROGRAM_SIZE_LIMIT_KB > 2048`,
including SITL, unless disabled by a board or build override. WebAssembly builds
cannot execute native modules. Runtime loading is disabled until `MOD_ENABLE` is set.

For a modules-only SITL Plane build:

```sh
./waf configure --board sitl --enable-modules --disable-lua-scripting
./waf plane
```

For STM32, select the board and keep or disable Lua as needed:

```sh
./waf configure --board CubeOrangePlus --enable-modules
./waf plane
```

The independent options are `--enable-modules` /
`--disable-modules` and `--enable-lua-scripting` /
`--disable-lua-scripting`. The older `--enable-scripting` /
`--disable-scripting` options select Lua. Disable both runtimes explicitly to
build without scripting. Extra hwdefs should use `AP_MODULE_ENABLED`
and `AP_SCRIPTING_LUA_ENABLED`; `AP_SCRIPTING_ENABLED` denotes shared support.

Supported module profiles are Cortex-M4/FPv4-SP and Cortex-M7/FPv5-D16 Thumb-2
hard float on STM32, and x86-64 on SITL/Linux. Other Linux architectures need a separate target
profile; ARM Linux cannot use the STM32 module. Executable-memory support covers
STM32H743/H757 and F405. Small-board flash/RAM limits may prevent a complete
vehicle build with the desired features.

## Build a module

Install Rust **1.98.1 or newer**, Python `pyelftools`, and the target:

```sh
rustup update stable
rustup target add --toolchain stable thumbv7em-none-eabihf
python3 -m pip install pyelftools
```

`RUSTC` can select a compiler explicitly. The standalone builder compiles modules
directly, without a Cargo project. A minimal source file is:

```rust
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::MavSeverity, Context, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    loop {
        context.gcs().send_text(MavSeverity::INFO, "Native module alive")?;
        context.sleep_ms(1000)?;
    }
}
```

Build an existing applet from the repository root:

```sh
Tools/modules/build.py \
    libraries/AP_Module/applets/aerobatics/main.rs \
    build/modules/airshow.apmod \
    --target thumbv7em-none-eabihf --cpu cortex-m7 --stack-size 16384 --heap-size 32768
Tools/modules/apmod_info.py build/modules/airshow.apmod
```

Use `--target x86_64-unknown-linux-gnu` and omit `--cpu` for x86-64 SITL/Linux.
Modules are target-specific. For ARM, `--cpu cortex-m4` is the portable default; use
`--cpu cortex-m7` for H743/H757, including their double-precision FPU. H7 boards
accept either profile; F405 accepts only M4 modules. The loader rejects unsupported
CPU requirements before allocating the image. This choice applies to Rust,
C++, Ada and Fortran. `apmod_info.py` shows each module's CPU/FPU requirement.

The stack declaration excludes the host-call margin, currently
4 KiB on Cortex-M and 16 KiB on x86-64. The declaration plus margin must fit
`MOD_STACK_MAX`; OS/guard overhead is additional. Measure the complete applet
and host-call stack usage; the declaration is not a stack proof.

Waf generates bindings below `build/<board>/libraries/AP_Module/` and rebuilds them
when inputs change. The module builder uses
`build/modules/libraries/AP_Module/`; Cargo SDK tests use Cargo's `OUT_DIR`.
Generated bindings are not checked into the source tree.

For C++, install Clang 21 or newer and standalone LLD 21 or newer (no Rust installation is required):

```sh
Tools/modules/build.py libraries/AP_Module/examples/simple_loop.cpp \
    build/modules/simple_cpp.apmod --target thumbv7em-none-eabihf \
    --stack-size 8192 --heap-size 4096
```

`CXX` selects Clang and `LLD` selects `ld.lld`. C++ modules use the freestanding
C++17 SDK in `sdk/cpp/`; exceptions, RTTI, TLS and automatic global constructors
or destructors are unsupported. Local RAII, virtual dispatch and module-owned
`new`/`delete` are supported. Both C++ target profiles require the default `-Os`;
`--debug` is supported, but other optimisation levels are rejected.

## Install and run

Copy `.apmod` files into `/APM/modules/` on SD card, or `./modules/` relative to the
SITL/Linux working directory. Set `MOD_ENABLE=1` and reboot. Check GCS messages
for loading, errors and memory reports before enabling application behavior.
The directory is not searched recursively.

For built-in modules, build the `.apmod` first and add an extra-hwdef entry:

```text
ROMFS modules/airshow.apmod build/modules/airshow.apmod
```

Pass that hwdef to `./waf configure --extra-hwdef <file>`, then build the firmware.
Alternatively put the module in `ROMFS_custom/modules/` before configuration.
The firmware build stores `.apmod` entries without compression and prelinks code
pointers. Other ROMFS files retain the normal compression policy.

STM32 can execute an `APMOD07` ROMFS module directly from qualifying executable
flash, allocating only mutable data/BSS, heap and stack in RAM. SD modules also
copy code into executable RAM. SITL/Linux copy ROMFS code because their embedded
data is not an executable mapping. Ordinary `@ROMFS/` reads reconstruct the
original file, allowing a module to be copied to SD. Globally uncompressed ROMFS
builds retain canonical bytes and copy code rather than prelinking.

ROMFS and writable files are combined in basename order. ROMFS wins a duplicate
basename, with a warning on each load generation. There is no configurable or
fixed module-count limit: available memory determines how many modules fit.
All discovered images are prepared before workers start; a load/start allocation failure
stops the generation rather than silently ignoring modules. Filenames may have
at most 63 characters, including `.apmod`.

## Configuration and memory

Enable, memory-limit and priority-limit changes require reboot.

| Parameter | Default | Purpose |
| --- | --- | --- |
| `MOD_ENABLE` | 0 | Enable module loading |
| `MOD_HEAP_MAX` | 32768 | Maximum backing heap capacity per module |
| `MOD_CODE_MAXSIZE` | 131072 | Per-module RX plus RW/BSS image limit |
| `MOD_STACK_MAX` | 32768 | Maximum declared stack plus host-call margin |
| `MOD_RESTART` | 0 | Change to request cooperative restart-all |
| `MOD_LD_CSUM` | -1 | Required loaded-file checksum; -1 disables |
| `MOD_RUN_CSUM` | -1 | Required running-file checksum; -1 disables |
| `MOD_DEBUG_OPTS` | 0 | Debug options described below |
| `MOD_THD_PRIO_MAX` | 2 on STM32, 1 otherwise | Maximum native worker priority; manager runs at this plus one |

The image budget rounds up to 4096 bytes; this does not pad stored ARM modules.
Only the actual image is allocated, with cache-line rounding for ARM RX RAM
and page rounding on POSIX. Larger ELF alignment requirements are preserved.
The HAL rounds stack requests to its minimum/alignment. Reports separate owned
RX, owned RW, stack reservations and heap usage. Borrowed ROMFS RX reports zero
owned RX bytes. Allocator bookkeeping, loader scratch, manager storage and
persistent firmware registrations are additional; heap usage is not total RAM.
The manager requests a separate 4 KiB stack on ARM or 32 KiB on x86-64.
Download `@SYS/modules.txt` over MAVFTP for per-module state, source, priority,
image/stack reservations, heap free/total bytes and allocation peaks.
The fields use `STATE=RUN`, `SRC=FILE` or `SRC=ROMFS`, `STK=bytes` and
`HEAP=free/total`. CPU load is included only where the HAL can measure it.
Discovery filenames and address-stable callback records are heap allocated, not
placed on the manager stack. Their storage is released only after the entire
generation has stopped and all binding resources are cleaned up. `MOD_MAX_MOD`
has been removed; its parameter index remains reserved. Individual binding
resource limits, such as sockets per module and shared device pools, still apply.

The required `--heap-size` stores each module's initial heap request. The loader
rejects requests over `MOD_HEAP_MAX`. Each worker receives its own declared stack
plus the host-call margin, not the entire `MOD_STACK_MAX` budget.

Like Lua, heaps can expand while armed, retaining 20 KiB of system memory.
Growth is bounded by `MOD_HEAP_MAX` and ten backing arenas. Capacity is charged
for all arenas, not just live allocations; freeing objects does not lower it.
Expansion warns with the module name and required initial heap, and latches a
pre-arm failure until reload/reboot without stopping the module or disarming.
Expanded capacity remains usable after disarm. Rebuild with a larger `--heap-size`
to change initial capacity; raise `MOD_HEAP_MAX` if needed. Fragmentation can
still prevent allocation. Capacity and arena-slot exhaustion are reported separately.

The 32 KiB default is not a demonstrated worst-case bound for every applet.
The aerobatics regression fixtures use a 32 KiB initial heap. Parameter-heavy
applets such as `vtol_quicktune` and
workload-dependent allocations in `net_webserver` need their own measurements;
client count, directory size and aerobatics schedule length can change demand.
Check peak/capacity reports with representative workloads before flight, and
configure both the module's initial request and `MOD_HEAP_MAX` accordingly.

This pre-alpha cutover intentionally rejects old `APRUST` and `APMOD06` modules: rebuild all
modules and update install paths and parameter names. Parameter storage indices
are unchanged, so old saved values carry over. In particular the former initial
heap value now becomes a maximum; review limits before enabling modules.

`MOD_DEBUG_OPTS` bit 4 disables module pre-arm checks. Bit 5 saves both current
checksums and clears itself. Bit 6 disables heap expansion for the next module
generation. Checksums are 23-bit-masked XORs of whole-file CRC32 values, not
authentication; identical files can cancel. Normally returned modules stay in
the loaded checksum but leave the running checksum.

## Restart, errors and scheduling

Changing `MOD_RESTART` requests stop for all workers, joins them and reloads the
files, including while armed. SDK sleeps return `Error::Stopped` promptly;
propagate that result or poll `stop_requested()` and return normally so local
destructors run. Restart does not itself disarm the vehicle.

Workers are never forcibly cancelled. A ten-second stop timeout retains live
code, stacks, heaps and handles and prevents a new generation. Late exits can be
reaped, but another explicit restart request is needed to retry. Panic and
infallible allocation failure park the worker until reboot, not a whole-firmware
panic. Errors latch a module pre-arm failure until the next generation. Normal
return and an empty directory are not errors. SYSTEM-check and force-arm bypasses
follow Lua semantics.

Workers run at the native priority stored in their `APMOD07` header, defaulting
to 2 on STM32 and 1 on Linux/SITL. Set it with `build.py --thread-priority N` or
`apmod_info.py module.apmod --thread-priority N`. Loading fails if the requested
priority exceeds `MOD_THD_PRIO_MAX`, or is outside the HAL's application range.
The manager runs at `MOD_THD_PRIO_MAX + 1`. The parameter requires reboot and
must leave a priority level available for the manager (STM32: 2..254;
Linux/SITL: 1..19). Raising it can let modules preempt flight-critical threads.

Normal ChibiOS configuration has no equal-priority time slicing: a busy module
can starve equal/lower-priority workers and Lua. Use positive sleeps;
`sleep_ms(0)` and stop polling do not yield.
Higher-priority threads can still preempt. POSIX behavior depends on policy:
SITL and `SCHED_OTHER` time-share without native priority enforcement, whereas
Linux FIFO has the same starvation concern.
There is no per-module CPU quota.

For load errors, use `apmod_info.py` and check the target, core ABI, imported API
name/hash and limits against the firmware. Known compiled-out APIs load but
return `Error::Unavailable`; unknown contracts reject loading. Formats and APIs
are experimental, so rebuilding against the current tree may be necessary.

## SDK and bindings

Bindings use Lua-style namespaces: `context.ahrs().get_gyro()` or
`context.vehicle().get_mode()`. Results use `Result` and optional outputs use
Rust's `Option` or C++'s `ap_module::Optional`; no flat compatibility aliases
are generated. Contexts and resource owners must remain on their worker thread,
and a context must outlive its proxies/owners. Rust enforces the cross-thread
restriction; C++ callers must uphold it. Value types are copies, not shared Lua userdata;
methods calling firmware require a live context. Binding locks block and are
released before returning to module code.

Native modules can exchange bounded shared memory by ID. In Rust,
`let block = context.shared_state(id, size)?` opens a context-owned reference;
`block.read(offset, &mut bytes)?` returns the block revision and
`block.write(offset, &bytes, expected_revision)?` copies under the same firmware
mutex. A nonzero expected revision makes the write conditional: `None` means
another writer changed the block and nothing was copied. Zero requests an
unconditional write. Reads and writes are individually atomic; a read followed
by a write requires this revision check to protect the whole transaction.
C++, Ada and Fortran expose the generated `shared_state.open/read/write/close`
family with a token, explicit byte arrays and a revision (zero on a stale write).

All openers of an ID must request the same size, from 1 through 16384 bytes.
Blocks start zeroed, are local to the running firmware, and are freed after the
last close or joined worker cleanup. Limits are 8 blocks, 64 KiB total backing
memory, 32 references globally and 4 per module context. Each copy is at most
2048 bytes; larger blocks require multiple transactions. Tokens belong to their
opening context and are never reused during a firmware boot. Memory is charged
to this bounded firmware pool, separately from module heaps. No module pointer
is retained and no raw shared pointer is exposed. Use a portable byte format
when different languages exchange state. `AP_MODULE_SHARED_STATE_ENABLED=0`
removes the service and its pool; its bindings then return `Unavailable`.

The generated `generator/coverage.json` reports Rust, C++ and Ada separately, distinguishing
host calls, local helpers, value fields, native operations, replacement-only APIs
and unsupported declarations. A replacement is not the original named wrapper and
can use different units. The report covers methods, aliases and audited firmware
version fields, not every constructor, field, enum, operator or global.

Generated Rust docs describe bounds and side effects. Manual Lua facilities have
audited native adapters, not automatic translations. Native bindings can differ
in encoding, ownership and validation. General input strings are NUL-free and
at most 255 bytes: Rust accepts UTF-8 `&str`; C++ uses byte-oriented `StringView`
without requiring UTF-8. Parameter names are at most 16 bytes. Some adapters have
smaller limits; use their generated declarations and documentation.

Use Rust's `context.parameter("NAME")` or C++'s `Parameter::create(context, "NAME")`
for repeated parameter access. Dynamic parameter
metadata/names are firmware-owned until reboot and share Lua's table limit.
Identical registration reuses metadata and restores saved values; unsaved changes
need not survive. Logger format registrations also persist until reboot. Keep
names and definitions stable across reloads.

Module exit cleans up owned resources but does not undo ordinary vehicle state
changes, such as modes, mission edits or RC/output overrides. Host calls can
block on devices or filesystem/parameter locks and cannot be interrupted by stop.
The module heap does not account for all host-owned queues/registry storage.
No module function pointers are registered with firmware.

The optional H7 MPU stack guard (`--enable-STACK_GUARD`) defaults off and is
compatible with USB debugging. It can recover a narrowly attributable stack
overflow, not arbitrary bad pointers, skipped guard pages or shared-state damage.
See the design for its limits.

## Tests

See the [tools guide](../../Tools/modules/README.md#tests) for Python,
SITL and Renode commands. C++ tests are under `tests/`, with standalone harnesses
under `examples/`. Simulation and compilation do not establish hardware timing,
cache correctness or flight safety.
