# Native module tools

These tools build, inspect and test ArduPilot's experimental Rust, C++, Ada and Fortran native
modules. See the [runtime guide](../../libraries/AP_Module/README.md) for firmware
configuration and installation, and the [design](../../libraries/AP_Module/DESIGN.md)
for the container, ABI and resource-ownership contracts.

The [loadable C Lua applet](../../libraries/AP_Module/applets/lua/README.md)
uses `lua/build.py` to run selected existing Lua scripts in modules-only
firmware on Cortex-M7 or x86-64 SITL.

Build the H7 aerobatics, Lua and sine demonstration images together with
`Tools/modules/build_demo.sh`. It can be invoked from any directory and writes to
`build/modules/demo/`; set `AP_MODULE_DEMO_OUTPUT_DIR` to choose another output
directory. Toolchain environment overrides are the same as for the individual
builders.

## Prerequisites

Install Python with `pyelftools`. Rust builds require Rust **1.98.1 or newer**. The minimum compiler
version is shared by the builder and CI through `rust/rust-toolchain-version`.
`RUSTC` selects a compiler other than the default `rustc`.

```sh
rustup update stable
rustup target add --toolchain stable thumbv7em-none-eabihf
python3 -m pip install pyelftools pytest
```

Supported targets are `thumbv7em-none-eabihf` for STM32 and
`x86_64-unknown-linux-gnu` for x86-64 SITL/Linux. ARM defaults to the M4-compatible
profile. Add `--cpu cortex-m7` for H743/H757-optimized code with FPv5 double
precision, or select `--cpu cortex-m4` explicitly for F405 and H7 portability.
H7 firmware accepts either; F405 rejects M7 modules before execution.
The CPU option applies to all language backends; x86-64 only accepts `--cpu x86-64`.
Host builds need native
GCC/binutils; firmware builds and some tests also need the ArduPilot ARM
toolchain. Renode integration tests require `RENODE` to name an executable;
SD-card tests also require mtools. Compiler-dependent tests fail if their
compiler is missing or too old. Renode tests skip when their emulator or
firmware fixture is unavailable.

C++ builds need Clang 21+ and standalone LLD 21+, selected with `CXX` and `LLD`.
ARM builds also need the matching hard-float `libgcc.a` from `arm-none-eabi-gcc`
(`ARM_GCC` can select another compiler) for integer and floating-point helpers.
They do not invoke Rust or require a Rust installation. The C++ SDK is
freestanding C++17: local RAII and virtual dispatch work; exceptions, RTTI, TLS
and global constructors/destructors are rejected. Only `-Os` is currently
accepted for C++ modules, including with `--debug`.

Debugger integration tests use a real GDB process. For SITL, set
`AP_MODULE_SITL_BINARY` to a Plane ELF with C++ debug information and run
`tests/test_gdb_runtime.py`; `AP_MODULE_GDB` overrides native `gdb`. The test
checks one and two modules, Rust values, backtraces and reload cleanup.
For the firmware USB monitor under Renode:

```sh
AP_MODULE_USB_ELF=/path/to/CubeOrangePlus/arduplane \
AP_MODULE_ARM_GDB=/path/to/python-enabled/arm-none-eabi-gdb \
RENODE=/path/to/patched/renode \
    python3 -m pytest -q Tools/modules/tests/test_gdb_usb_runtime.py
```

The USB test launches its own emulator and sparse SD image; it does not attach
to a physical board or use kernel USB/IP drivers. Renode needs the guest
DebugMonitor and USB/IP patches described in `Tools/renode/patches/README.md`.
Both integration suites require GDB Python dependencies and the matching Rust
toolchain; absent optional firmware/emulator fixtures skip explicitly.

The native-module CI workflow builds module-enabled Plane and Copter SITL and
runs the generator, common runtime and per-language regression suites. It installs
Rust, Clang/LLD, native and ARM GNU Fortran, and a cached GNAT LLVM frontend built
from pinned sources. Tests requiring optional firmware, emulator or ONVIF-enabled
SITL fixtures report explicit skips when those fixtures are unavailable.

## Build and inspect

Run from the repository root:

```sh
Tools/modules/build.py \
    libraries/AP_Module/applets/aerobatics/main.rs \
    build/modules/airshow.apmod \
    --target thumbv7em-none-eabihf --cpu cortex-m7 --stack-size 16384 --heap-size 32768
Tools/modules/apmod_info.py build/modules/airshow.apmod
Tools/modules/apmod_info.py --fixups build/modules/airshow.apmod
```

`build.py` dispatches `.rs` to `rust/build.py`, `.cpp`/`.cc`/`.cxx` to
`cpp/build.py`, `.adb` to the experimental `ada/build.py`, and `.f90` to
`fortran/build.py`. Shared bindings are generated below
`build/modules/libraries/AP_Module/`. Rust uses fat LTO; C++ uses Clang/LLD.
All backends package the linked ELF, retaining the companion `.elf` beside the output.
Rust also retains `lib<name>_sdk.rlib`; C++ retains its intermediate `.o` files.
`--heap-size` is required and declares initial heap capacity, checked against
`MOD_HEAP_MAX` on load. Armed growth is permitted only up to that cap.
`--stack-size` declares module stack use, not a measured bound. The firmware's
`MOD_STACK_MAX` must additionally cover the host-call margin: 4 KiB on Cortex-M
or 16 KiB on x86-64; backend overhead is additional.
`--thread-priority` declares native worker priority, defaulting to 2 on ARM and
1 on Linux. Firmware rejects priorities above `MOD_THD_PRIO_MAX` or outside its
HAL's application range. The manager runs at that parameter plus one. SITL and
non-realtime Linux retain host time slicing rather than enforcing these priorities.

`apmod_info.py` validates the file and reports its target, ABI, code/data sizes,
alignments, CPU/FPU requirement, stack requirement, initial heap, thread priority,
relocations and imported API names.
It does not check whether a particular firmware provides those APIs or has enough RAM.
It automatically uses a companion `.elf` to split file size into code, read-only
data, initialized RW data, alignment padding and container overhead. The ELF
must reproduce the module exactly; `--elf PATH` selects a different companion,
and `--no-elf` ignores it. Without an ELF, the tool reports the combined image
size and container overhead; section boundaries cannot be recovered from the
module alone. Padding means gaps between ELF sections and the RX/RW boundary,
not zero-valued instructions or constants within a section. Stack, heap and
zero-fill RAM are not stored in the file.

Change a module's resource declarations without rebuilding its code:

```sh
Tools/modules/apmod_info.py module.apmod --stack-size 16384 --heap-size 32768
Tools/modules/apmod_info.py module.apmod --thread-priority 2
```

Each option can be used alone. Sizes are bytes, from 256 through 16777216;
priorities are integers from 1 through 254, further restricted by the firmware.
The tool validates the module and companion ELF before replacing the file in
place, updates its CRC and displays the new values. The ELF remains usable.
Changing declarations does not establish actual memory requirements: the stack
excludes the host-call margin and the heap is its initial capacity. Firmware
still enforces `MOD_STACK_MAX`, `MOD_HEAP_MAX` and `MOD_THD_PRIO_MAX`. The file checksum changes,
so refresh configured loaded/running checksum requirements before use.
The current `APMOD07` format adds thread priority; rebuild older modules and
firmware together rather than relabelling an old header.

`pack.py` is the lower-level ELF converter. Normal SDK builds should use
`build.py`, which supplies the generated ABI manifest and target profile.
Without a manifest, the converter emits a test-only relocation probe, not a
vehicle-loadable module. Unsupported sections, unresolved symbols and unsupported
instruction relocations are rejected rather than delegated to a firmware linker.

## Experimental Ada (SITL and STM32)

The [Ada backend guide](ada/README.md) describes the GNAT LLVM toolchain,
supported runtime-free subset and tests. The small
[loop example](../../libraries/AP_Module/examples/simple_loop.adb) sends `Ada tick`
once per second using generated `Gcs.Send_Text`. Ada has generated clients for
the complete native host-call catalogue. ARM uses full ROPI/RWPI, including mutable
module-local globals. GCC-based GNAT cannot substitute for GNAT LLVM here.

## Fortran (SITL and STM32)

The [Fortran simple loop](../../libraries/AP_Module/examples/simple_loop.f90)
reads gyro values, computes their magnitude in Fortran, and sends
`Fortran tick <seconds> gyro <magnitude>` once per second. Generic templates
generate the complete 585-host-call catalogue as `ap_module_bindings`, using
`ISO_C_BINDING` and the generated C++
clients; formatted text uses the firmware formatter, not Fortran I/O. The loader,
firmware, container and resource limits are identical for all languages.
See the [Fortran API guide](fortran/README.md) for types, naming and unsigned-value rules.

Install GNU Fortran (tested with gfortran 15.2), Clang 21+ and standalone LLD 21+.
`FC`, `CXX` and `LLD` select the compilers/linker. Rust is not needed to build or
run a Fortran module. From the repository root:

```sh
mkdir -p /tmp/fortran-demo/modules
Tools/modules/build.py libraries/AP_Module/examples/simple_loop.f90 \
    /tmp/fortran-demo/modules/fortran.apmod --target x86_64-unknown-linux-gnu \
    --stack-size 8192 --heap-size 4096
Tools/modules/apmod_info.py /tmp/fortran-demo/modules/fortran.apmod
Tools/autotest/sim_vehicle.py -v ArduPlane --waf-configure-arg=--enable-modules --use-dir /tmp/fortran-demo
```

In MAVProxy, set `MOD_ENABLE` to 1 and reboot (`param set MOD_ENABLE 1`, then
`reboot`). The Fortran module can run alongside Rust and C++ modules; the number
loaded is limited by available memory, not a module-count parameter.
Do not install modules in the repository's `modules/` submodule directory.

For STM32H7, use an ARM GNU Fortran cross compiler (tested with 15.2), for example
the `gfortran-arm-linux-gnueabihf` package. Although that compiler targets Linux
by default, the builder explicitly selects M4-compatible or M7 Thumb hard float,
assembles with Clang and links without Linux startup, libc or libgfortran.
The ArduPilot `arm-none-eabi-gcc` toolchain supplies only referenced compiler
helpers from the matching hard-float `libgcc.a`; `ARM_GCC` overrides that compiler.

```sh
FC=arm-linux-gnueabihf-gfortran Tools/modules/build.py \
    libraries/AP_Module/examples/simple_loop.f90 build/modules/fortran_H7.apmod \
    --target thumbv7em-none-eabihf --cpu cortex-m7 --stack-size 8192 --heap-size 4096
```

Copy `fortran_H7.apmod` to `/APM/modules/` on microSD and enable `MOD_ENABLE`.
SITL and STM32 require separately compiled images. No firmware loader changes
or additional runtime resolver are needed.

The generated bindings cover the full host-call catalogue, but this is still a
runtime-free subset of the Fortran language, using Fortran 2008's C interoperability.
Only `-Os` is accepted; `--debug` retains Fortran/C++ DWARF. Fixed-size storage
and simple scalar/array arithmetic work. `PRINT`, `WRITE` (including internal
formatted writes), runtime-dependent allocation/intrinsics, and other uses of
libgfortran are not supplied and must fail the build. Avoid compiler-created
array temporaries requiring allocation. ARM Fortran uses a GOT-free PIC subset:
keep mutable variables in automatic stack locals, not `SAVE`, initialized locals,
`DATA` or mutable module globals. The packer rejects GOT/RW references it cannot
relocate safely. GCC reserves `r9` for the C++ bridge's RWPI static base; read-only
Fortran constants and code use PC-relative addressing. These restrictions do not
apply to the existing Rust/C++ backends. Flang 21 cannot lower ARM Fortran yet.

Run all-binding package/rebase and native ABI checks, known-value maths and status
tests, and the three-language SITL reload test with native and ARM GNU Fortran installed:

```sh
AP_MODULE_SITL_BINARY=/path/to/sitl/bin/arduplane ARM_FC=arm-linux-gnueabihf-gfortran \
    python3 -m pytest -q Tools/modules/fortran/tests
```

`ARM_FC` selects the cross compiler for tests without changing their native `FC`.
For H743 Renode SD/ROMFS execution and leak-free restart checks, build
`Tools/modules/build_romfs_runtime.py --language fortran` and run
`Tools/modules/tests/test_romfs_manager.py` with the matching
`AP_MODULE_ROMFS_BUILD_ROOT` and `RENODE` settings. The harness uses a known
vector rather than vehicle IMU hardware to check the Fortran calculation.

## Source-level debugging

Build the module and SDK with full debug information, keeping the matching
`.apmod` and `.elf` together on the host:

```sh
Tools/modules/build.py libraries/AP_Module/examples/simple_loop.rs \
    build/modules/simple_loop.apmod --target thumbv7em-none-eabihf \
    --stack-size 16384 --heap-size 4096 --debug --opt-level 1
```

`--debug` enables frame pointers and host-only DWARF, including debugger unwind
information. Debug sections and pretty-printer scripts are not packed into the
module. Frame pointers and a different `--opt-level` can change code size,
stack use and timing. The default optimisation level remains `s`.

Use a **Python-enabled** ARM GDB for STM32 or native GDB for SITL, with
`pyelftools` importable by its Python interpreter. Load the matching firmware
ELF with C++ debug information.

For example, configure and build SITL with:

```sh
CXXFLAGS=-g ./waf configure --board sitl --enable-modules --debug-symbols
./waf plane
```

On STM32, configure with `--debug-symbols --enable-USB-debug
--enable-modules` instead. Connect normally and use:

```gdb
source Tools/modules/apmod_gdb.py
apmod printers
apmod symbols build/modules/simple_loop.apmod
apmod list
info threads
break simple_loop::run
continue
bt
info locals
```

`apmod printers /path/to/rustc` selects the module's Rust toolchain; otherwise
it uses `RUSTC` or `rustc`. Install that toolchain's `rust-src` component to step
into `core`/`alloc`. The helper validates the companion ELF against the module,
matches the runtime's whole-file CRC, and maps RX and RW sections independently.
This supports separate ARM RAM allocations and RX in ROMFS. ARM RWPI static
locations are corrected in a temporary host ELF, never on the board. An ARM
unwinder handles the runtime's naked entry bridge.

Register multiple module files with `apmod symbols FILE.apmod ...`. Registration
can precede module startup; internal breakpoints follow `Module::start()` and
`Module::release()` without adding firmware hooks. Before release, code
breakpoints in that module are deleted and symbols removed, preventing stale
software-breakpoint writes into reused RAM. Recreate those breakpoints after
reload. Remove data watchpoints before reloading modules. `apmod clear` removes
the helper's loaded symbols and lifecycle hooks, including module code breakpoints.
Keep the helper enabled while module breakpoints are installed; clear it before
changing firmware. Optimised variables may still be unavailable.

For USB debugging with `--enable-USB-debug` firmware:

```sh
Tools/debug/gdb_usb.py build/CubeOrangePlus/bin/arduplane --port /dev/serial/by-id/DEVICE-if02 \
    --gdb /path/to/python-enabled/arm-none-eabi-gdb \
    --apmod build/modules/simple_loop.apmod
```

`--apmod` is repeatable and loads the helper. Rust pretty-printers are selected
only for Rust DWARF; C++ debugging does not invoke Rust tools. IDE and
`--serve` sessions can source the helper in their own GDB client instead.
No USB protocol or module-format change is needed. Debugging stops the vehicle;
use a disarmed bench setup, not an aircraft in flight.

## Tests

Shared Python tests live in `tests/`; language-specific tests live in
`rust/tests/`, `cpp/tests/`, `ada/tests/` and `fortran/tests/`. Generator tests live with the
native generator.
These toolchain/loader tests run without a vehicle firmware build:

```sh
python3 -m pytest -q Tools/modules/rust/tests/test_build.py \
    Tools/modules/tests/test_config.py \
    Tools/modules/tests/test_modules.py \
    Tools/modules/rust/tests/test_sdk.py \
    Tools/modules/tests/test_imports.py \
    Tools/modules/tests/test_split_alignment.py
python3 -m pytest -q libraries/AP_Module/generator
flake8 Tools/modules
```

Remaining tests exercise SITL vehicles or ChibiOS examples. Build Plane with
native modules enabled before running `tests/test_runtime.py` or the binding/applet tests.
Lua comparison tests also require Lua. `AP_MODULE_SITL_BINARY` selects a separately
built Plane executable. Do not replace it during a test: SITL executes it again
on reboot.

For the H743 SD runtime harness:

```sh
./waf configure --board MatekH743 \
    --extra-hwdef libraries/AP_Module/tests/enabled.hwdef
./waf --targets examples/ModuleRuntime
RENODE=/path/to/renode python3 -m pytest -q Tools/modules/tests/test_manager.py
```

The mixed ROMFS/SD harness uses a separate build directory:

```sh
Tools/modules/build_romfs_runtime.py
RENODE=/path/to/renode python3 -m pytest -q Tools/modules/tests/test_romfs_manager.py
```

Add `--cpu cortex-m7` to the harness builder to embed an M7-profile ROMFS module.
The SD/ROMFS test exercises M4 and M7 SD modules in all three languages on H743.

`OWNED_THREAD_BOARD` selects a compatible board fixture; `AP_MODULE_BUILD_ROOT`
and `AP_MODULE_ROMFS_BUILD_ROOT` select existing example build roots. Tests for
`OwnedThread`, `ExecutableMemory`, `RustHeap`, `ModuleLoader`, `ModuleMath` and
`StackGuard` need the corresponding examples built first. H7 stack-guard tests
require a Renode build with the Cortex-M stacking-fault fix; they do not establish
real-hardware fault recovery or flight safety.

To exercise stack recovery with USB debugging enabled, build the CubeOrangePlus
examples and run both the unattached and attached-monitor tests. Use the patched
Renode described in [the USB debug guide](../renode/patches/README.md):

```sh
./waf configure --board CubeOrangePlus --enable-modules --enable-STACK_GUARD --enable-USB-debug --debug-symbols
./waf --targets examples/StackGuard,examples/ModuleRuntime
STACK_GUARD_BOARD=CubeOrangePlus RENODE=/path/to/renode \
    python3 -m pytest -q Tools/modules/tests/test_stack_guard.py
AP_STACK_USB_ELF=build/CubeOrangePlus/examples/StackGuard RENODE=/path/to/renode \
    python3 -m pytest -q Tools/modules/tests/test_stack_guard_usb.py
```

The attached tests check debugger probes against an active guard, repeated
overflow recovery, mutex release, FP use and refusal of unattributable or
critical-section faults. `AP_MODULE_BUILD_ROOT` selects a non-default build root
for the unattached tests; `STACK_GUARD_BOARD` defaults to `MatekH743`.

Vehicle autotests build and install modules through scoped helpers that refuse
to overwrite existing files. For example:

```sh
Tools/autotest/autotest.py build.Plane test.Plane.AerobaticsScripting_rust
```

Test sources/data remain beside the tools or under `libraries/AP_Module/tests/`.
Frozen fixtures in `libraries/AP_Module/tests/compat/` must not be regenerated
during routine SDK changes.
