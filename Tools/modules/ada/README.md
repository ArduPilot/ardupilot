# Experimental Ada modules

Ada modules use the same container, loader and worker ABI as Rust and C++.
Generic templates generate Ada clients for the complete audited native host-call
catalogue, alongside Rust and C++. The small core SDK provides cooperative
sleep, stop requests and text output. There is no full Ada runtime.

## Toolchain

Use [GNAT LLVM](https://github.com/AdaCore/gnat-llvm), Clang/standalone LLD 21+
and a matching LLVM `llc` (tested with LLVM 21). `ADA` selects the
`llvm-gnat1` frontend; `LLVM_LLC`, `CXX` and `LLD` select the other tools.
Use GNAT's `ADA_INCLUDE_PATH` to select the toolchain's Ada runtime **source
specifications** for the target if they are not installed in its default
search path. The runtime library itself is not linked.
ARM builds also use the ArduPilot ARM GCC toolchain's `libgcc` for compiler
helpers such as M4 software double arithmetic; `ARM_GCC` can select its driver.

Ordinary GCC-based GNAT does not produce ARM ROPI/RWPI. Its PIC globals use
a GOT, which the module packer rejects. GNAT LLVM emits target-specific LLVM
IR; `llc` then generates PC-relative constants/code and r9-relative mutable
data, matching the existing Rust/C++ addressing model. No loader changes or
Ada support in the firmware are needed.

From the repository root, with the compiler environment set:

```sh
Tools/modules/build.py libraries/AP_Module/examples/simple_loop.adb \
    build/modules/ada_H7.apmod --target thumbv7em-none-eabihf \
    --cpu cortex-m7 --stack-size 8192 --heap-size 4096
Tools/modules/apmod_info.py build/modules/ada_H7.apmod
```

For x86-64 SITL, select its runtime source specifications, use
`--target x86_64-unknown-linux-gnu` and omit `--cpu`. For portable M4/M7 ARM
code use `--cpu cortex-m4`. Installation and firmware parameters follow the
[runtime guide](../../../libraries/AP_Module/README.md).

## Supported subset

The builder compiles one application `.adb` unit (with its companion `.ads`
where needed), the core SDK and generated binding body, not a GPR project. Export
`ap_module_ada_run` with the signature shown in
[simple_loop.ads](../../../libraries/AP_Module/examples/simple_loop.ads).
Do not retain its borrowed context after returning.

Static library-level data, fixed-size stack objects, scalar calculations and
ordinary subprogram calls are supported. Mutable static data resets on each
module load. Only `--opt-level=s` is currently accepted. `--debug` retains Ada
DWARF and the companion ELF; intermediate IR, objects and ALI files remain
beside the output in its `.ada/` directory.

The SDK configuration rejects tasking, allocators, implicit heap allocation,
finalization, secondary stacks, implicit executable trampolines, exception handlers, streams and dynamic
library-level elaboration. There is no binder, `libgnat`, standard I/O or
exception unwinder. Missing runtime functions fail at build/link time.

Bounds, range, arithmetic-overflow and assertion checks are retained. Supported
GNAT check failures call a non-returning helper that records the check name and
panics the module. They do not unwind or resume execution. The normal firmware
panic/retained-worker policy applies; this is not general fault isolation.

## Bindings

`AP_Module.Bindings` contains namespaces such as `Ahrs`, `Gcs`, `Vector3f`,
`SerialPort` and `I2c`. Calls return `AP_Module.Status`; values are `out`
parameters and mutable receivers are `in out`. Check the status before using
an output. `AP_Module.Statuses` names every host status, including `UNAVAILABLE`.
Qualify those constants when also using the parent package: `use` clauses for
both packages would make the shared `OK` and `STOPPED` names ambiguous.
Do not pass overlapping objects to multiple `out`/`in out` formals; normal Ada
aliasing and copyback rules still apply.

```ada
Status := AP_Module.Bindings.Ahrs.Get_Gyro (Ctx, Gyro);
if Status = AP_Module.Statuses.OK then
   Status := AP_Module.Bindings.Vector3f.Length (Ctx, Gyro, Magnitude);
end if;
Status := AP_Module.Bindings.Gcs.Send_Text
  (Ctx, AP_Module.Types.MavSeverity_INFO, "Ada bindings ready");
```

Declare `Gyro` as `AP_Module.Types.Vector3f`, `Magnitude` as
`Interfaces.C.C_float`, and make `AP_Module.Status` operators visible with
`use type AP_Module.Status`. Records, fixed arrays, optional values and enum
types live in `AP_Module.Types`. Optional values have `Present` and `Value`
fields; arrays are zero-based. Enum types preserve the full underlying integer
range so unknown firmware values do not raise a constraint error.
Boolean values use `Interfaces.C.C_bool`; convert to `Boolean` when combining
them with `Standard.Boolean` expressions such as a status comparison.
Ada keywords gain a `_Call` suffix, for example `SerialPort.Begin_Call`.

Generated C-compatible values cross a local bridge to the existing typed C++
clients. Record layouts are explicit in Ada and checked with C++ static
assertions; neither Ada nor C++ object layout becomes a firmware wire format.
The existing clients retain packet validation, optional-result rules and
copyback policies. Mutable inputs are published only on success; outputs are
initialized even on error. Strings use Ada `String`, or fixed byte arrays where
the audited C++ API requires them. Normal string limits and NUL rules apply.

Handle values borrow their module context. Use factory results, do not invent
tokens, and explicitly close resources with the corresponding namespace's
close call when finished. Ada has no generated RAII owners; firmware's existing
per-context cleanup remains the final backstop. Do not move a context or its
handles between threads. Feature guards and runtime availability apply just
as they do for Rust/C++.

There are no subsystem-specific Ada templates or handwritten per-binding bridge
functions. The emitter consumes the same audited typed plans and call models as
C++; unused calls, conversion helpers and imports are linker-collected. Rust's
templates, codecs, ownership model and LTO path are unchanged. Coverage JSON
reports Ada separately; convenience helpers absent from Ada remain classified
as replacements rather than being counted as named APIs.
In particular, integer-to-text uses the firmware formatter rather than
assuming Ada's runtime-dependent `'Image` is available.

## Tests

With the toolchain installed:

```sh
python3 -m pytest -q Tools/modules/ada/tests
```

`ADA_NATIVE_INCLUDE_PATH` and `ADA_ARM_INCLUDE_PATH` let the tests switch runtime
source specifications between targets. Compiler-dependent cases explicitly skip
if GNAT LLVM is absent. SITL uses `AP_MODULE_SITL_BINARY`; H7 tests use `RENODE`
and `AP_MODULE_BUILD_ROOT` containing the `ModuleMath` and `ModuleRuntime`
examples. Tests cover link-address independence, mutable data, debug builds,
rejected runtime features, SITL reloads and deliberate runtime-check failures.
The H7 tests exercise the real ChibiOS loader, split RX/RW memory and worker
lifecycle, including repeated reloads and panic handling.
`tests/stress.py` creates an entry retaining every binding. Tests compile and
pack it at two addresses for native, M4 and M7, then execute all calls through
the real Ada/C++ boundary in a mock host. Generator sanitizer tests independently
exercise every bridge with successful and unavailable replies and round-trip
all ABI record/array types.
