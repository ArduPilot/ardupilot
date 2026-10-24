# Runtime-free Fortran modules

The `.f90` backend uses GNU Fortran with Fortran 2008 C interoperability, Clang
21+ and standalone LLD 21+. It supports x86-64 Linux/SITL, Cortex-M4 and Cortex-M7
without libgfortran, Rust tools, or another firmware loader. See the
[build guide](../README.md#fortran-sitl-and-stm32) for commands and tool selection.

## Generated API

Generic templates generate all 585 host-call clients from the same audited typed
plans as C++. Outputs are rebuilt under
`build/modules/libraries/AP_Module/sdk/fortran/`; do not edit or check them in.
Only used clients, conversions and imports survive linker garbage collection.
Rust clients and firmware bindings are unaffected by this local C bridge.

Use `ap_module_bindings` for types, named enum/status constants and host calls:

```fortran
function run(ctx) bind(C, name="ap_module_fortran_run") result(status)
    use ap_module_bindings
    implicit none
    type(c_ptr), value :: ctx
    integer(c_int32_t) :: status
    type(vector3f) :: gyro

    status = ahrs_get_gyro(ctx, gyro)
    if (status /= module_ok) return
    status = gcs_send_text(ctx, mavseverity_info, "Fortran ready")
end function
```

Names are case-insensitive `group_method` names, for example `vector3f_length`,
`filesystem_read_at` and `parameter_cache_get`. Names longer than Fortran's
63-character limit keep a readable prefix and a deterministic hash suffix.
The generated source is the complete API reference, including argument types
and `intent(in)`, `intent(out)` or `intent(inout)` directions.

Calls return `integer(c_int32_t)` status, with results in separate `out_*`
arguments. Check status before using results. Outputs are initialized even on
error; `inout` values change only on success. Do not overlap objects passed to
multiple output/input arguments: normal Fortran aliasing rules apply.
Optional results have `%present` and `%value` fields; check `%present` too.

Records are `type, bind(C)` values; arrays are fixed-size records with
zero-based `%data(0:n-1)` components. These are local C layouts, not wire packets.
The generated bridge converts fields explicitly and checks C layout sizes,
alignment and offsets. Public strings are `character(kind=c_char, len=*)`;
the wrapper passes their actual length, including trailing blanks, without a
descriptor across the C boundary. Substrings and non-unit lower bounds work.
Embedded NULs and oversized strings are rejected by the shared C++ clients.

Use `ap_module` selectively for core `time_us`, `sleep_ms`, `stop_requested`
and `send_text_floats`. For example, `use ap_module, only: sleep_ms` avoids
ambiguous status constants when also using `ap_module_bindings`. The simple-loop
example uses the generated AHRS/GCS calls and the firmware formatter for text;
Fortran `PRINT` and internal formatted `WRITE` need libgfortran and are not supported.

## Integer values and resource tokens

Fortran has no standard unsigned C-interoperable integer. Unsigned API values
therefore use same-width `integer(c_int8_t/c_int16_t/c_int32_t/c_int64_t)` bit
patterns. For example, a `uint8_t` value of 255 is represented as `-1_c_int8_t`.
The C bridge preserves all bits in both directions. For unsigned comparisons or
conversions, widen and mask, rather than sign-extending blindly:

```fortran
unsigned_byte = iand(int(byte_value, c_int32_t), 255_c_int32_t)
unsigned_word = iand(int(word_value, c_int64_t), int(z'FFFFFFFF', c_int64_t))
```

Use `IBITS`, `IAND`, `IOR`, `ISHFT` and `TRANSFER` for full-width bit operations.
Do not use signed overflow as unsigned arithmetic. A full-range uint64 cannot
be represented as a positive standard 64-bit integer; retain its bit pattern
or split it into masked high/low halves. Named enum constants use these same
integer kinds, so unknown bits remain representable. `logical(c_bool)` is used
for booleans, not integer truth values.

Resource factories return context-owned tokens. Close them with the matching
API and do not reuse consumed tokens; Fortran does not have Rust's consuming
handle types. Firmware also cleans up resources at module exit. The context
and handles belong to one module worker: do not retain the context after the
entry function returns or use it from another thread.

## Language limits and tests

Only `-Os` is accepted. There is no libgfortran, formatted I/O, runtime-dependent
allocation or intrinsic support; unresolved runtime calls fail the build.
Avoid array temporaries requiring heap allocation. On ARM, use automatic stack
locals, not mutable `SAVE`, `DATA`, initialized locals or module globals: GCC
cannot generate the required RWPI addressing. The packer rejects unsafe GOT/RW
references. These limits are unchanged by the generated bindings.

`tests/stress_fortran.py` generates a typed call to every binding.
`tests/test_fortran_generated.py` packages/rebases all calls for native, M4 and
M7, executes the native Fortran/C++ ABI with sanitizers, and checks records,
strings, optional filesystem results and error outputs in SITL and H7 Renode.
The main `tests/test_fortran.py` suite also checks build-time rejection of
unsupported features and mixed-language reloads.
