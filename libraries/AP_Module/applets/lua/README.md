# Loadable C Lua interpreter

This experimental applet runs ArduPilot's existing Lua 5.3.6 C interpreter
inside one native module, with built-in Lua disabled. The sources in
`libraries/AP_Scripting/lua/src` are reused with private headers replacing
firmware dependencies. A module-only option keeps instruction hooks active
in finalizers; normal firmware Lua behaviour is unchanged. No loader or ABI
changes are needed.
It supports x86-64 SITL and Cortex-M7 hard float, including CubeOrangePlus.
Contributions were AI-assisted using Codex and Claude review.

## Build and install

From the repository root, using Clang and standalone LLD 21 or newer:

```sh
python3 Tools/modules/lua/build.py build/modules/lua-h7.apmod \
    --target thumbv7em-none-eabihf --cpu cortex-m7
python3 Tools/modules/apmod_info.py build/modules/lua-h7.apmod
./waf configure --board CubeOrangePlus --enable-modules --disable-lua-scripting
./waf plane
```

`CC`, `CXX` and `LLD` select tools outside `PATH`. The builder defaults to a
131072-byte heap and 24576-byte declared stack. `--heap-size`, `--stack-size`,
`--debug`, `--instructions` and `--script-dir` override defaults. ARM defaults
to the M7 profile; M4 modules are not supported by this applet.

Copy the module to `/APM/modules/` and selected Lua files to `/APM/scripts/`.
Configure these parameters and reboot:

```text
MOD_ENABLE       1
MOD_HEAP_MAX     131072
MOD_STACK_MAX    32768
MOD_CODE_MAXSIZE 131072
```

The normal 32768-byte module heap limit must be raised. ARM workers reserve
an additional 4096-byte host-call margin. These resource requests are not
worst-case bounds for arbitrary scripts. SD-loaded code occupies executable
RAM in addition to heap, RW data, stacks and manager. Module-loading scratch
also contributes to peak RAM demand. The existing ROMFS installation mechanism
can execute code from flash, while retaining heap, RW and stack costs.

For SITL, build a separate image and firmware:

```sh
python3 Tools/modules/lua/build.py build/modules/lua-sitl.apmod \
    --target x86_64-unknown-linux-gnu
./waf configure --board sitl --enable-modules --disable-lua-scripting
./waf plane
```

Install under `modules/` and `scripts/` relative to the SITL working directory.
Use `MOD_HEAP_MAX=131072` and `MOD_STACK_MAX=49152`: x86-64 adds a 16384-byte
host-call margin. Increase `MOD_CODE_MAXSIZE` if the reported image requires it.

## Supported examples and APIs

The following examples run without source changes:

- `hello_world.lua`
- `simple_loop.lua`
- `ahrs-print-angle-and-rates.lua`
- `param_add.lua`
- `protected_call.lua`

Start with only these examples in the script directory. An existing SD card
can contain other applets that require bindings this module does not expose.
For example, `plane_aerobatics.lua` also exceeds this host's 65536-byte source
limit. The module reports the offending path and size when rejecting a file.
Directories ending in `.lua` are ignored.

Use `--script-dir /APM/module_scripts` when building to keep the module's test
scripts separate from an existing `/APM/scripts` installation.

| Lua API | Implemented operations |
| --- | --- |
| Standard libraries | ArduPilot sandbox base, math, string and table |
| `gcs` | `send_text`, `send_named_float`, `send_named_int` |
| `ahrs` | `get_roll_rad`, `get_pitch_rad`, `get_yaw_rad`, `get_gyro` |
| `Vector3f` | Constructor and `x`, `y`, `z` accessors |
| `param` | `get`, `set`, `add_table`, `add_param` |
| `Parameter` | Constructor, `init`, `get`, `set`, `set_and_save` |
| `gps` | `status` |
| Time | `millis`, `micros`, boxed `uint32_t`, wrapping addition/subtraction, comparisons, `tofloat`, `toint`, `tostring` |

Availability still depends on firmware features and hardware. Other APIs,
including `io`, `package`/`require`, coroutines, serial and logger, are not
exposed. `SCR_*` runtime parameters are absent in modules-only firmware, so
`param_get_set_test.lua`, which changes those parameters, is unsuitable here.

Lua uses 32-bit integers and floats, matching normal ArduPilot builds.
Integer/string formatting is local; decimal floats use the HAL formatter.
Float precision is limited to 60 and output to 255 bytes per conversion.
Hexadecimal float formatting (`%a`/`%A`, including numeric `%q`) raises an
error. Numeric tokens are limited to 255 bytes. Base-2 and base-10 logarithms
are computed from `logf` and may differ slightly from libc's specialized
functions. Random sequences use a module-local PRNG and need not match normal
Lua.

## Scheduling, errors and reload

The module scans its directory nonrecursively for up to 16 `.lua` files, each
at most 65536 bytes, loading text only. Scripts have separate environments.
They return no values to finish, or a callback and nonnegative millisecond
delay. Deadlines are relative to callback start. A positive sleep after each
scheduler pass allows other ChibiOS workers to run, including for zero delays.

The default budget is 10000 instructions per callback. A hook checks stop
requests every 100 instructions and keeps firing after budget failure so
`pcall` cannot absorb it indefinitely. C-call recursion is limited to 12,
parser recursion to 32 and pattern recursion to 32 to constrain stack use.
Long C library calls and in-flight firmware calls cannot be interrupted by
the instruction hook.
Heap and source-size limits still apply.

A loading/runtime error stops the entire interpreter and returns an error to
the native manager, participating in its usual pre-arm checks. The initial
host does not continue other scripts after one fails. Allocation failure
returns a Lua memory error rather than invoking fatal SDK allocation.
Changing `MOD_RESTART` closes Lua, releases cached parameter storage, and
reloads the module and Lua files. Hooks remain active during close finalizers.

Loaded script paths, sizes and CRC32 values are reported to GCS. Size and
mtime are checked before and after streaming; this is a best-effort check,
not an atomic snapshot. Native `MOD_*` checksums cover the `.apmod` only.
External Lua files do not yet have required-checksum/pre-arm protection.
Native modules run with firmware privileges; see the parent module docs.

## Validation

The tests run unchanged examples and exercise syntax errors, instruction
budgets (including `pcall` loops), recursion, heap exhaustion, invalid delays,
environment separation, finalizers and repeated restart:

```sh
AP_MODULE_SITL_BINARY=/path/to/modules-only/arduplane \
    python3 -m pytest -q Tools/modules/tests/test_lua_module.py -k 'not h7'
```

Set `AP_MODULE_LUA_REFERENCE_BINARY` to a normal Lua-enabled SITL binary to
include the output comparison against built-in scripting. That comparison
is skipped when no reference binary is available.

The ARM packaging test compares different link bases and relocates to RAM
and flash RX addresses. For M7 instruction execution and memory measurements
under Renode, build the HAL harness:

```sh
./waf configure --board CubeOrangePlus --enable-modules --disable-lua-scripting
./waf --targets examples/ModuleLua
RENODE=/path/to/renode OWNED_THREAD_BOARD=CubeOrangePlus \
    python3 -m pytest -q -s Tools/modules/tests/test_lua_module.py -k h7
```

`AP_MODULE_BUILD_ROOT` selects an alternative firmware build root. The harness
loads from emulated SD, exercises protected errors and recursive C callbacks,
measures stack use, and checks that ten unload/reload cycles restore identical
free memory. Emulation does not establish physical-board timing or cache
behaviour; validate those on the intended vehicle setup.
