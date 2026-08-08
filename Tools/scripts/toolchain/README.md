# ArduPilot STM32 GNU toolchain

This directory builds a Linux-hosted `arm-none-eabi` toolchain for ArduPilot
from source. The result contains GCC and G++, GNU binutils (`ld`, `as`, `ar`,
`nm`, `objcopy`, and the other target utilities), GDB with Python 3 scripting,
and both full and nano variants of newlib and libstdc++.

Two version profiles are supplied:

| Profile | GCC | binutils | GDB | newlib |
| --- | --- | --- | --- | --- |
| `gcc-16.1` (default) | 16.1.0 | 2.47 | 17.2 | 4.6.0.20260123 |
| `ardupilot-10` | Arm 10.2.1 vendor snapshot | 2.35.1 snapshot | 17.2 | 3.3.0 |

`ardupilot-10` uses GCC, binutils, and newlib from Arm's complete
`10-2020-q4-major` source bundle, matching ArduPilot's historical default
toolchain rather than approximating it with the upstream GCC 10.2.0 release.
It deliberately uses GDB 17.2: GDB 10 does not build against current Python
3.14 headers, while the debugger version has no effect on generated firmware.

The components in `gcc-16.1` were checked against the upstream release sites
on 2026-08-08. Versions are data in `versions/*.conf`, so adding or retaining
another combination does not require changing the build logic.

## Host requirements

The scripts target a GNU/Linux build host. On Debian or Ubuntu, install:

```sh
sudo apt-get install \
    build-essential bison flex texinfo curl xz-utils bzip2 patch \
    libgmp-dev libmpfr-dev libmpc-dev libisl-dev libexpat1-dev \
    python3-dev zlib1g-dev libncurses-dev
```

Expect a full build to take several hours and tens of gigabytes of scratch
space. The build is resumable at completed stage boundaries.

## Build

`Tools/scripts/toolchain/build-toolchain.py` is the build driver. The examples
below are run with Python 3 from the root of an ArduPilot checkout.

Build the default GCC 16.1 toolchain:

```sh
python3 Tools/scripts/toolchain/build-toolchain.py
```

Build the toolchain matching ArduPilot's historical default:

```sh
python3 Tools/scripts/toolchain/build-toolchain.py --profile ardupilot-10
```

Useful options include:

```sh
python3 Tools/scripts/toolchain/build-toolchain.py --list-profiles
python3 Tools/scripts/toolchain/build-toolchain.py --show-profile gcc-16.1
python3 Tools/scripts/toolchain/build-toolchain.py --profile gcc-16.1 --jobs 16
python3 Tools/scripts/toolchain/build-toolchain.py --profile gcc-16.1 --download-only
python3 Tools/scripts/toolchain/build-toolchain.py --profile gcc-16.1 \
    --work-dir /fast/toolchain-work \
    --prefix /opt/ardupilot-gcc-16
```

Source archives are kept in `Tools/scripts/toolchain/downloads/` and are shared
by all work directories. The default install is
`Tools/scripts/toolchain/_build/install/<package-name>`. Every downloaded
release archive is checked with SHA-512 before extraction. The `gcc-16.1` profile
pins the same prerequisite versions and checksums as GCC's
`contrib/download_prerequisites` script; the Arm source bundle supplies them
for `ardupilot-10`.

Only use the reported
`Tools/scripts/toolchain/_build/install/<package-name>/bin` directory in
`PATH`. Directories inside a work directory, including `nano-install`, are
internal staging areas and are not standalone toolchains.

Do not change a profile underneath an existing work directory. The script
records an input fingerprint and stops if a resumed build no longer matches
it. Use a new `--work-dir` for a different configuration.

## Rebuild newlib only

After changing the newlib patch, configuration, or forbidden-symbol policy,
the full and nano libraries can be rebuilt without rebuilding GCC, binutils,
libstdc++, or GDB:

```sh
python3 Tools/scripts/toolchain/build-toolchain.py \
    --profile gcc-16.1 --rebuild-library
```

This requires a completed build using the same profile, work directory, and
install prefix. It restores pristine newlib sources, reapplies the repository
patch, discards only `obj/newlib` and `obj/newlib-nano`, and rebuilds and
reinstalls those two library variants. It then regenerates the manifest,
sanitizes all target archives, and runs the complete toolchain verifier. Build
state is updated only after those checks pass.

## Linux release packages

Create a package after a full build or library-only rebuild with `--package`:

```sh
python3 Tools/scripts/toolchain/build-toolchain.py --profile gcc-16.1 --package
python3 Tools/scripts/toolchain/build-toolchain.py --profile gcc-16.1 \
    --rebuild-library --package
```

To verify and package an already installed toolchain without running any build
stages:

```sh
python3 Tools/scripts/toolchain/build-toolchain.py \
    --profile gcc-16.1 --package-only
```

Packaging currently supports x86_64 Linux only. It produces:

```text
Tools/scripts/toolchain/_build/packages/ardupilot-arm-none-eabi-16.1.0-x86_64-linux.tar.xz
Tools/scripts/toolchain/_build/packages/ardupilot-arm-none-eabi-16.1.0-x86_64-linux.tar.xz.sha256
```

The archive contains one matching top-level directory, as do the existing
ArduPilot STM32 toolchain downloads. Debug sections are removed from a staging
copy to keep the release compact; the installed toolchain is not modified.
The staged copy is verified again after stripping and before it is archived.
Use `--package-dir DIR` to select a different output directory.

## Newlib safety policy

After building all multilibs, `scripts/sanitize-newlib.sh` removes archive
members that define symbols listed in `config/forbidden-newlib-symbols.txt`.
The default policy includes ArduPilot's complete ChibiOS blacklist: termination
and host-command entry points, newlib allocation, buffered file I/O, host time
calls, `setjmp`, and the process-global `gmtime`. Headers retain their normal
declarations, so accidental use compiles but fails at link time.

Newlib is built with `NDEBUG`, and its optional reentrancy-allocation
verification is disabled. This removes internal `__assert_func` calls from
routines such as `rand()` and floating-point formatting; newlib's assertion
handler calls the deliberately removed `abort()`. ArduPilot handles allocation
failures and fatal conditions in its own runtime instead.

The version-independent patch in `patches/newlib-disable-internal-asserts.patch`
also removes the unconditional allocation assertion in newlib's `eBalloc`
helper. The verifier scans every installed newlib archive for internal
`__assert_func` references so changes in later newlib releases fail the build.

Archive members are the smallest removable unit. Both libc and libgloss
archives (including `libnosys` and the semihosting libraries) are sanitized. If
a new newlib version puts a forbidden symbol in an object containing other
public functions, those functions are removed with it and recorded in
`share/ardupilot-toolchain/removed-newlib-members.txt`. The post-build verifier
scans every installed target archive and fails if any forbidden definition
survives.

This complements ArduPilot's existing link-time blacklist. It does not replace
the wrappers and stubs in `Tools/ardupilotwaf/chibios.py` and
`libraries/AP_HAL_ChibiOS/hwdef/common`.

## ArduPilot compatibility

ArduPilot's ChibiOS build passes both `--specs=nano.specs` and
`--specs=nosys.specs`. Accordingly, this build produces:

- full and nano newlib for every R/M-profile multilib;
- `libstdc++_nano.a` and `libsupc++_nano.a`, not only `libc_nano.a`;
- the nano-specific `newlib.h` selected by `nano.specs`;
- libgloss/nosys specifications while leaving low-level calls for ArduPilot's
  own ChibiOS stubs; and
- GDB configured against the selected host Python 3 interpreter.

The verifier compiles representative Cortex-M0, M3, M4F, and M7 programs and
links a nano/nosys M7 program that calls `rand()` and floating-point
`snprintf()`, using the same library selection as waf. A real firmware build is
still required before adopting a new compiler because compiler diagnostics,
code generation, and firmware size can change between GCC releases:

```sh
export PATH="$PWD/Tools/scripts/toolchain/_build/install/ardupilot-arm-none-eabi-16.1.0/bin:$PATH"
./waf configure --board CubeOrange
./waf copter
```

## Individual checks

The helper scripts can also be run directly:

```sh
Tools/scripts/toolchain/scripts/check-host.sh
Tools/scripts/toolchain/scripts/verify-toolchain.sh \
    Tools/scripts/toolchain/_build/install/ardupilot-arm-none-eabi-16.1.0 \
    Tools/scripts/toolchain/config/forbidden-newlib-symbols.txt
Tools/scripts/toolchain/tests/run.sh
```

The installed `share/ardupilot-toolchain/manifest.txt` captures the selected
source versions, multilibs, and Python version. The neighboring removal report
records every newlib archive member stripped by the safety policy.
