# AP_HAL_Zephyr Board README: native_sim

Board bring-up reference for the Zephyr host-native simulation target.

- Zephyr board name: `native_sim/native/64` (64-bit LP64 variant)
- HAL board: `HAL_BOARD_ZEPHYR`
- waf board name: `native_sim`

`../../README.md` covers build rules and layout. This
document only covers what is specific to native_sim.

## 1) What native_sim Is

native_sim is Zephyr's built-in host-native simulation board. It compiles
Zephyr and the application into a Linux ELF that runs directly on the
development host — no ARM toolchain, no cross-compilation, no hardware.

It is the successor to native_posix. This port uses the 64-bit variant.
`modules/zephyr/boards/native/native_sim/board.yml` declares one SoC (`native`)
with one variant (`64`), which is what `native_sim/native/64` selects. The
generated Kconfig for this board has `CONFIG_ARCH_POSIX=y` and
`CONFIG_64BIT=y`.

Board definition files come from Zephyr itself
(`modules/zephyr/boards/native/native_sim/`). This tree adds no board
directory for native_sim — only a Kconfig fragment and a devicetree overlay
(section 5).

### 1.1 How this differs from ArduPilot SITL

|                       | native_sim                       | ArduPilot SITL              |
| --------------------- | -------------------------------- | --------------------------- |
| What runs natively    | Full Zephyr RTOS + AP_HAL_Zephyr | ArduPilot core only         |
| HAL layer             | AP_HAL_Zephyr (real HAL path)    | AP_HAL_SITL                 |
| Peripheral simulation | Zephyr virtual drivers           | ArduPilot sim models        |
| Purpose               | Test the Zephyr HAL layer itself | Test ArduPilot flight logic |

native_sim exercises the actual AP_HAL_Zephyr driver code path, which makes it
useful for compile-and-boot checks without target hardware. It is not a flight
simulator: `AP_SIM_ENABLED = 0` is set by the board class, so none of
ArduPilot's sensor simulation models are compiled in.

## 2) Current Status

Builds and runs. native_sim is one of the four current maintained Zephyr
targets; see `../../README.md`.

The long-standing build failure was fixed by commit `c9b8f8296a`
("AP_HAL_Zephyr: restore native_sim build - host libc, not POSIX shim +
picolibc"). The root cause was never DroneCAN-specific: the base `prj.conf`
enables `CONFIG_POSIX_SYSTEM_INTERFACES`, which selects
`NATIVE_LIBC_INCOMPATIBLE` and forces picolibc — but the ArduPilot half of a
native_sim build is compiled by the host g++ against glibc's C++ headers, so
picolibc's `sys/cdefs.h` shadowed glibc's and every translation unit touching
`<cmath>` died with "`__BEGIN_DECLS` does not name a type". The board fragment
now disables that shim, so the libc choice falls back to its `NATIVE_BUILD`
default, `EXTERNAL_LIBC` = host glibc.

Observed when running the built binary: Zephyr boots, the AP_HAL_Zephyr threads
(monitor, timer, io, rcin, rcout, storage) all start, ZMS storage mounts on the
flash simulator and reads back its chunks, `callbacks->setup()` returns, and the
main loop ticks. The console `LOOPRATE` line reported `loop_hz=284` in that run.

That loop rate is a host-scheduled, simulated-time figure. It is not comparable
to the hardware loop-rate requirement and must not be quoted as one.

## 3) Prerequisites

native_sim needs no ARM cross-compiler.

Required:

- Host C/C++ toolchain (gcc/g++, e.g. Ubuntu `build-essential`)
- `cmake` and `ninja` on PATH
- Zephyr source, either via `ZEPHYR_BASE` or a populated `modules/zephyr`
  (see `../../README.md`)

Not required:

- `arm-none-eabi-gcc` or any cross toolchain
- J-Link, pyOCD, LinkServer or any flash programmer
- Physical hardware of any kind

Toolchain selection is automatic:

- `class native_sim` in `Tools/ardupilotwaf/boards.py` overrides
  `configure_toolchain()` to set `cfg.env.TOOLCHAIN = 'native'`, skipping
  cross-compiler setup.
- `Tools/ardupilotwaf/zephyr.py` sets `ZEPHYR_TOOLCHAIN_VARIANT = 'host'` for
  any `ZEPHYR_BOARD` starting with `native_sim`, so Zephyr's CMake build uses
  host GCC too.

On 32-bit vs 64-bit: the board class hardcodes the 64-bit variant. The plain
32-bit `native_sim` board is not configured in this tree and is not exercised;
selecting it would mean changing `env.ZEPHYR_BOARD` in `boards.py` and having a
32-bit host libc available.

## 4) Simulated Peripherals

What Zephyr provides on this board, and what AP_HAL_Zephyr actually binds to.
The "AP_HAL_Zephyr" column matters: several emulated controllers exist in the
native_sim devicetree that the HAL does not look up, because its device lookups
are written against real SoC node labels.

| Peripheral                | Zephyr side                                                                                                                                                                                    | AP_HAL_Zephyr side                                                                                                                                                                                                            |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| UART                      | `CONFIG_UART_NATIVE_PTY=y`; `uart0`/`uart1` are `zephyr,native-pty-uart`. The overlay gives `uart0` `on-stdinout`, so console output goes to the process stdout.                               | SERIAL0 maps to `uart0` via the generated `HAL_UART_DT_DEVICE_LOOKUP`; `HAL_UART_NUM_SERIAL_PORTS` is 1. Higher serial indexes print "UART: device lookup returned nullptr".                                                  |
| SPI                       | `spi0` (`zephyr,spi-emul-controller`) exists in the devicetree, but `CONFIG_SPI_EMUL` `depends on EMUL` and `CONFIG_EMUL` is not enabled by any fragment, so the emulator driver is not built. | No SPI devices. `hwdef.dat` declares no `IMU`/`BARO`/`COMPASS` probes.                                                                                                                                                        |
| I2C                       | Same as SPI: `i2c0` (`zephyr,i2c-emul-controller`) is present, `CONFIG_I2C_EMUL` also `depends on EMUL`, so it is not built.                                                                   | No I2C devices.                                                                                                                                                                                                               |
| GPIO                      | `CONFIG_GPIO=y`; `gpio0` is `zephyr,gpio-emul` and `CONFIG_GPIO_EMUL=y` comes on automatically from that node.                                                                                 | `hwdef.dat` maps no GPIO pins.                                                                                                                                                                                                |
| ADC                       | `CONFIG_ADC_EMUL=y`; `adc0` is `zephyr,adc-emul`.                                                                                                                                              | `AnalogIn::init()` only resolves `lpadc1`/`lpadc2`, which do not exist here, so `_adc_ready` stays false and analog reads are unavailable.                                                                                    |
| Flash / parameter storage | `CONFIG_FLASH_SIMULATOR=y`; `flashcontroller0` is `zephyr,sim-flash` with a `fixed-partitions` layout. `CONFIG_ZMS=y` from the base `prj.conf`.                                                | `Storage` mounts ZMS on the simulated flash. Observed at runtime: offset `0xfc000`, 4096-byte sectors, 16 sectors, 32 chunks read back.                                                                                       |
| FatFS volume              | `CONFIG_FAT_FILESYSTEM_ELM=y`, `CONFIG_DISK_ACCESS=y`, `CONFIG_DISK_DRIVER_RAM=y`; the overlay adds a `zephyr,ram-disk` node named `SD`, 2048 x 512 B = 1 MB.                                  | The name matches `ZEPHYR_DISK_NAME` in `sdcard.cpp`. Its primary purpose is structural: FatFS generates `FF_VOLUMES` from devicetree disk nodes and hard-errors at zero, so at least one disk node is required for the build. |
| CAN                       | `CONFIG_CAN=y`, `CONFIG_CAN_LOOPBACK=y`; `can_loopback0` (`zephyr,can-loopback`) is enabled.                                                                                                   | Compile-only. `CANIface::init()` resolves only `flexcan1`/`flexcan2`, which do not exist on native_sim, so `init()` returns false and no CAN interface comes up. The DroneCAN code still compiles and links.                  |
| PWM                       | `CONFIG_PWM=n`; the devicetree has no PWM nodes at all.                                                                                                                                        | `RCOutput` falls through to its no-map branch; every channel entry is null and `_apply_channel()` skips them.                                                                                                                 |
| USB CDC                   | No `usb_cdc_acm0` node.                                                                                                                                                                        | Not available.                                                                                                                                                                                                                |

`hwdef.dat` sets `HAL_INS_DEFAULT` to `HAL_INS_NONE`, so no IMU backend is
probed and the vehicle boots cleanly with no inertial sensor.

## 5) Implementation Details

### 5.1 Board class in boards.py

`class native_sim(zephyr_board)` in `Tools/ardupilotwaf/boards.py`:

- overrides `configure_toolchain()` to use the native host toolchain
- `env.ZEPHYR_BOARD = "native_sim/native/64"`, with `ZEPHYR_MFR` and
  `ZEPHYR_SOC` both empty (no SoC-specific HAL layer)
- `CONFIG_HAL_BOARD = HAL_BOARD_ZEPHYR`,
  `CONFIG_HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_NONE`, `AP_SIM_ENABLED = 0`
- `self.with_can = True` — enables the DroneCAN include paths
- parses `libraries/AP_HAL_Zephyr/hwdef/native_sim/hwdef.dat` and promotes
  `HAL_NUM_CAN_IFACES` from it into `cfg.define()`
- `CANARD_MULTI_IFACE = 1`, `CANARD_IFACE_ALL = 0x1`,
  `CANARD_ENABLE_CANFD = 0`, `CANARD_ENABLE_ASSERTS = 1`, `CANARD_64_BIT = 1`

### 5.2 Toolchain variant in zephyr.py

`Tools/ardupilotwaf/zephyr.py` checks
`env.ZEPHYR_BOARD.startswith('native_sim')` and sets
`ZEPHYR_TOOLCHAIN_VARIANT = 'host'` instead of `gnuarmemb`/`zephyr`.

### 5.3 Kconfig fragment

`libraries/AP_HAL_Zephyr/zephyr/boards/native_sim_native_64.conf` is merged on
top of `libraries/AP_HAL_Zephyr/zephyr/prj.conf` (fragment discovery accepts the
board-variant filename). It:

- re-enables `CONFIG_CONSOLE=y` — native_sim's board Kconfig selects
  `POSIX_ARCH_CONSOLE`, which depends on it
- disables `CONFIG_PINCTRL` and `CONFIG_CLOCK_CONTROL` (SoC-specific, absent
  on native_sim)
- turns off the NXP drivers the base config brings in (`UART_MCUX_LPUART`,
  `SPI_NXP_LPSPI`, `I2C_MCUX_LPI2C`, `GPIO_MCUX_IGPIO`, `ADC_MCUX_LPADC`) and
  selects the native/emulated ones instead
- disables `CONFIG_PWM`
- keeps the filesystem layer on and enables the RAM disk backing it
- enables `CONFIG_CAN` and `CONFIG_CAN_LOOPBACK`
- turns off `CONFIG_POSIX_SYSTEM_INTERFACES` and its children
  `CONFIG_POSIX_TIMERS` / `CONFIG_POSIX_DEVICE_IO` — this is the fix described
  in section 2. The children must be set `=n` explicitly too, or their base-layer
  `=y` assignments become ineffective-assignment configure failures once the
  parent is off.
- `CONFIG_REBOOT=y` for `Scheduler::reboot()` -> `sys_reboot()`
- `CONFIG_TEST_RANDOM_GENERATOR=y` for `Util::get_random_vals()` ->
  `sys_rand_get()`; there is no entropy hardware on the simulator

The fragment's `CONFIG_SPI_EMUL=y` / `CONFIG_I2C_EMUL=y` lines do not currently
take effect — both symbols `depend on EMUL` and nothing enables `CONFIG_EMUL`
(see the SPI and I2C rows in section 4).

### 5.4 Devicetree overlay

`libraries/AP_HAL_Zephyr/zephyr/boards/native_sim_native_64.overlay`:

- `&uart0 { on-stdinout; }` routes the console UART to the process
  stdin/stdout, so `hal.console->printf()` output appears on the terminal
- adds a `zephyr,ram-disk` node named `SD` (512-byte sectors x 2048 = 1 MB)
  so FatFS has at least one devicetree disk to generate `FF_VOLUMES` from

### 5.5 HAL_NUM_CAN_IFACES and CANARD_MULTI_IFACE

`libraries/AP_HAL/board/zephyr.h` defines `HAL_NUM_CAN_IFACES` under an
`#ifndef` guard, so the value the board class pushes through `ap_config.h`
(1, from `hwdef.dat`) wins.

`CANARD_MULTI_IFACE = 1` is required even with a single interface:
`CanardTxQueueItem::iface_mask` is declared inside `#if CANARD_MULTI_IFACE`
in `modules/DroneCAN/libcanard/canard.h`, but the TX loop in
`libraries/AP_DroneCAN/AP_Canard_iface.cpp` reads and clears
`txf->iface_mask` outside any such guard. Without the define the build does not
compile.

## 6) Configure and Build

```bash
./waf configure --board native_sim
./waf copter -j12
```

See `../../README.md` for when a re-`configure` is actually needed
— those rules apply to native_sim exactly as to the other Zephyr boards.

Artifacts:

| Path                                               | What it is                                                                                       |
| -------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| `build/native_sim/zephyr_build/zephyr/zephyr.exe`  | The runnable host executable (x86-64 ELF, dynamically linked). This is the build output you run. |
| `build/native_sim/zephyr_build/zephyr/zephyr.elf`  | Relocatable object from the Zephyr link stage, not directly runnable                             |
| `build/native_sim/zephyr_build/zephyr/libzephyr.a` | Zephyr static library                                                                            |
| `build/native_sim/lib/bin/libarducopter.a`         | Vehicle-specific ArduPilot archive                                                               |
| `build/native_sim/lib/libArduCopter_libs.a`        | Shared ArduPilot library archive                                                                 |

`build/native_sim/bin/` stays empty. That is normal for every Zephyr board in
this tree: waf compiles ArduPilot into static archives under `lib/`, and the
final link is performed by Zephyr's CMake build with `ARDUPILOT_LIB` pointing at
them.

No `.apj` is produced and there is no upload step — `zephyr.py`'s upload task
returns early for `native_sim` boards, since a host executable is not firmware.

## 7) Running the Output Binary

```bash
./build/native_sim/zephyr_build/zephyr/zephyr.exe
```

The process boots Zephyr, initialises AP_HAL_Zephyr, and runs the ArduPilot
scheduler loop. It will not arm or fly — there are no sensor inputs — but it
exercises the scheduler, storage, and console paths.

Console output goes to stdout (via the overlay's `on-stdinout` on `uart0`).
Expect repeated "UART: device lookup returned nullptr for serial N" lines for
every serial port above SERIAL0; only one UART is mapped.

SIGINT (Ctrl-C) and SIGTERM both stop it cleanly; native_sim prints a
`Stopped at <n>s` line on the way out.

## 8) Known Limitations

Structural to this target:

- No real sensor data. `HAL_INS_DEFAULT` is `HAL_INS_NONE`, and no baro or
  compass probe lists are declared.
- No SPI or I2C devices — the emulated bus controllers are not compiled in
  (section 4), and nothing is attached to them anyway.
- No RC input and no motor output; there is no PWM hardware in the devicetree.
- No USB CDC.
- No analog input; `AnalogIn` binds only to NXP LPADC node labels.
- CAN is compile-only. The loopback controller exists on the Zephyr side but
  `CANIface` never binds to it.
- Real-time behaviour is not representative: timing comes from the host OS
  scheduler and Zephyr's simulated clock. Loop rates measured here say nothing
  about hardware performance.
- Only the 64-bit board variant is configured.

## 9) Intended Use

native_sim is a compile-and-boot check for the AP_HAL_Zephyr code path that
needs no target hardware. A passing build-and-run confirms:

- the HAL compiles and links against the current Zephyr headers
- the Zephyr kernel initialises without assertion failures
- the AP_HAL_Zephyr scheduler brings up all its threads
- `Storage` mounts and reads ZMS
- `AP_Vehicle::setup()` completes and the main loop ticks

It does not validate peripheral wiring, driver timing, DMA behaviour or sensor
correctness. Those need real hardware — the primary target is `mr_vmu_rt1176`.

No CI job currently builds native_sim: nothing under `.github/workflows/`
references any Zephyr board. Running it is a manual step today.

## 10) Quick Bring-up Checklist

1. Ensure `ZEPHYR_BASE` is exported or `modules/zephyr` is populated
2. `./waf configure --board native_sim`
3. `./waf copter -j12`
4. Confirm `build/native_sim/zephyr_build/zephyr/zephyr.exe` exists
5. Run it and confirm the boot banner, the ZMS mount line, and a `LOOPRATE`
   line with a non-zero `loop_hz`
6. Ctrl-C to exit
