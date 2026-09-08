# MR-VMU-RT1176 Flight Controller (Zephyr)

The MR-VMU-RT1176 is NXP's open-source reference design implementing the
Pixhawk FMUv6X-RT standard: an FMU module on an NXP carrier board. This is the
primary target of the ArduPilot-on-Zephyr port.

- waf board: `mr_vmu_rt1176`
- Zephyr board target: `mr_vmu_rt1176/mimxrt1176/cm7`
- HAL board: `HAL_BOARD_ZEPHYR`, HAL class `HAL_Zephyr`, namespace `Zephyr::`
- `APJ_BOARD_ID` 35 (same as the PX4 `px4_fmu-v6xrt` bootloader reports)

## Features

- NXP MIMXRT1176 (Cortex-M7 at 1 GHz, plus an unused Cortex-M4)
- 64 MB external FlexSPI Octal-DDR NOR (MX25UM51345G), XIP, no internal program
  flash
- Three SPI IMU slots: one onboard, two on a shock-mounted daughterboard over
  FPC. Fitted parts vary by board revision, so the hwdef probes rather than
  assumes
- Two BMP388-family barometers (onboard on I2C1, offboard on I2C2)
- BMM150 compass on the offboard daughterboard
- 9 serial ports including USB CDC, 3 I2C buses, 3 SPI buses, 2 CAN buses
- 12 FMU-direct PWM outputs, 8 of them DShot-capable pads. No IOMCU is fitted
- Dual-path RC input: single-wire SBUS/CRSF on LPUART6 plus PPM-SUM pulse
  capture
- microSD slot on USDHC1, FatFs logging
- Two SMBus smart-battery power inputs (POWER1, POWER2). There is no analog
  battery sense on this board
- Two USB CDC-ACM interfaces: MAVLink and mcumgr SMP
- 100Base-T1 Ethernet, not implemented

There is no safety switch (`HAL_HAVE_SAFETY_SWITCH 0`).

## Where the configuration actually lives

| Topic                                           | File                                                                                                      |
| ----------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| Sensors, buses, ArduPilot-layer config          | `hwdef.dat` (this directory)                                                                              |
| Wiring, connectors, schematic-derived nets      | `../../zephyr/boards/arm/mr_vmu_rt1176/docs/WIRING.md`                                                    |
| Pin mux, clocks, peripheral enables, memory map | `../../zephyr/boards/arm/mr_vmu_rt1176/mr_vmu_rt1176_mimxrt1176_cm7.dts` and `mr_vmu_rt1176-pinctrl.dtsi` |
| Kconfig                                         | `../../zephyr/prj.mr_vmu_rt1176.conf`, plus `prj.nxprt1176.conf`, `prj.nxp.conf`, `prj.conf`              |
| Board class and waf glue                        | `Tools/ardupilotwaf/boards.py`, `Tools/ardupilotwaf/zephyr.py`                                            |
| Feature status against ChibiOS                  | `../../COMPARED_TO_CHIBIOS.md`                                                                            |
| Standing policies, submodule and west policy    | `../../PROCESS.md`                                                                                        |
| Bootloader A/B, and why there is no verified boot | `../../BOOTLOADER_SECURITY.md`                                                                            |
| Debugging tools and crash dumps                 | `../../DEBUGGING.md`                                                                                      |
| Reference PDFs (git-ignored, fetch locally)     | `../../zephyr/boards/arm/mr_vmu_rt1176/docs/README.md`                                                    |

## Processor and memory

- SoC `MIMXRT1176DVMAA` (`Kconfig.mr_vmu_rt1176`:
  `SOC_PART_NUMBER_MIMXRT1176DVMAA`)
- Cortex-M7 at 1 GHz (`Kconfig.defconfig`:
  `SYS_CLOCK_HW_CYCLES_PER_SEC = 1000000000`)
- Kernel tick rate 1 MHz (`CONFIG_SYS_CLOCK_TICKS_PER_SEC=1000000`, base
  `prj.conf`)
- Hard float: `-mfpu=fpv5-d16 -mfloat-abi=hard` in `boards.py`, matched by
  `CONFIG_FPU=y` and `CONFIG_FPU_SHARING=y`. Check with
  `arm-none-eabi-objdump -d <elf> | grep -c 'v..\.f32'`; a build with those
  flags missing links and runs, and contains zero VFP instructions
- The Cortex-M4 is not used. dual-core M4 is future work

Memory map, from the board DTS:

| Region                                   | Address      | Size    | Chosen as      |
| ---------------------------------------- | ------------ | ------- | -------------- |
| OCRAM                                    | `0x20200000` | 1024 KB | `zephyr,sram`  |
| ITCM                                     | `0x00000000` | 480 KB  | `zephyr,itcm`  |
| DTCM                                     | `0x20000000` | 32 KB   | `zephyr,dtcm`  |
| External FlexSPI NOR (MX25UM51345G, XIP) | `0x30000000` | 64 MB   | `zephyr,flash` |

ITCM and DTCM come out of the 512 KB FlexRAM, split by the `&flexram`
`flexram,bank-spec` property as 15 banks ITCM plus 1 bank DTCM. The `&itcm` and
`&dtcm` `reg` overrides must stay consistent with that bank-spec. Without the
property FlexRAM stays at its fuse default, which is unprogrammed.

Code runs XIP from the external NOR. Hot code is placed into ITCM by
`../../zephyr/itcm_hot_code.ld`, applied from `../../zephyr/CMakeLists.txt`.
ITCM is at 99.4% of 491,520 B, so there is no room for another round.

Flash partitions (`partitions` node under the FlexSPI NOR):

| Partition                     | Offset      | Size                  |
| ----------------------------- | ----------- | --------------------- |
| `boot_partition` (`mcuboot`)  | `0x000000`  | 128 KB                |
| `slot0_partition` (`image-0`) | `0x020000`  | 3 MB                  |
| `slot1_partition` (`image-1`) | `0x320000`  | 3 MB                  |
| `storage_partition`           | `0x620000`  | 58 MB - 128 KB - 1 MB |
| `coredump_partition`          | `0x3f00000` | 1 MB                  |

The application links at `0x30022000`: `CONFIG_FLASH_LOAD_OFFSET=0x22000` with
`CONFIG_NXP_IMXRT_BOOT_HEADER=n`, matching the resident bootloader's
`APP_LOAD_ADDRESS` plus `APP_VECTOR_OFFSET`.

`mr_vmu_rt1176.yaml` declares `ram: 2048` and `flash: 8192`. Those are Zephyr
twister metadata fields, not the real memory map. Use the table above.

## Status

`../../COMPARED_TO_CHIBIOS.md` carries per-feature status.

| Area                | State                                                                                   |
| ------------------- | --------------------------------------------------------------------------------------- |
| GPIO                | implemented (`GPIO.cpp`)                                                                |
| UART                | eDMA async API on every LPUART (`CONFIG_UART_ASYNC_API=y`, `dmas` on each `&lpuartN`)   |
| SPI                 | eDMA (`CONFIG_SPI_NXP_LPSPI_DMA=y`, `dmas` on `&lpspi1/2/3`)                            |
| I2C                 | eDMA (`dmas` on `&lpi2c1/2/3`)                                                          |
| AnalogIn            | LPADC1 (`CONFIG_ADC_MCUX_LPADC=y`)                                                      |
| RCInput             | SBUS/serial protocols on LPUART6 (SERIAL7) plus PPM-SUM edge/QTMR capture               |
| RCOutput            | 12 FlexPWM outputs (`HAL_PWM_COUNT 12`)                                                 |
| Storage             | BootROM-API flash backend (`CONFIG_AP_RT1176_ROMAPI_FLASH=y`), `HAL_STORAGE_SIZE 16384` |
| USB                 | device stack plus two CDC-ACM instances (MAVLink, mcumgr SMP)                           |
| CAN                 | classic CAN 2.0B on FlexCAN1/2, `HAL_NUM_CAN_IFACES 2`                                  |
| microSD and logging | USDHC1 plus FatFs (`CONFIG_FAT_FILESYSTEM_ELM=y`)                                       |
| DSP and gyro FFT    | CMSIS-DSP backed (`HAL_WITH_DSP 1`, `HAL_GYROFFT_ENABLED 1`)                            |
| Lua scripting       | enabled (`AP_SCRIPTING_ENABLED 1`)                                                      |
| Watchdog            | WDOG1 (`CONFIG_WATCHDOG=y`, `CONFIG_WDT_DISABLE_AT_BOOT=n`)                             |
| Coredump            | custom backend (`CONFIG_AP_RT1176_COREDUMP=y`) into `coredump_partition`                |
| mcumgr / SMP        | second USB CDC (`CONFIG_MCUMGR=y`, `zephyr,uart-mcumgr = &cdc_acm_uart1`)               |

Measured:

- IMU sampling: 8 kHz gyro FIFO on the ICM-42688-P (`INS_GYRO_RATE=3`),
  hardware-verified 2026-08-09
- Main loop: 550-560 Hz hardware-verified 2026-08-12, against a 400 Hz
  requirement. `hwdef.dat` sets `SCHEDULER_DEFAULT_LOOP_RATE 600`

SPI, I2C and UART all run through eDMA. The `&lpspi1/2/3` DTS comment blocks
are a historical narrative ending at the 2026-08-09 single-chunk full-duplex
fix; the live settings are the node properties and the Kconfig fragments, not
the prose above them.

Open on this board: CAN-FD, DShot, Ethernet, dual-core M4. `CONFIG_ZMS is not
set`; the ROM-API flash backend is what storage uses.

Still backed by `AP_HAL_Empty` (`AP_HAL_Zephyr_Namespace.h`): `Flash`,
`OpticalFlow`, `WSPIDevice` / `WSPIDeviceManager`. `DSP` is not a stub here,
`HAL_WITH_DSP 1` selects the real CMSIS-DSP-backed `Zephyr::DSP`.

The coredump path is implemented, but its fault to reset to
`@SYS/crash_dump.bin` retrieval cycle is not hardware-verified.

## Building

First time only:

```bash
./Tools/scripts/zephyr_get_prerequisites.sh
```

It installs host packages, populates `modules/zephyr`, and syncs the Zephyr
dependency submodules. It refuses to run outside the repository root. Also
required: `cmake` and `ninja` in `PATH`, the normal ArduPilot Python venv, and
Zephyr sources found via `ZEPHYR_BASE` or `modules/zephyr`. If Zephyr cannot be
found, waf still runs and logs `Zephyr: ZEPHYR_BASE/modules/zephyr not found,
skipping Zephyr side build`.

Dependencies are git submodules, not west. Do not run `west` in this repository
root, see [PROCESS.md](../../PROCESS.md).

```bash
./waf configure --board mr_vmu_rt1176
./waf copter -j12
```

Optional:

```bash
./waf configure --board mr_vmu_rt1176 --enable-stats   # per-thread CPU LOAD% in @SYS/threads.txt
./waf --targets=tool/CPUInfo -j12                      # small binary, quick per-operation checks
```

Artifacts under `build/mr_vmu_rt1176/`:

| Path                              | What                                         |
| --------------------------------- | -------------------------------------------- |
| `zephyr_build/zephyr/zephyr.elf`  | final ELF with ArduPilot linked in           |
| `zephyr_upload.bin`               | padded flat binary for the bootloader        |
| `zephyr_upload.apj`               | uploader.py image                            |
| `zephyr_build/zephyr/libzephyr.a` | the Zephyr side of the link                  |
| `hwdef.h`                         | generated from `hwdef.dat` at configure time |
| `compile_commands.json`           | build database                               |

There is no `bin/arducopter` for this board. That path is produced only by the
host-executable target (`native_sim`). `_run_ap_final_link()` in
`Tools/ardupilotwaf/zephyr.py` produces `zephyr.elf` after the ArduPilot archive
exists.

### Build gotchas

**Edit `hwdef.dat`, re-run `./waf configure`.** `configure` regenerates
`build/<board>/hwdef.h`, but waf has no dependency edge from AP sources to that
generated header, so a plain rebuild keeps the old content and the link
succeeds. Symptom: a sensor that will not appear.

**Edit `prj*.conf`, a plain `./waf copter` updates the Zephyr side.** The
fragment merge is a build step (`_write_autogen_prj_conf()`, called from
`def build()` in `Tools/ardupilotwaf/zephyr.py`), not a configure step. Check
with:

```bash
grep <SYMBOL> build/mr_vmu_rt1176/zephyr_build/ardupilot_prj_autogen.conf
```

AP-side C++ gets Kconfig through `-imacros autoconf.h` (`boards.py`), which
waf's header scanner does not follow. Any symbol that reaches AP translation
units, directly or through inlined kernel functions such as the tick-rate math,
needs:

```bash
rm -rf build/mr_vmu_rt1176/libraries build/mr_vmu_rt1176/ArduCopter
```

**A DTS edit that adds or removes nodes** renumbers devicetree ordinals and can
leave stale objects referencing symbols that no longer exist. That fails loudly
(`undefined reference to '__device_dts_ord_NNN'`); wipe the build directory when
you see it. A DTS edit that only changes a property needs nothing, ninja tracks
those files.

**Check the ELF exists before drawing conclusions from it.** After a failed
build, `objdump` and `nm` on a missing file return 0, which reads like a
measurement.

**`--bootloader` switches the whole build** between bootloader and app.
Reconfigure before each. Both write
`build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.bin`, and `zephyr_upload.apj`
is regenerated per build, so keep durable copies of anything you want to
re-upload later. Before uploading, check the apj's `image_size` matches an app
and not a bootloader; measured 2026-07-29 those were 169080 and roughly 83000
bytes respectively.

## UART mapping

`SERIAL_ORDER` in `hwdef.dat` is
`OTG1 USART4 USART8 USART3 USART5 USART10 USART11 USART6 LPUART1`, generating
`HAL_UART_DT_DEVICE_LOOKUP` with `HAL_UART_NUM_SERIAL_PORTS 9`:

| ArduPilot | Device         | Function                                                        |
| --------- | -------------- | --------------------------------------------------------------- |
| SERIAL0   | `usb_cdc_acm0` | USB CDC ACM, MAVLink (GCS)                                      |
| SERIAL1   | `lpuart4`      | TELEM1                                                          |
| SERIAL2   | `lpuart8`      | TELEM2                                                          |
| SERIAL3   | `lpuart3`      | GPS1 (`DEFAULT_SERIAL3_PROTOCOL 5`)                             |
| SERIAL4   | `lpuart5`      | GPS2                                                            |
| SERIAL5   | `lpuart10`     | TELEM3                                                          |
| SERIAL6   | `lpuart11`     | External, DTS node not enabled                                  |
| SERIAL7   | `lpuart6`      | RC input, single-wire (`RCInput.cpp` hardcodes `hal.serial(7)`) |
| SERIAL8   | `lpuart1`      | Debug connector, also `zephyr,console` and `zephyr,shell-uart`  |

`lpuart11` has no enabled node and no pinctrl group in the board DTS, so SERIAL6
resolves to nothing at runtime even though the slot exists.

Every enabled LPUART carries `dmas`/`dma-names` and runs the async eDMA UART
API.

`zephyr,console = &lpuart1`. [PROCESS.md](../../PROCESS.md) records a standing
policy (2026-08-15) that consoles move to USB CDC on all boards and lists this
board as not yet compliant. `CONFIG_LOG=n` here comes from `prj.nxprt1176.conf`
as a sanctioned case-by-case disable under the same policy, with the
cbprintf-hang rationale in that file.

## RC input

Two paths, as on Pixhawk-standard boards:

- Serial protocols (SBUS/CRSF and the rest) on `&lpuart6`: `single-wire`,
  `current-speed = <100000>`, eDMA. Reaches ArduPilot as SERIAL7 and is scanned
  by `AP_RCProtocol`.
- PPM-SUM (CPPM) by pulse capture: `rcin-gpios = <&gpio2 8 GPIO_ACTIVE_HIGH>` in
  the DTS `zephyr,user` node, with hardware capture via `pwms = <&qtmr1 0 0 0>`
  (`CONFIG_AP_RCIN_PWM_CAPTURE=y`, `CONFIG_PWM_CAPTURE=y`).

Both paths land on the same pad, `EMC_B1_40`, so `hwdef.dat` sets
`RCIN_PULSE_GPIO_SHARES_UART_PAD 1` and `RCInput.cpp` runs a runtime MUX_MODE
arbiter between them. Register-level detail is in
[RC input pad arbitration](#rc-input-pad-arbitration) below.

The board does have a dedicated PPM pad, `FMU_PPM_INPUT` = ball M2 =
`GPIO_EMC_B2_12` = `GPIO_MUX2_IO22` = gpio2 pin 22. It is not fed by this
carrier's RCIN position. Measured 2026-08-13 with the receiver in the carrier's
RCIN 3-pin servo-rail position, the CPPM pulse train arrived on `EMC_B1_40`
(about 470 transitions/s, 9 pulses at 50 Hz) and `EMC_B2_12` showed nothing. A
carrier that routes RC-IN conditioning to both nets could use pin 22 with
`RCIN_PULSE_GPIO_SHARES_UART_PAD` removed, in which case `RCInput.cpp` takes its
plain `gpio_pin_configure_dt()` path and no arbitration happens.

## PWM output

12 FMU-direct channels (`HAL_PWM_COUNT 12`, `NUM_SERVO_CHANNELS 16`). No IOMCU
is fitted (`HAL_WITH_IO_MCU 0`); the schematic makes it an optional part and our
hardware does not populate it. Channel map from `RCOutput.cpp`:

| Output | Zephyr PWM node | Pad         |
| ------ | --------------- | ----------- |
| 1      | `flexpwm1_pwm0` | `EMC_B1_23` |
| 2      | `flexpwm1_pwm1` | `EMC_B1_25` |
| 3      | `flexpwm1_pwm2` | `EMC_B1_27` |
| 4      | `flexpwm2_pwm0` | `EMC_B1_06` |
| 5      | `flexpwm2_pwm1` | `EMC_B1_08` |
| 6      | `flexpwm2_pwm2` | `EMC_B1_10` |
| 7      | `flexpwm2_pwm3` | `EMC_B1_19` |
| 8      | `flexpwm3_pwm0` | `EMC_B1_29` |
| 9      | `flexpwm3_pwm1` | `EMC_B1_31` |
| 10     | `flexpwm3_pwm3` | `EMC_B1_21` |
| 11     | `flexpwm4_pwm0` | `EMC_B1_00` |
| 12     | `flexpwm4_pwm1` | `EMC_B1_02` |

DShot is not implemented, though pads for channels 1-8 have a FlexIO pinctrl
state declared (`pinmux_dshot_fmu_ch1_8`) and `ap_rcout_mux` switches to it, see
[The DShot pinctrl precedent](#the-dshot-pinctrl-precedent).

## SPI devices

`&lpspi1`, `&lpspi2` and `&lpspi3` are enabled, each with `dmas`/`dma-names` and
`clock-frequency = <DT_FREQ_M(10)>`. `&lpspi4` is `status = "disabled"`. There
is no LPSPI5 on this board.

`SPIDEV` entries in `hwdef.dat`, generated into `HAL_SPI_DT_SPEC_DECLS` and
`HAL_SPI_DT_SPEC_LOOKUP`, all at 8 MHz, SPI mode 3:

| Device name        | Bus      | CS index     |
| ------------------ | -------- | ------------ |
| `imu_sensor1`      | `lpspi1` | `cs-gpios` 0 |
| `imu_sensor2`      | `lpspi2` | `cs-gpios` 0 |
| `imu_sensor3`      | `lpspi3` | `cs-gpios` 0 |
| `imu_sensor3_gyro` | `lpspi3` | `cs-gpios` 1 |

Devices are named by bus, not by part, because the fitted silicon varies by
board revision (`WIRING.md`, "Revision variance").

The generated `HAL_INS_PROBE_LIST` offers each bus to `Invensensev3` and then
`Invensense`:

```text
IMU Invensensev3 SPI:imu_sensor1 ROTATION_NONE
IMU Invensense   SPI:imu_sensor1 ROTATION_NONE
IMU Invensensev3 SPI:imu_sensor2 ROTATION_NONE
IMU Invensense   SPI:imu_sensor2 ROTATION_NONE
IMU Invensensev3 SPI:imu_sensor3 ROTATION_NONE
IMU Invensense   SPI:imu_sensor3 ROTATION_NONE
```

Detection decides what binds. Four constraints on that list:

- **Probes are not side-effect free.** `Invensensev2`'s whoami check writes a
  bank-select register, which corrupts a live ICM-426xx and makes the working
  IMU disappear. Only add a backend that is known non-destructive on the fitted
  parts.
- **`lpspi1`'s onboard IMU is an ICM-42686-P**, WHO_AM_I `0x44`, identified
  2026-08-09. ArduPilot's `Invensensev3` has no `0x44` entry, so it does not
  bind. Supporting it means adding `0x44` plus its scale table to
  `AP_InertialSensor_Invensensev3`, not adding hwdef probe lines.
- **`lpspi2` carries the ICM-42688-P** (`0x47`) and is the IMU the loop-rate
  numbers are measured on.
- **BMI088 on `lpspi3` is declarable but disabled.** The two-device probe line
  exists, commented out in `hwdef.dat`, and both chip selects are wired. It was
  disabled 2026-08-09 because its DeviceBus thread measured 25-26% of a core for
  a redundant second IMU instance.

Fast sampling is off by default (`HAL_DEFAULT_INS_FAST_SAMPLE 0`) on
bus-saturation grounds; the measurement is in the `hwdef.dat` comment.

`CONFIG_AP_SPI_PROBE_DIAG` is a read-only boot scan for "what is on this bus"
questions. It is `=n` in `prj.mr_vmu_rt1176.conf`.

## I2C devices

`&lpi2c1`, `&lpi2c2` and `&lpi2c3` are enabled at `I2C_BITRATE_FAST` (400 kHz),
each with `dmas`. `I2C_ORDER I2C1 I2C2 I2C3` gives `HAL_I2C_BUS_COUNT 3`:

| AP bus | Zephyr node | Role                                              |
| ------ | ----------- | ------------------------------------------------- |
| 0      | `lpi2c1`    | external / GPS1 connector, POWER1 SMBus           |
| 1      | `lpi2c2`    | onboard baro                                      |
| 2      | `lpi2c3`    | offboard daughtercard (baro plus compass, shared) |

Declared devices (`HAL_I2C_DEVICES_LIST`):

| Driver   | Bus | Address | Notes                                                   |
| -------- | --- | ------- | ------------------------------------------------------- |
| `BMP388` | 1   | `0x76`  | onboard baro                                            |
| `BMP388` | 2   | `0x77`  | offboard baro; the AP BMP388 driver also handles BMP390 |
| `BMM150` | 2   | `0x10`  | compass, marked external, `ROTATION_NONE`               |

`CONFIG_AP_I2C_PROBE_DIAG=n`. The full I2C probe scan ate most of the CPU and
stretched boot to over 170 s; with it off, boot is 4 s.

## Analog inputs and power

- `&lpadc1` is enabled, `&lpadc2` is `status = "disabled"`. `AnalogIn.cpp`
  compiles each under its own `DT_NODE_HAS_STATUS(...)` guard, so only LPADC1 is
  live.
- `hwdef.dat` sets `HAL_HAVE_BOARD_VOLTAGE 1` and `HAL_HAVE_SERVO_VOLTAGE 1`.
- There is no analog battery sense. POWER1 and POWER2 are SMBus smart-battery
  connectors: the BMS reports voltage and current over I2C and is configured at
  runtime with `BATT_MONITOR` parameters. The analog nets that exist are
  internal rail monitors. The DTS `aliases` node states this, having previously
  carried wrong `adc-batt0`/`adc-batt1` aliases.

## USB

`&usb1` (nxp,ehci) is enabled with `phy-handle = <&usbphy1>`. `&usbphy1` must
stay enabled or enumeration fails with "device not accepting address".
`CONFIG_USB_DEVICE_STACK_NEXT=y` with `CONFIG_USBD_CDC_ACM_CLASS=y`; VID/PID and
descriptor strings come from `USBD_DEVICE_DEFINE` and `USBD_DESC_*` in
`../../zephyr/src/main.cpp`. The board enumerates high-speed as
`27b1:0004 ArduPilot mr_vmu_rt1176` with the OCOTP UID as its serial.

| Node            | Interface label | Use                                             |
| --------------- | --------------- | ----------------------------------------------- |
| `usb_cdc_acm0`  | `MAVLink`       | SERIAL0 (GCS link), and `GPIO::usb_connected()` |
| `cdc_acm_uart1` | `SMP`           | `zephyr,uart-mcumgr` transport                  |

`Tools/scripts/61-ardupilot-rt1176.rules` renames the two host CDC nodes to
`-if-mavlink` and `-if-smp`, plus short `/dev/serial/by-ap/{mavlink,smp}` forms,
using the DTS interface `label` strings.

## CAN

`&flexcan1` and `&flexcan2` are enabled at `bitrate = <1000000>`. `hwdef.dat`
sets `HAL_NUM_CAN_IFACES 2` and `CAN_ORDER 1 2`; `boards.py` sets
`with_can = True` with `CANARD_IFACE_ALL = 0x3` and `CANARD_ENABLE_CANFD = 0`.

`CANIface.cpp` is a classic-CAN driver. CAN-FD is not supported. DroneCAN is the
only CAN protocol in scope: `AP_PICCOLOCAN_ENABLED 0` and
`AP_FETTEC_ONEWIRE_ENABLED 0` in `hwdef.dat`, each with its compile-failure
reason recorded there.

`HAL_STORAGE_SIZE 16384` is load-bearing for CAN. The 8 KB default gives
StorageManager only 10 areas with no `StorageCANDNA` slot, and AP_DroneCAN's DNA
server then fails to init.

## Loading firmware

The board ships with a resident bootloader speaking the PX4 serial upload
protocol, so `uploader.py` and mcumgr both write the app slot over USB. Only the
SWD plus ISP path can replace the bootloader itself.

### uploader.py over USB

```bash
python3 Tools/scripts/zephyr_upload_app.py build/mr_vmu_rt1176/zephyr_upload.apj
```

`APJ_BOARD_ID 35` in `hwdef.dat` must match what the bootloader reports or the
image is rejected.

`zephyr_upload_app.py` exists because a bare `uploader.py` run globs
`/dev/serial/by-id/usb-Ardu*` and fires MAVLink reboot bytes at every match,
rebooting other ArduPilot boards on the same bench. It pins the port with a glob
(`usb-ArduPilot_MR-VMU-RT1176*`) rather than a raw tty, so uploader.py's own
catch of the re-enumerated `-BL` device still works. Pinning a raw
`/dev/ttyACM*` breaks that and hangs the upload.

If the upload sits at "Attempting reboot...", the board will not reboot itself
into the bootloader. Open the window externally:

```bash
python3 Tools/scripts/zephyr_pin_reset.py     # pulses nRST through the probe, zero DAP traffic
```

`Tools/scripts/zephyr_flash.sh` wraps upload plus hardware reset in a retry loop
and documents the observed failure modes (bootloader "INVALID OPERATION",
uploader timeout, board left in the bootloader). It predates the port-pinning
tool and tells you to pass no `--port`; use the glob pin in
`zephyr_upload_app.py` instead, which preserves the same `-BL` catch.

`uploader.py`'s erase progress bar is a timer animation:
`__drawProgressBar(label, 10.0, 10.0)` is hard-coded to 100% once about 11 s
elapse, with no board feedback in it. Its only success path is `__trySync()`,
which accepts INSYNC+OK only, so a board replying INSYNC+FAILURE in 20 ms is
reported as a 20-second timeout. To see what the board actually replied, send
`CHIP_ERASE` (0x23) followed by `EOC` (0x20) over the bootloader serial port and
read the two response bytes: 0x12 then 0x10 is OK, 0x11 is FAILURE, 0x13 is
INVALID.

### mcumgr / SMP over USB

`CONFIG_MCUMGR=y` with `zephyr,uart-mcumgr = &cdc_acm_uart1` exposes an SMP
server on the second USB CDC-ACM interface, alongside the MCUBoot-style A/B slot
pair in the partition table. `Tools/scripts/zephyr_smp_upload.py` drives it.

### A/B update path

`Tools/AP_Bootloader/mcuboot_ab.cpp` implements MCUBoot's **overwrite-only**
mode, which is a first-class MCUBoot mode rather than a simplification: there
is no scratch area and no revert. An image found pending in `slot1_partition`
is copied over `slot0_partition`, then slot 1's header sector is erased so the
copy cannot repeat. Swap-with-revert is the documented follow-on and is not
implemented.

The image is what `imgtool` 2.4.0 emits: header magic `0x96f3b83d`, the padded
image, then a TLV trailer. Only the SHA256 TLV is checked, so the copy is
integrity-verified and not authenticated - see `../../BOOTLOADER_SECURITY.md`.

Boot order is A/B first, then the normal AP_Bootloader flow: is slot 1
pending, verify it, erase slot 0, copy, erase slot 1's header, then fall
through to the uploader.py protocol, the timeout and `jump_to_app`. The NOR's
erase-then-program-once constraint is satisfied naturally by that copy path.

Anything that can place an imgtool image at slot 1 can drive this: mcumgr over
the SMP server above, or ArduPilot-side staging.

### SWD

Runner defaults from `../../zephyr/boards/arm/mr_vmu_rt1176/board.cmake`:

- J-Link device `MIMXRT1176xxxxx_M7`, speed 4000, `--reset-after-load`
- pyOCD target `mimxrt1170_cm7`, frequency 4000000

Probe on this bench: MCU-Link-MR (CMSIS-DAP) on the Pixhawk Debug Full 10-pin
connector.

- **Restrict pyOCD to AP0.** Its default discovery walks AHB-AP#1, the dormant
  CM4's debug port, which answers WAIT forever and wedges the DAP. See
  [pyOCD and the dormant CM4](#pyocd-and-the-dormant-cm4).
- **Soft resets trap this SoC in BootROM** at `PC=0x00223104`. Use a hardware
  reset: `pyocd reset -m hw`, or `Tools/scripts/zephyr_pin_reset.py`, which also
  clears a wedged DAP without any SWD traffic.
- **`pyocd flash` and `west flash --runner pyocd` do not work here.** pyOCD's
  builtin flash algorithm fails `result_code=1` on this target. Unresolved.
- **`west flash` is not part of this project's workflow.** West is not used
  inside this repository.

If the board latches in the BootROM (`VTOR = 0x0021xxxx`, no USB, SWD reads of
flash failing with "memory transfer failed" because FlexSPI is unconfigured),
`pyocd ... -c reset` does not clear it. This does:

```sh
pyocd commander -t mimxrt1170_cm7 --connect pre-reset -c "rw 0xE000ED08"
```

`pre-reset` asserts nRESET before connecting and brings the board back to the
bootloader in seconds. It leaves the core HALTED, so follow with `-c go`, or the
1 ms tick never runs, `bootloader()` never times out, `jump_to_app()` is never
called, and it looks exactly like a hang.

## Replacing the bootloader

Writing a bootloader image to NOR offset 0 needs LinkServer over SWD with the
part in ISP mode. The routine `uploader.py` flow writes the app region only, via
whatever bootloader is already resident.

### Why ISP mode is required

The i.MX RT1176 has no internal program flash. It boots from external FlexSPI
NOR via a mask-ROM BootROM baked into silicon. LinkServer's flash driver only
works with the part in ISP / serial-download mode. Once real firmware has booted
normally it reconfigures FlexSPI into an Octal-DDR LUT state LinkServer's SFDP
chip-detect cannot handle, and the chip is detected as `0B` instead of `64MB`. A
plain reset or power-cycle does not clear that state.

Two ways into ISP mode:

```sh
python3 Tools/scripts/rt1176_enter_isp.py
```

calls the BootROM's `runBootloader(void *arg)` entry at `0x0021001C` over SWD
with `arg=0xEB100000` (RM Table 10-68: tag `0xEB`, boot-mode[23:20]=1 serial
downloader, media[19:16]=1 USB). It halts the core, stages the arg in RAM,
points PC at the ROM function with r0 = &arg per AAPCS, and resumes. No firmware
change is needed; the entry lives in immutable boot ROM. After it runs the board
enumerates as an NXP SDP device (`1FC9:013D`) and LinkServer's SFDP driver sees
the clean boot-state FlexSPI it needs. This requires a responsive core to halt.

The physical BOOT0 strap is the fallback for a dead board, and is what makes the
board unbrickable. Hold BOOT0 during power-ON to force the BootROM into ISP mode
instead of executing the NOR. BOOT0 is a pushbutton inside the FMU module:
4 Allen bolts to get the module off the carrier, 4 Phillips screws, ribbon
cable; the button is on the SD-card PCB face at the opposite edge, smaller than
the oscillator can beside it. Press it with a flat-tipped tool. With the module
kept accessible on the bench, a bootloader swap is about 30 s.

What the A/B path checks, and why these boards have no verified boot, is in
`../../BOOTLOADER_SECURITY.md`.

### Prerequisites

- MCU-Link-MR probe attached over the Pixhawk Debug Full 10-pin connector,
  **reflashed with official NXP firmware**. LinkServer refuses the open-source
  firmware, reporting "SWD (DEBUG) disabled", which is a false capability flag.
- LinkServer installed. `rt1176_linkserver_flash.py` looks at `$LINKSERVER`,
  then `/usr/local/LinkServer_*/LinkServer`, then `PATH`.
- The bootloader image:

  ```sh
  ./waf configure --board=mr_vmu_rt1176 --bootloader && ./waf bootloader -j12
  ```

  emits `build/mr_vmu_rt1176/ap_bootloader_<board>.bin`, raw offset-0 layout
  with the FCB at `0x400` tag `FCFB` and the IVT at `0x1000`. Sanity-check both
  with `xxd` before flashing, see
  [Sanity checks on a bootloader image](#sanity-checks-on-a-bootloader-image).
- **An app-restore APJ staged first.** The flasher passes `-e`, a full-chip
  erase, which wipes the app and the param sectors. Build the app and keep its
  `zephyr_upload.apj` aside before touching the bootloader.

### Procedure

1. Put the board in ISP mode, either `rt1176_enter_isp.py` or BOOT0 held through
   a power cycle.
2. Flash:

   ```sh
   python3 Tools/scripts/rt1176_linkserver_flash.py path/to/ap_bootloader_<board>.bin
   ```

   The script exports the LinkServer device config (EVK CM7-only base, flash
   driver swapped to `MIMXRT1170_SFDP_MXIC_OPI.cfx` for this board's Macronix
   Octal-DDR part), then runs `load <bin>:0x30000000 -e -R` and `verify`. With
   no argument it flashes the app build's `zephyr_upload.bin`; for bootloader
   installs always pass the bin explicitly.

   `-e` mass-erases the whole 64 MB NOR first and the erase phase prints no
   progress. Measured 2026-08-14, erase plus a 128 KB write took **167 s**,
   nearly all of it silent at `Pb: (0) Mass erasing Flash`. So:

   - never wrap the flasher in a timeout under about 6 minutes. A 120 s timeout
     kills it mid-erase and looks like a hang (`Aborted!`, flash left partially
     erased, recoverable because the board is still in ISP mode);
   - capture full output to a log file, never through a pipe or `tail`. A killed
     pipe loses everything;
   - the mass erase wipes the app and params, so have the app-restore `.apj`
     staged.

3. Power-cycle without BOOT0.
4. Verify by behaviour, never by SWD readback. Live SWD reads of FlexSPI content
   return zeros or garbage, which is documented and not an error.
   - The bootloader enumerates on USB as an AP_Bootloader CDC device, for
     example
     `/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176-BL_310D080E8292B37F-if00`.
   - Re-upload the app:
     `timeout 30 python3 Tools/scripts/uploader.py <staged app .apj>` and confirm
     the app boots and sends a heartbeat over USB.
   - Params were erased with the chip. `defaults.parm` re-seeds on boot.

Flashing the bootloader erases the app slot. Always re-upload the app
afterwards, or `jump_to_app()` will correctly refuse to boot a blank slot.

`Tools/scripts/zephyr_install_ap_bootloader.py` installs AP_Bootloader over a
resident PX4 bootloader without needing BOOT0. It does not upgrade an
already-resident AP_Bootloader.

### Failure modes seen on hardware

**`Em(12) Target rejected debug access` at the driver mailbox (`0x2000A7C0`),
"Flash Driver V.2 startup failed"** (2026-08-13). SFDP detect succeeds, then
LinkServer's `VECTRESET` to start its on-target flash driver leaves the core
unable to serve RAM access. This is the core-reset trap: pyOCD installs
connect-time trap code via `IOMUXC_LPSR_GPR26`, and core-type resets re-trigger
it, parking the CM7 in BootROM at `PC=0x00223104`. The LPSR domain survives
resets, and with VBAT fitted possibly power-cycles too. Recovery ladder:

1. Redo the ISP entry and retry the flash immediately, with no pyOCD activity in
   between.
2. If it repeats, clear the trap register explicitly before retrying. Verify the
   GPR26 address against the RM first, then zero it with a single pyocd write.
   Or remove VBAT during the power-cycle so the retained domain clears.
3. The failure happens before any erase, so the resident bootloader and app are
   untouched.

**Chip detected as `0B`, or SFDP failure.** The board is not in ISP mode. Redo
step 1.

**`Wire ACK Fault in DAP access`.** Same cause: LinkServer's flash driver only
works in ISP mode.

**`dmesg` needs sudo on some hosts.** Unprivileged it returns empty, which is
indistinguishable from "no USB events".

## Bring-up checklist

1. `./Tools/scripts/zephyr_get_prerequisites.sh`, or export `ZEPHYR_BASE`
2. `./waf configure --board mr_vmu_rt1176`
3. `./waf copter -j12`
4. Confirm `build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.elf` and
   `build/mr_vmu_rt1176/zephyr_upload.apj` exist
5. `python3 Tools/scripts/zephyr_upload_app.py`
6. Verify the board re-enumerates without the `-BL` suffix, so the application
   is running and not the bootloader
7. Verify MAVLink on SERIAL0 over USB CDC
8. Verify IMU, baro and compass detection against the SPI and I2C sections
9. Verify RC input (serial protocol and/or PPM) and PWM outputs 1-12
10. For performance work, read `@SYS/threads.txt` and `@SYS/tasks.txt`, over
    MAVFTP or with `Tools/scripts/zephyr_sysinfo.py` (SWD, non-halting) as the
    fallback. They emit the same `ThreadsV2`/`TasksV2` format as
    `AP_HAL_ChibiOS`, so captures compare column-for-column against a ChibiOS
    board. Build with `--enable-stats` for the CPU LOAD% column, and discard the
    first read after boot.

## Bring-up reference

### AP_Bootloader on Zephyr

`AP_Bootloader` on Zephyr to USB CDC to `uploader.py` to `jump_to_app()` to a
running ArduPilot app, verified end-to-end 2026-07-29:

```text
uploader.py: Erase 100%  Program 100%  Verify 100%  Rebooting.  EXIT 0
VTOR       = 0x30022000     app's vector table (bootloader's is 0x30002000)
PC         = 0x3003eb0e     executing inside the app
SRC_SRSR   = 0x00000000     no lockup, no reset
bl_info.py   sync = False   bootloader gone, app owns the port
```

`hwdef-bl.dat` sets `BOARD_NAME MR-VMU-RT1176-BL` and `APJ_BOARD_ID 35`. Its
header comment still describes the file as untested groundwork; that comment is
from 2026-07-26 and predates the working port.

Seven defects had to be fixed to get there, each one a place where
`support_Zephyr.cpp` or the `__ZEPHYR__` branch had a stub or a
`TODO(zephyr-bootloader, UNTESTED)` and `support.cpp` had working code a few
lines away.

#### 1. `usbphy1` disabled in the board DTS

`usb1` was `okay` and referenced it via `phy-handle`, but the PHY node itself was
`status = "disabled"`. The EHCI controller initialises fine without it and the
D+ pull-up still asserts, so every device-side indicator reads green:
`usbd_init()`=0, `usbd_enable()`=0, `udc_is_initialized()`=1,
`udc_is_enabled()`=1, CFSR=0, HFSR=0. Meanwhile the host sees
`new full-speed USB device` then `device descriptor read/64, error -32`. The
failure is below the layer all of those report on. Only `dmesg` shows it.

#### 2. FS-only descriptor registration

Only `USBD_SPEED_FS` config, classes and code-triple were registered, on a
controller with `CONFIG_UDC_DRIVER_HIGH_SPEED_SUPPORT_ENABLED=y`. Fixed by
adding a `cube_hs_config` plus HS registration gated on
`usbd_caps_speed() == USBD_SPEED_HS`, mirroring
`samples/subsys/usb/common/sample_usbd_init.c`.

#### 3. Flash addressing off by the bootloader region

`flash_func_*` based protocol offsets at `FLASH_LOAD_ADDRESS` (`0x30000000`)
instead of the app area. ChibiOS uses
`flash_base = 0x08000000 + FLASH_BOOTLOADER_LOAD_KB*1024`. Any upload would have
erased the bootloader out from under itself.

#### 4. `flash_func_sector_size()` never returned 0

It is the erase loop's only terminator:

```c
for (uint16_t i = 0; flash_func_sector_size(i) != 0; i++)
```

Returning a constant made the loop run until `uint16_t` wrapped at 65536.

#### 5. ROM API: wrong instance, never initialised

Three deviations from
`modules/zephyr/boards/nxp/vmu_rt1170/flexspi_nor_config.c`:

|                  | Reference                                  | Ours, before                           |
| ---------------- | ------------------------------------------ | -------------------------------------- |
| FlexSPI instance | `1`                                        | `0`                                    |
| NOR driver init  | `ROM_FLEXSPI_NorFlash_Init(instance, cfg)` | never called                           |
| Config argument  | writable RAM copy                          | `const_cast` of the flash-resident FCB |

Every ROM flash call returned `kStatus_InvalidArgument` (status 4). PX4's imxrt
bootloader independently confirms instance 1 and the range-erase form.

#### 6. The FCB `lookup_table` defined only sequence 0 (Read)

That is all the ROM needs to boot XIP, and the FCB was authored purely for boot.
`ROM_FLEXSPI_NorFlash_Erase` and `ProgramPage` drive the same LUT and need the
write-side sequences, at indices fixed by `NOR_CMD_LUT_SEQ_IDX_*`:

| Idx | Sequence     | Opcode |
| --- | ------------ | ------ |
| 1   | Read Status  | `0x05` |
| 3   | Write Enable | `0x06` |
| 5   | Erase Sector | `0x20` |
| 8   | Erase Block  | `0xD8` |
| 9   | Page Program | `0x02` |
| 11  | Chip Erase   | `0x60` |

With no Read Status sequence the ROM issued its erase, polled an undefined LUT
entry for the WIP bit, never saw ready, and spun inside the BootROM forever.
Sequences copied from the in-tree Zephyr boards `nxp/mimxrt1170_evk` and
`phytec/phyboard_atlas`, which define them byte-identically. Their extra
sequence 12, Set Read Register `0xC0`, programs dummy cycles for a quad `0xEB`
read; we read single-pad `0x03` with no dummy cycles and don't need it.

Related: `EraseBlock` (64 KB) returns `kStatus_InvalidArgument` on this part
because its FCB sets `is_uniform_block_size = false`. The 4 KB `EraseSector`
and the range `Erase` forms both work.

#### 7. `flash_write_buffer()` was a stub, and buffering must be page-aligned

It returned `false` unconditionally, so `PROG_MULTI` failed on its first block.
Implementing it exposed a second issue: `support.cpp` anchors its buffer at
whatever address the caller first supplied, which is fine on STM32's 32-byte
write lines and fatal here. A buffer straddling a page boundary programs two
pages partially, then reprograms the second on the next flush, and a NOR page
may only be programmed once per erase. Symptom: 987 page programs for a 661-page
image, a CRC mismatch, and the image's first 32 bytes reading back `0x00`
instead of `0xFF`. The buffer is now anchored to a page-aligned base and indexed
by offset within the page.

### Cache and MPU teardown at the jump

`do_jump()` disables D-cache and I-cache, but the block is gated on
`#if defined(STM32F7) || defined(STM32H7)`. RT1176 is also a Cortex-M7 with both
caches, so on Zephyr they stayed on across the handoff. The app started against
the bootloader's cached view of flash it had just reprogrammed, plus inherited
MPU regions it had not configured, and hit `M7_LOCKUP` (`SRC_SRSR` bit 2) every
time, with `g_jump_stage == 4` proving the jump itself was fine.

The `__ZEPHYR__` branch of `do_jump()` now does `SCB_DisableDCache()`,
`SCB_DisableICache()` and `ARM_MPU_Disable()` with DSB/ISB. Before repointing
VTOR it also does `k_timer_stop()` on the 1 ms tick, SysTick off, PendST and
PendSV cleared, and every NVIC line masked and cleared.

**Do not `__disable_irq()` there.** On ARMv7-M, Zephyr's `reset.S` locks
interrupts with BASEPRI (the `cpsid i` in it is the ARMv6-M path) and
`irq_unlock()` also writes BASEPRI. Nothing ever clears PRIMASK, so setting it
boots the app with interrupts masked forever, trading a lockup for a silent
hang.

`#if defined(STM32F7) || defined(STM32H7)` around `SCB_DisableDCache()` looks
ChibiOS-specific but is a Cortex-M7 architectural requirement.

### PX4 v6xrt compatibility

Bidirectional by design: an app built for the PX4 v6xrt bootloader uploads and
boots here, and vice versa.

|                    | PX4 v6xrt     | Ours          |
| ------------------ | ------------- | ------------- |
| App slot           | `0x30020000`  | `0x30020000`  |
| Upload writes from | slot offset 0 | slot offset 0 |
| Vector table       | `0x30022000`  | `0x30022000`  |
| `BOARD_ID`         | 35            | 35            |

`APP_VECTOR_OFFSET` was added to `bl_protocol.cpp`, defaulting to 0 so every
existing ChibiOS board stays bit-identical; `hwdef_zephyr.h` sets it to `0x2000`.
The first `0x2000` of an RT117x app image is its FCB/IVT boot header, which is
why the offset exists. Keep it in sync with `CONFIG_FLASH_LOAD_OFFSET=0x22000`
in `prj.mr_vmu_rt1176.conf` and the `0x2000` pad in
`Tools/ardupilotwaf/zephyr.py`.

`hwdef_zephyr.h` also sets `BOARD_FLASH_SIZE (4 * 1024)` KB, not the full 64 MB.
CHIP_ERASE walks every sector in the declared APP region and `uploader.py` allows
20 s for the whole erase; 64 MB is 16352 sectors, well past that budget. 4 MB
leaves about 3.9 MB for the app at 992 sectors, of which
`flash_func_erase_sector` skips any already blank. An image over about 3.9 MB
needs this raised, which re-opens the erase-budget question.

### Bootloader diagnostics over SWD

Symbol addresses move every build. Re-read them from the current ELF each time:

```sh
E=build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.elf
arm-none-eabi-nm $E | grep -E "g_jump_|g_erase_|g_prog_"
pyocd commander -t mimxrt1170_cm7 --connect attach -c "rw <addr>"
```

| Symbol                                                 | Meaning                                                                         |
| ------------------------------------------------------ | ------------------------------------------------------------------------------- |
| `g_jump_magic`                                         | `0x4A554D50` (`'JUMP'`) once `jump_to_app()` has run                            |
| `g_jump_stage`                                         | 1 entered, 2 lead-words ok, 3 entrypoint ok, 4 about to jump; 10/11/12 rejected |
| `g_jump_detail`                                        | offending value for the 10/11/12 cases                                          |
| `g_erase_fail_unit` / `_status` / `g_erase_units_done` | per-block erase outcome                                                         |
| `g_prog_fail_offset` / `_status` / `g_prog_pages_done` | per-page program outcome                                                        |

The `g_jump_*` markers are `__noinit`, not `.bss`, so they survive the warm reset
that a lockup causes. `g_jump_magic` distinguishes "we wrote this" from
uninitialised RAM. Clear the markers and `SRC_SRSR` before each test, or you read
a stale value from an earlier boot. The markers and the `JUMP_MARK`/`JUMP_FAIL`
macros in `bl_protocol.cpp` are debug scaffolding and can be stripped; they are
`#ifdef __ZEPHYR__`-guarded so ChibiOS is unaffected.

Also high-value:

- **`SRC_SRSR` at `0x40C04010`**, reset cause. Bit 0 POR, 1 software,
  **2 M7_LOCKUP**, 5 WDOG, 14 CDOG. Cumulative and write-1-clear: clear it with
  `ww 0x40C04010 0xFFFFFFFF` before a test so the reading is attributable.
- **PC sampling** (`-c halt -c "reg pc" -c go`): `0x2020xxxx` ramfunc erase,
  `0x0021xxxx` BootROM, `0x3000xxxx` bootloader, `0x3002xxxx`-`0x3004xxxx` app.
- **VTOR at `0xE000ED08`**: `0x30002000` bootloader, `0x30022000` app,
  `0x0021xxxx` ROM.

### Sanity checks on a bootloader image

A bad image costs an ISP recovery cycle.

- `FCFB` magic at file offset `0x400`
- LOAD segment at `0x30000000` (`readelf -l`)
- `jump_to_app` and `board_info` present in the ELF
- LUT sequences present: `xxd -s 0x490 -l 4` gives `0504 0424` (Read Status)

### Bootloader configuration and stubs

`port_setbaud`, `flash_func_read_otp` and the RTC-signature pair are stubs by
choice: fixed-baud USB CDC, no OCOTP fusemap work done, and
`AP_FASTBOOT_ENABLED` is off.

`led_on`/`led_off`/`led_toggle` use
`GPIO_DT_SPEC_GET_OR(DT_ALIAS(led_amber), gpios, {0})` and degrade to a no-op if
the alias is absent or the pin does not configure. This board's LED pins are
noted in its DTS as guessed rather than schematic-derived, so the LEDs are
implemented but unverified.

`prj.mr_vmu_rt1176.conf` is the uploadable config (`FLASH_LOAD_OFFSET=0x22000`,
`BOOT_HEADER=n`), so an app built with it is not directly ISP-flashable. Set
those back to `0x0` and `y` for a standalone image. Otherwise recovery is "flash
a bootloader via ISP, then upload".

`Tools/scripts/rt1176_direct_flash.py` writes straight to `0x30000000`,
bypassing the bootloader entirely, and needs that direct-boot config
(`CONFIG_NXP_IMXRT_BOOT_HEADER=y`, `CONFIG_FLASH_LOAD_OFFSET=0x0`). It is the
diagnostic path, not the production one.

### RC input pad arbitration

RC-IN on this carrier reaches one MCU pad: `EMC_B1_40`, LPUART6's TX pad, used
in single-wire/half-duplex mode (`CTRL[LOOPS]=1`, `CTRL[RSRC]=1`) so the same pad
both drives and senses the line for SBUS/CRSF. Classic Pixhawk boards wire RC-IN
to two separate MCU pins, a dedicated timer-capture pin plus a UART pin, both
tied to the same net on the PCB. This carrier does not.

#### GPIO cannot observe a pad it does not own via MUX_MODE

There is no register, no bit and no trick that changes this. It is how the
IOMUXC input path is wired for a peripheral with no DAISY/SELECT_INPUT entry.

Register evidence, read over SWD on a live board with an oscilloscope-confirmed
8-channel idle-high CPPM signal present on the wire (2026-08-13). GPIO2 base
`0x40130000`, from the generated `zephyr.dts` (`gpio2: gpio@40130000`):

| Register | Address      | Value                                                                                                                             | Meaning                                         |
| -------- | ------------ | --------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------- |
| GDIR     | `0x40130004` | bit8=0                                                                                                                            | pin 8 is an input, correct                      |
| ICR1     | `0x4013000c` | bit8 region=0                                                                                                                     | expected 0 for EDGE_BOTH; EDGE_SEL governs      |
| IMR      | `0x40130014` | bit8=1                                                                                                                            | interrupt genuinely armed for pin 8             |
| EDGE_SEL | `0x4013001c` | bit8=1                                                                                                                            | both-edge trigger genuinely armed for pin 8     |
| PSR      | `0x40130008` | constant 0, 0 transitions in 14,439 samples over 5 s; repeated after reseating the connector, 16,800 samples, still 0 transitions | pad reads permanently LOW despite a live signal |

Cross-check that the read method works: bit 11 of that same PSR register (gpio2
pin 11, the LPSPI1 chip-select, toggled by the running IMU driver) reads a
different, real value from bit 8 in the same read. The SWD reads are live
per-bit register content, not a frozen snapshot.

IOMUXC pad control, `SW_MUX_CTL_PAD_GPIO_EMC_B1_40` at
`0x400E8000 + 0x10 + 40*4 = 0x400E80B0`:

```text
value = 0x00000013
MUX_MODE (bits 3:0) = 3   LPUART6, matches the DTS pinmux, not GPIO's ALT
SION     (bit 4)    = 1   set by LPUART6's own `input-enable;` pinctrl property,
                          a single per-pad bit, not per-peripheral
```

i.MX RT1170 Processor Reference Manual Rev. 3 (11/2024) §12.4.6.42
"SW_MUX_CTL_PAD_GPIO_EMC_B1_40 SW MUX Control Register" (p.843), field for
field:

```text
Bits 3-0  MUX_MODE  Select 1 of 8 iomux modes for pad GPIO_EMC_B1_40:
   0000b ALT0  SEMC_RDY (SEMC)
   0001b ALT1  XBAR1_INOUT12 (XBAR1)
   0010b ALT2  MQS_RIGHT (MQS)
   0011b ALT3  LPUART6_TXD (LPUART6)          <- SBUS/CRSF
   0101b ALT5  GPIO_MUX2_IO08 (GPIO_MUX2)     <- software PPM edge capture
   0111b ALT7  ENET_1G_MDC (ENET_1G)
   1001b ALT9  CCM_CLKO1 (CCM)
   1010b ALT10 GPIO8_IO08 (GPIO8)             <- the "fast GPIO" alias, a
                                                 DIFFERENT alt value, not a way
                                                 around MUX_MODE

Bit 4  SION  Software Input On Field.
   "Force the selected mux mode Input path no matter of MUX_MODE
    functionality." 0b = input path determined by functionality,
    1b = force input path of pad GPIO_EMC_B1_40.

Reset value: 0000_0005h   this pad's power-on default is ALT5 (GPIO); our
                          board's pinctrl reprograms it to ALT3 (LPUART6)
                          during boot, which is why MUX_MODE read 3 above.
```

§12.3.2 "SW Loopback through SION bit" (p.454) settles what SION does:

> "A limited option exists to override the default pad functionality and force
> the input path to be active regardless of the value driven by the
> corresponding module. This can be done by setting the SION (Software Input On)
> bit... Uses include: LoopBack - **Module x drives the pad and also receives pad
> value as an input.**"

SION is loopback for whichever module `MUX_MODE` already selects. It lets that
one module read back what it is itself driving or sensing. It is not a general
"any peripheral can see any pad" switch. With `MUX_MODE=3` (LPUART6), SION helps
LPUART6's own loopback, which is why single-wire mode needs `input-enable;` on
that pinctrl group at all. It does nothing for GPIO, which is not the selected
module.

§12.3.3 "Daisy chain - multi pads driving same module input pin" (p.454-455)
covers the other mechanism that could in principle let a peripheral see a pad it
does not own, and rules it out on two counts. First, GPIO has no `SELECT_INPUT`
register. Second, even for peripherals that do have one, daisy chaining is not a
`MUX_MODE` bypass but an additional requirement layered on top of it:

> "A module port involved in 'daisy chain' requires **two** software
> configuration commands, **one for selecting the mode for this pad**
> (programmable via the `IOMUXC_SW_MUX_CTL_<PAD>` registers) **and one** for
> defining it as the input path (via the daisy chain registers)."

Cross-checked against the vendored SDK header
`modules/hal/nxp/mcux/mcux-sdk-ng/devices/RT/RT1170/periph/PERI_IOMUXC.h`: the
`SELECT_INPUT[]` array at offset `0x498`, base `0x400E8498`, lists
`FLEXCAN1_RX_SELECT_INPUT` through `XBAR1_IN_SELECT_INPUT_35`, all
flexible-routing peripherals that can source from one of several pads. Nothing
named `GPIO2_IO08` or similar appears in it. GPIO's pad association is fixed at
the `MUX_MODE` level with no daisy option, so there is no second command
available for it.

#### The pad in both roles

One physical pin, two Zephyr pinctrl node names, two `MUX_MODE` values, the same
`SW_MUX_CTL_PAD`/`SW_PAD_CTL_PAD` register pair:

| Role                                      | Devicetree pinmux node                     | `MUX_MODE` (ALT) |
| ----------------------------------------- | ------------------------------------------ | ---------------- |
| LPUART6 TX, single-wire SBUS/CRSF         | `iomuxc_gpio_emc_b1_40_lpuart6_tx`         | **3**            |
| GPIO2 IO08, software PPM edge capture     | `iomuxc_gpio_emc_b1_40_gpio_mux2_io08_cm7` | **5**            |
| XBAR1_IN12 to QTMR1, hardware PPM capture | XBAR1 alt on the same pad                  | **1**            |

Both pinctrl nodes resolve to the same Zephyr-generated pinmux tuple format,
`{MUX_CTL_reg, MUX_MODE, INPUT_REG, INPUT_DAISY, PAD_CTL_reg}`:

```text
LPUART6_TX: {0x400e80b0, 3, 0, 0, 0x400e82f4}
GPIO IO08:  {0x400e80b0, 5, 0, 0, 0x400e82f4}
```

from
`build/mr_vmu_rt1176/zephyr_build/zephyr/include/generated/zephyr/devicetree_generated.h`,
searching for `emc_b1_40_lpuart6_tx_P_pinmux` and
`emc_b1_40_gpio_mux2_io08_cm7_P_pinmux`. That file is regenerated on every
configure, so re-derive from a fresh build rather than trusting these literals if
the toolchain or SDK version changes.

`INPUT_REG` and `INPUT_DAISY` are both 0 for this pad in both roles, confirming
directly, not just by inference from the missing `SELECT_INPUT` table entry, that
neither function uses a DAISY input select here. `MUX_MODE` is the only thing
determining which peripheral the pad's signal reaches.

#### IOMUXC pad registers

Base `IOMUXC_BASE = 0x400E8000`, from
`modules/hal/nxp/mcux/mcux-sdk-ng/devices/RT/RT1170/MIMXRT1176/MIMXRT1176_cm7_COMMON.h`.

- `SW_MUX_CTL_PAD[]`: struct offset `0x10`, step `0x4`, index 40
  (`kIOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_B1_40`), so **`0x400E80B0`**
  - bits `[3:0]` `MUX_MODE`: 1 = XBAR1_IN12, 3 = LPUART6_TXD, 5 = GPIO2_IO08
  - bit `4` `SION`
- `SW_PAD_CTL_PAD[]`: struct offset `0x254`, step `0x4`, index 40
  (`kIOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_B1_40`), so **`0x400E82F4`**
  - pull-up/down, drive strength, slew rate. `UARTDriver.cpp::_begin()`'s
    RXINV-tracking logic already flips this register at runtime for polarity.
    Anything that takes over pull direction must not fight that.

#### GPIO2 register map

All offsets from `GPIO2_BASE = 0x40130000` (`gpio2: gpio@40130000` in the
generated `zephyr.dts`), standard i.MX RT `GPIO_Type` layout, confirmed field by
field against `gpio_mcux_igpio.c`'s register accesses.

| Register | Offset | Address      | Purpose                                               |
| -------- | ------ | ------------ | ----------------------------------------------------- |
| DR       | `0x00` | `0x40130000` | data register (output value)                          |
| GDIR     | `0x04` | `0x40130004` | direction, 0 = input                                  |
| PSR      | `0x08` | `0x40130008` | pad status, live input read, bit 8 = this pin         |
| ICR1     | `0x0C` | `0x4013000C` | interrupt config, pins 0-15, 2 bits per pin           |
| ICR2     | `0x10` | `0x40130010` | interrupt config, pins 16-31                          |
| IMR      | `0x14` | `0x40130014` | interrupt mask (enable), bit 8 = this pin             |
| ISR      | `0x18` | `0x40130018` | interrupt status, write-1-clear                       |
| EDGE_SEL | `0x1C` | `0x4013001C` | 1 = both-edge trigger overrides ICR, bit 8 = this pin |

Pin 8 throughout, because the DTS alt is `gpio_mux2_io08`: GPIO2, IO08.

#### The runtime arbiter

`RCInput.cpp` writes `SW_MUX_CTL_PAD_GPIO_EMC_B1_40` directly, which is the only
way to move this pad between its owners:

```c
static constexpr uint32_t RCIN_MUX_CTL_ADDR = 0x400E80B0U;
static constexpr uint32_t RCIN_MUX_ALT_UART = 3U;
#if defined(CONFIG_AP_RCIN_PWM_CAPTURE)
static constexpr uint32_t RCIN_MUX_ALT_PPM = 1U;   /* ALT1 XBAR1_IN12 -> QTMR */
#else
static constexpr uint32_t RCIN_MUX_ALT_PPM = 5U;   /* ALT5 GPIO_MUX2_IO08 */
#endif
```

These addresses are hardcoded to this pad, not derived from `rcin-gpios` or the
DTS. Another shared-pad board needs its own constants, not a reuse of these.

`_set_pad_mux()` is driven from `_update()`. The default is LPUART6, so SBUS and
CRSF behaviour is unchanged; it periodically steals the pad for one
PPM-frame-length window to check for PPM. Whichever side gets a valid frame first
latches permanently, and `_pad_latched` then stops all further switching. That
matches `pulse_input_enable()`'s one-way-latch semantics for the UART side,
extended to work in the GPIO-wins direction too, so a locked link costs nothing
extra. State lives in `RCInput.h`: `_pad_latched`, `_pad_is_gpio`,
`_gpio_probe_until_us`, `_next_gpio_probe_us`.

With `RCIN_PULSE_GPIO_SHARES_UART_PAD` defined, `RCInput.cpp` must not call
`gpio_pin_configure_dt()`, because that would additionally reprogram the pull.
On a board with a dedicated capture pin the macro stays undefined and the plain
path runs `gpio_pin_configure_dt(&rcin_gpio, GPIO_INPUT | GPIO_PULL_UP)`. PPM
idles high, and a floating or disconnected lead has to sit quietly rather than
harvest noise edges.

`UARTDriver.cpp::_begin()` calls `uart_rx_disable()` before `uart_rx_enable()`.
Without it, any UART whose `_begin()` is retried without a clean teardown, which
is what happens when no UART protocol ever locks, fails `uart_rx_enable()` with
`-EBUSY` forever, spamming the console and costing real CPU. A CPPM-only
receiver guarantees that condition.

#### The DShot pinctrl precedent

Runtime pad sharing between FlexPWM and FlexIO already exists in this tree, via
pinctrl states rather than a raw register write. In the board DTS:

```dts
ap_rcout_mux: ap-rcout-mux {
    compatible = "ardupilot,rcout-pinmux";
    pinctrl-0 = <&pinmux_flexpwm1_pwm0_default ...>;   /* "default" state */
    pinctrl-1 = <&pinmux_dshot_fmu_ch1_8>;              /* "dshot" state */
    pinctrl-names = "default", "dshot";
};
```

backed by `../../zephyr/src/ap_rcout_pinmux.c`, a plain-C file, not C++, because
`PINCTRL_DT_DEFINE`'s macro expansion fails `-Wnarrowing` as C++. It calls
`pinctrl_apply_state()` to switch between the declared states at runtime and is
exposed to `RCOutput.cpp` through a small `extern "C"` facade,
`include/ap_rcout_pinmux.h`.

The same shape works for RC input: an `ap_rcin_mux` node with `pinctrl-0` =
`pinmux_lpuart6_default`'s existing group and `pinctrl-1` = a new
`pinmux_rcin_gpio_capture` group referencing
`iomuxc_gpio_emc_b1_40_gpio_mux2_io08_cm7`, plus an `ap_rcin_pinmux.c` sibling.

### pyOCD and the dormant CM4

Raw `pyocd cmd -c halt ...` intermittently fails with `Error reading AP#0 IDR` or
`Transfer error while reading AHB-AP#1 ROM table`. pyOCD's default discovery
walks AHB-AP#1, the dormant CM4's debug port, which answers WAIT forever and
wedges the DAP. `pyocd cmd` has no easy flag for `valid_aps`, so connect from a
small inline Python script instead:

```python
from pyocd.core.helpers import ConnectHelper
session = ConnectHelper.session_with_chosen_probe(
    target_override='cortex_m',
    connect_mode='attach',
    options={'frequency': 4000000, 'valid_aps': [0]})
```

`Tools/scripts/zephyr_pcsr_sample.py` and `Tools/scripts/zephyr_sysinfo.py`
already use this pattern. `Tools/scripts/zephyr_pin_reset.py` clears a wedged DAP
without any SWD traffic at all.
