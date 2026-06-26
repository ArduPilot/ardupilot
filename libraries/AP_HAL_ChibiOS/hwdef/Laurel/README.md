# Laurel (RP2350B) ArduPilot Port

The Laurel target runs ArduPilot on a custom RP2350B flight controller
board built around the Raspberry Pi RP2350B (QFN-80, 48 GPIO) running at
375 MHz. Unlike the Pico2 carrier-board reference target, Laurel has its
own fixed sensor stack, power rails, PWM outputs, edge buttons, and a
different bus layout.

This README documents the Laurel-specific wiring and the current state of
the `hwdef/Laurel/` target.

## Pin Numbering Conventions

Three different numbering schemes may appear in Laurel documentation:

| Notation | Example | Meaning |
|----------|---------|---------|
| **GPIO N** / **GP N** | `GPIO12`, `GPIO45` | Logical RP2350 GPIO number. In `hwdef.dat` this appears as `PA<N>`. This is the primary naming used by ArduPilot. |
| **RP2350B QFN-80 pin N** | `pin 35`, `pin 75` | Physical package pin on the RP2350B chip. Used when tracing the PCB or matching schematic netlists. |
| **Board net name** | `BOOTSW`, `BEC_5V_EN` | Schematic / PCB signal name on the Laurel board. |

Laurel is a custom RP2350B board, not a Pico2W module, so the Pico2W
header-pin numbering from the Pico2 README does not apply here.

## Features

- RP2350B dual-core Cortex-M33 @ 375 MHz
- 520 KB SRAM
- 8 MB boot/XIP flash: `W25Q64JVXGIM` (Winbond, 133 MHz max, CS = `QSPI_SS` pin75 — dedicated QMI hardware pin)
- 16 MB blackbox flash: `W25Q128JVPIM` (Winbond, 133 MHz max, CS = `PA0`/GPIO0/pin77) — SPI bus not yet confirmed in hwdef
- USB CDC serial on `SERIAL0`
- 2 hardware UARTs + 2 PIO UARTs in the current Laurel hwdef
- 4 PWM motor outputs on GPIO28-31
- SPI0 IMU bus with onboard ICM42688P
- shared SPI1 OSD / microSD hardware option
- I2C0 barometer bus with onboard DPS310
- 3 ADC inputs on RP2350B-only GPIO40-42
- green LED on GPIO7 and blue LED on GPIO6
- onboard RGB LED on GPIO39
- buzzer on GPIO5
- 5 V and 9 V regulator enable GPIOs
- Hardware watchdog enabled by default

## Buttons And Boot Modes

Laurel has two physical edge pushbuttons:

- `RESET / SW2`: wired to the RP2350 `RUN` pin (package pin 35) with a pull-up. This is the hardware reset button.
- `BOOT / SW1`: wired to the `BOOTSW` net, which is tied back through a resistor to the RP2350 `QSPI_SS` strap pin (package pin 75). This is the BOOTSEL-style boot button.

In practice this means:

- pressing `RESET / SW2` resets the MCU
- holding `BOOT / SW1` while asserting reset should force the RP2350 ROM USB bootloader path
- normal ArduPilot firmware upload after the AP bootloader is installed uses the AP bootloader over USB, not the ROM BOOTSEL path

The Laurel hwdef does not declare these buttons as GPIO inputs because they
act as hardware reset / strap signals, not runtime firmware buttons.

## Serial Port Mapping

| Serial port | Function | GPIO |
|-------------|----------|------|
| `SERIAL0` | USB CDC console / MAVLink | USB OTG1 |
| `SERIAL1` | DVTX / MSP DisplayPort | TX=`GPIO12`, RX=`GPIO13` |
| `SERIAL2` | GPS | TX=`GPIO8`, RX=`GPIO9` |
| `SERIAL3` | CRSF / ELRS | TX=`GPIO20`, RX=`GPIO21` |
| `SERIAL4` | AUX / spare | TX=`GPIO34`, RX=`GPIO35` |

Current hwdef defaults:

- `SERIAL0`: MAVLink2 over USB
- `SERIAL1`: MSP DisplayPort
- `SERIAL2`: GPS
- `SERIAL3`: RCIN protocol by default (`SERIAL3_PROTOCOL=23`); receives CRSF/ELRS from receiver on `GPIO21`
- `SERIAL4`: MAVLink2

Two additional RX-only pads exist on the Laurel PCB and are not yet declared
in the current hwdef:

- `GPIO36` — `DVTX_SBUS_RX` — SBUS receiver input, on connector `J5`.
- `GPIO37` — `TELEM_RX` — ESC sensor telemetry receive, on connector `J1` pin 4.

They are intentionally omitted from `hwdef.dat` because the RP2350 serial
path requires a full-duplex TX+RX pair to declare a serial port.

## PWM Outputs

Laurel currently exposes four PWM outputs:

| Output | GPIO | RP2350 PWM slice |
|--------|------|------------------|
| PWM1 | GPIO28 | PWM6_A |
| PWM2 | GPIO29 | PWM6_B |
| PWM3 | GPIO30 | PWM7_A |
| PWM4 | GPIO31 | PWM7_B |

These are configured as ArduPilot GPIOs `50` through `53` when used as
servo outputs. Standard 50 Hz PWM is supported. DShot is not supported on
RP2350 in this target.

## LEDs, Beeper, And Power Rails

| Function | GPIO | Notes |
|----------|------|-------|
| Blue status LED | GPIO6 | Exported in hwdef as `LED_BLUE`; hwdef drives output low at boot |
| Green status LED | GPIO7 | Exported in hwdef as `LED_GREEN`; hwdef drives output low at boot |
| Buzzer | GPIO5 | PWM `ALARM` output driving the onboard electromagnetic transducer at a fixed nominal 4 kHz |
| 5 V regulator enable | GPIO14 | `BEC_5V_EN`, driven high by default in application firmware |
| 9 V regulator enable | GPIO15 | `BEC_9V_EN`, held low by default for bring-up |

Laurel also has an onboard single-wire RGB LED on `GPIO39`, but serial LED
support remains disabled for RP2350 in this target, so that LED is not yet
bound in `hwdef.dat`.

### Laurel Pre-Arm Beep Counts

Laurel enables a board-specific tone-count mapping for common pre-arm failures.
When the buzzer reports a pre-arm fault, the first failing check in the current
pre-arm cycle is mapped as follows:

- `3` beeps: RC input / transmitter failure
- `4` beeps: battery or board-voltage failure
- `5` beeps: sensor / estimator input failure (`BARO`, `COMPASS`, `GPS`, `INS`, `AIRSPEED`, `RANGEFINDER`, `VISION`, `FFT`)
- `6` beeps: configuration / system / storage-style failure (`PARAMETERS`, `LOGGING`, `SWITCH`, `GPS_CONFIG`, `SYSTEM`, `MISSION`, `CAMERA`, `AUX_AUTH`, `OSD`)

If no Laurel-specific mapping matches, ArduPilot falls back to the normal
generic notify tones.

## SPI Buses

### SPI0 - IMU Bus

| Signal | GPIO |
|--------|------|
| SCK | GPIO2 |
| MISO | GPIO4 |
| MOSI | GPIO3 |
| IMU CS | GPIO1 |
| IMU DRDY | GPIO22 |

Current onboard sensor:

- `ICM42688P` on SPI0, chip-select `GPIO1`
- Laurel schematic cross-check: `CIPO=GPIO4`, `CSn=GPIO1`, `SCLK=GPIO2`, `COPI=GPIO3`, `IRQn=GPIO22`

Additional noted net:

- `IMU_CLKIN` is routed to `GPIO23`, but is not currently configured by this port

ArduPilot probe line in the current target:

```text
IMU Invensensev3 SPI:icm42688 ROTATION_NONE
```

### SPI1 - Shared OSD / microSD Bus

| Signal | GPIO |
|--------|------|
| SCK | GPIO26 |
| MISO | GPIO24 |
| MOSI | GPIO27 |
| OSD CS | GPIO17 |
| microSD CS | GPIO25 |

Current onboard peripheral:

- SPI-mode microSD card on the same SPI1 clock/data pins with dedicated chip-select `GPIO25`

Laurel cannot use the hardware OSD and microSD functions at the same time on this hardware.
The current Laurel hwdef chooses SPI-mode microSD support and leaves the hardware OSD SPI device disabled.
The commented OSD lines are kept in the hwdef so a user can switch the SPI1 role later.

## I2C Bus

Laurel uses `I2C0`, not `I2C1`:

| Signal | GPIO |
|--------|------|
| I2C0 SCL | GPIO45 |
| I2C0 SDA | GPIO44 |

Current onboard sensor:

- `DPS310` barometer at `0x76`
- Laurel schematic cross-check: `SCL=GPIO45`, `SDA=GPIO44`, with pull-ups fitted on the bus
- DPS310 `SDO` is pulled low, confirming the `0x76` address used by the hwdef

Current ArduPilot probe line:

```text
BARO DPS310 I2C:0:0x76
```

`AP_COMPASS_PROBING_ENABLED` is enabled, so an external I2C compass can be
added on this bus if the board design exposes it.

## Analog Inputs

Laurel uses RP2350B-only ADC-capable pins above GPIO29:

| Function | GPIO | ADC channel |
|----------|------|-------------|
| Battery voltage | GPIO40 | ADC0 |
| Battery current | GPIO41 | ADC1 |
| RSSI analog | GPIO42 | ADC2 |

The current hwdef leaves voltage/current scale factors at `1.0`, so these
values are placeholders until the Laurel analog frontend is calibrated.

## Sensor Stack

The current Laurel target probes the following onboard sensors:

| Sensor | Bus | Connection |
|--------|-----|------------|
| ICM42688P IMU | SPI0 | `GPIO1` CS, `GPIO22` DRDY |
| DPS310 barometer | I2C0 | `0x76` |
| AT7456E / MAX7456 OSD | SPI1 | `GPIO17` CS |

The Laurel README is intentionally more specific than the Pico2 README here:
the sensor buses are fixed by the board layout rather than being a generic
carrier-board example.

## Storage

Main parameter storage uses the RP2350 XIP flash. Logs use the SPI-mode microSD card on the shared SPI1 bus (confirmed working as of 2026-06-26; `LOG_BACKEND_TYPE=1` is set `@READONLY` in defaults.parm):

- main flash size: 8 MB
- bootloader region: first 32 KB
- parameter storage region: next 32 KB
- application region: remainder of flash
- logical parameter capacity: 8 KB with `AP_FLASH_STORAGE_QUAD_PAGE 1`

The Laurel board carries a secondary `W25Q128JVPIM` (Winbond 128 Mbit / 16 MB)
blackbox flash in addition to the boot flash. Its chip-select is `PA0` (GPIO0,
pin77), confirmed from the Betaflight board config. The SPI bus it shares has
not yet been confirmed from the schematic. Storage currently consists of the
main XIP flash (`W25Q64JVXGIM`) plus the SPI-mode microSD card.

## Connectors

Laurel has two switched power rails that feed the connector VOUT pins:

| Rail | Enable GPIO | Default state | Consumers |
|------|-------------|---------------|-----------|
| 5 V BEC | GPIO14 (`BEC_5V_EN`) | **HIGH** (always on) | J2, J6, J7, J8, J10, J11 |
| 9 V BEC | GPIO15 (`BEC_9V_EN`) | **LOW** (off) | J5 pin 1 |

J1 pin 1 is raw `VBAT` (unswitched battery voltage), not a regulated output.
J5 pin 1 is 9 V but **unpowered by default** — `BEC_9V_EN` must be driven high
to enable video transmitter power.

### J1 — ESC Connector (8-pin)

`J1` is the main ESC connector. It carries battery power, the current-sense
signal, ESC telemetry, and all four PWM motor outputs:

| J1 Pin | Signal | GPIO | Notes |
|--------|--------|------|---------|
| 1 | VBAT | — | Battery positive supply to/from ESC |
| 2 | GND | — | Battery ground |
| 3 | BAT_CURRENT | GPIO41 | Analog current-sense input (`BATT_CURRENT_SENS`, ADC channel 1) |
| 4 | TELEM_RX | GPIO37 | ESC sensor telemetry RX (not yet declared in `hwdef.dat`) |
| 5 | ESC1 | GPIO28 | PWM motor output 1 (ArduPilot GPIO 50) |
| 6 | ESC2 | GPIO29 | PWM motor output 2 (ArduPilot GPIO 51) |
| 7 | ESC3 | GPIO30 | PWM motor output 3 (ArduPilot GPIO 52) |
| 8 | ESC4 | GPIO31 | PWM motor output 4 (ArduPilot GPIO 53) |

`J1` pin 4 (`TELEM_RX` / `GPIO37`) is an RX-only input for ESC telemetry
protocols (KISS, BLHeli32, Scorpion, etc.). It is not yet declared in
`hwdef.dat` because ArduPilot's RP2350 serial path requires a matched TX+RX
pair to register a port.

### J2 — CRSF / ELRS Receiver Connector (4-pin)

`J2` is the RC receiver input connector, intended for CRSF/ELRS serial
receivers. In ArduPilot this maps to `SERIAL3` (PIO UART).

| J2 Pin | Signal | GPIO | Notes |
|--------|--------|------|-------|
| 1 | FTRX_VOUT_5V | — | 5 V supply to receiver |
| 2 | GND | — | Ground |
| 3 | FTRX_RX | GPIO21 | FC receives from receiver (`SERIAL3` RX) |
| 4 | FTRX_TX | GPIO20 | FC transmits to receiver (`SERIAL3` TX) |

### J3 — microSD Card Socket (9-pin)

`J3` is the onboard microSD card socket. Laurel uses SPI mode via `SPI1`.
The socket is the standard spring-loaded push-push type.

| J3 Pin | Signal | GPIO | Notes |
|--------|--------|------|-------|
| 1 | DAT2 | — | Not connected in SPI mode |
| 2 | CD/DAT3 | GPIO25 | SPI chip-select (`SDCARD_SPI_CS`) |
| 3 | CMD | GPIO27 | SPI MOSI (`SPI1_SDO`) |
| 4 | VDD | — | 3.3 V supply to card |
| 5 | CLK | GPIO26 | SPI clock (`SPI1_SCK`) |
| 6 | VSS/GND | — | Ground |
| 7 | DAT0 | GPIO24 | SPI MISO (`SPI1_SDI`) |
| 8 | DAT1 | — | Not connected in SPI mode |
| 9 | DET | — | Card-detect switch (not connected in hwdef) |

### J5 — Digital VTX / SBus Connector (6-pin)

`J5` carries both the digital video transmitter serial link (MSP DisplayPort
via `SERIAL1`) and the SBUS receiver input.

| J5 Pin | Signal | GPIO | Notes |
|--------|--------|------|-------|
| 1 | DVTX_VOUT_9V | — | 9 V supply to digital VTX |
| 2 | GND | — | Ground |
| 3 | DVTX_TX | GPIO12 | FC transmits to VTX (`SERIAL1` TX, MSP DisplayPort) |
| 4 | DVTX_RX | GPIO13 | FC receives from VTX (`SERIAL1` RX) |
| 5 | GND | — | Ground |
| 6 | DVTX_SBUS | GPIO36 | SBUS receiver input (RX-only; not yet declared in `hwdef.dat`) |

`J5` pin 1 supplies 9 V from the 9 V BEC (`BEC_9V_EN` / GPIO15). This rail
is **disabled by default** in the ArduPilot firmware (GPIO15 held low). Drive
GPIO15 high to power the VTX.

`J5` pin 6 (`GPIO36`) is RX-only and is not yet declared in `hwdef.dat`
for the same reason as `J1` pin 4 — no matched TX pin for a full serial port.

### J6 — I2C / External Compass Connector (4-pin)

`J6` exposes the `I2C0` bus. An external GPS+compass module's magnetometer
connects here. `AP_COMPASS_PROBING_ENABLED` is enabled, so any I2C compass
on this bus will be detected automatically.

| J6 Pin | Signal | GPIO | Notes |
|--------|--------|------|-------|
| 1 | I2C_VOUT_5V | — | 5 V supply |
| 2 | GND | — | Ground |
| 3 | SDA | GPIO44 | `I2C0` data (`I2C0_SDA`) |
| 4 | SCL | GPIO45 | `I2C0` clock (`I2C0_SCL`) |

Note: `J6` shares the `I2C0` bus with the onboard `DPS310` barometer. Both
can coexist as long as the compass address does not conflict with `0x76`.

### J7 — GPS / GNSS Connector (4-pin)

`J7` is the GPS serial connector. In ArduPilot this maps to `SERIAL2`
(hardware UART1, default protocol: GPS).

| J7 Pin | Signal | GPIO | Notes |
|--------|--------|------|-------|
| 1 | GNSS_VOUT_5V | — | 5 V supply to GPS module |
| 2 | GND | — | Ground |
| 3 | GNSS_RX | GPIO9 | FC receives from GPS (`SERIAL2` RX) |
| 4 | GNSS_TX | GPIO8 | FC transmits to GPS (`SERIAL2` TX) |

A combined GPS+compass module uses **both** `J7` (UART) and `J6` (I2C
compass). Connect `J7` for the GNSS serial link and `J6` for the
magnetometer.

### J8 — Addressable LED Connector (3-pin)

`J8` is the RGB LED strip output. The ArduPilot serial-LED subsystem is
not yet enabled on the RP2350 target, so this output is currently inactive.

| J8 Pin | Signal | GPIO | Notes |
|--------|--------|------|-------|
| 1 | LED_VOUT_5V | — | 5 V supply to LED strip |
| 2 | GND | — | Ground |
| 3 | ONE_WIRE_LED | GPIO38 | WS2812 / addressable LED data |

### J10 — Spare UART Connector (4-pin)

`J10` exposes the second PIO UART (`SERIAL4`). Default ArduPilot protocol:
MAVLink2.

| J10 Pin | Signal | GPIO | Notes |
|---------|--------|------|-------|
| 1 | SPARE_UART_VOUT_5V | — | 5 V supply |
| 2 | GND | — | Ground |
| 3 | SPARE_UART_RX | GPIO35 | FC receives (`SERIAL4` RX) |
| 4 | SPARE_UART_TX | GPIO34 | FC transmits (`SERIAL4` TX) |

### J11 — Spare GPIO Connector (6-pin)

`J11` breaks out spare GPIOs and an analog sense input. Specific GPIO
assignments are not yet confirmed from the schematic.

| J11 Pin | Signal | GPIO | Notes |
|---------|--------|------|-------|
| 1 | SPARE_VOUT_5V | — | 5 V supply |
| 2 | GND | — | Ground |
| 3 | SPARE_GPIO1 | TBD | |
| 4 | SPARE_GPIO2 | TBD | |
| 5 | SPARE_GPIO3 | TBD | |
| 6 | SPARE_ASNS | TBD | Analog sense input |

### J12 — SWD Debug Connector (3-pin)

`J12` exposes the RP2350 SWD debug port. It is the only connector not on the
PCB edge — a small 3-pin header near the middle of the board that points
vertically when the PCB is flat.

| J12 Pin | Signal | Notes |
|---------|--------|-------|
| 1 | SWCLK | SWD clock (nearest board edge) |
| 2 | GND | Ground |
| 3 | SWDIO | SWD data |

## Firmware Building

Build the application firmware:

```bash
./waf configure --board=Laurel --debug
./waf copter -j12
```

Build the Laurel bootloader:

```bash
./waf configure --board=Laurel --debug --bootloader
./waf bootloader -j12
```

> **Important:** `waf configure` must be re-run whenever `hwdef.dat` or
> `rp2350_ramfunc2_registry.txt` is edited.  These files are consumed at
> configure time to generate linker scripts and compilation flags; a
> subsequent `waf copter` without reconfiguring will silently use stale
> generated files and produce an incorrect binary.

Expected output artifacts:

- `build/Laurel/bin/arducopter`
- `build/Laurel/bin/arducopter.bin`
- `build/Laurel/bin/arducopter.apj`

## Flashing

### RP2350 ROM BOOTSEL path

Because Laurel has dedicated `BOOT / SW1` and `RESET / SW2` buttons, the ROM
USB bootloader path is accessible by holding `BOOT` while asserting
`RESET`.

That path is useful for:

- initial recovery
- testing the ROM USB boot mode
- loading non-ArduPilot recovery images
- re/uploading the arduPilot bootloader in .uf2 format with: './waf bootloader --upload'

### AP bootloader path

Once the ArduPilot bootloader is installed, the preferred firmware update path
is **OpenOCD over SWD** (see below). Do not use `uploader.py` or `./waf copter --upload`
for Laurel — the uploader sends a MAVLink reboot-to-bootloader command and then waits
for the board to re-enumerate; if the board does not respond immediately it hangs
indefinitely and requires manual intervention.

### SWD / OpenOCD path (preferred for all development flashing)

Laurel exposes SWD through connector `J12` — see the Connectors section for
the full pinout. This is the preferred flash path for development: it always
works regardless of board state, never hangs, and does not require the board
to be in bootloader mode.

```bash
~/openocd-pico/openocd \
  -s ~/openocd-pico/scripts \
  -f interface/cmsis-dap.cfg \
  -f target/rp2350.cfg \
  -c "adapter speed 5000; program build/Laurel/bin/arducopter.bin verify reset exit 0x10010000"
```

**Important:** always flash `arducopter.bin` at offset `0x10010000` (the app start address,
after the 64 KB bootloader region). Never flash `arducopter_with_bl.hex` via OpenOCD —
that file contains segments at STM32 addresses and will overwrite the bootloader.

using *a* dedicated Pico2W for a debugger, running debugprobe_on_pico2.uf2
    https://github.com/raspberrypi/debugprobe/releases/download/debugprobe-v2.3.0/debugprobe_on_pico2.uf2
    as the debugger
    BOOTSEL flash the above file to a Pico2w, label it "debugger", and ..

    Debugger board pin | Debugger GPIO | Target signal        | Notes
    board pin 3        | GND           | GND                  | Common ground — mandatory
    board pin 4        | GPIO2         | SWCLK                | SWD clock
    board pin 5        | GPIO3         | SWDIO                | SWD data
    optional extras
    board pin 6        | GPIO4/UART0RX | target GPIO1 (board pin 2) | console RX←TX
    board pin 7        | GPIO5/UART0TX | target GPIO0 (board pin 1) | console TX→RX


## Debugging Laurel, see Pico2/Debugger.md as its the same process.

## Current Laurel-Specific Notes

- Laurel is an RP2350B board, so GPIOs above `GPIO29` are valid and are used heavily.
- The current hwdef enables `USB`, `SPI0`, `SPI1`, `I2C0`, `ADC`, `PWM`, watchdog, SPI-mode microSD, and MCU monitoring.
- The current target enables the 5 V rail by default and keeps the 9 V rail disabled by default during bring-up.
- `HAL_HAVE_SAFETY_SWITCH` is disabled.
- `HAL_HAVE_IMU_HEATER` is disabled.
- `HAL_BARO_ALLOW_INIT_NO_BARO` and `AP_INERTIALSENSOR_ALLOW_NO_SENSORS` are enabled to keep bring-up practical.
- The current target intentionally does not model the secondary blackbox flash.
- The current target intentionally does not expose the RX-only Laurel pads yet.
- The current target intentionally leaves the onboard RGB LED on `GPIO39` undocumented as a firmware feature because serial LED support is still disabled on RP2350.

## Known Limitations

| Feature | Status |
|---------|--------|
| DShot / BLHeli / SerialLED | Not supported on current RP2350 target |
| CAN / DroneCAN | Not supported by RP2350 hardware |
| Hardware OSD and microSD together | Not possible on Laurel hardware; current target chooses microSD |
| Secondary blackbox flash (`W25Q128JVPIM`) | CS = `PA0`/GPIO0/pin77 confirmed; SPI bus not yet confirmed — not modelled in hwdef |
| RX-only extra serial pads | Not yet represented in current serial definitions |
| Analog scaling calibration | Placeholder scale factors still in use |

## Summary

Laurel is not just a renamed Pico2 target. It is a different RP2350B flight
controller board with:
- different sensor buses
- different PWM outputs
- different ADC pins
- different flash layout expectations
- dedicated hardware `BOOT` and `RESET` buttons

Use this README and `hwdef/Laurel/hwdef.dat` together as the canonical local
reference for the current Laurel port.

## Implemented Features

This short list reflects the implemented RP2350 platform features already
landed on the Pico2 base port and inherited by the current Laurel target.

| Feature | Status |
|---------|--------|
| USB serial console and MAVLink | Implemented and working |
| USB reconnect and late-open recovery | Implemented |
| USB bootloader and app handoff | Implemented |
| Unique USB serial number | Implemented |
| Hardware UART support | Implemented |
| PIO UART support | Implemented |
| RC input capture | Implemented |
| Standard PWM outputs | Implemented |
| SPI peripheral support | Implemented |
| ADC inputs and MCU temperature | Implemented |
| Flash-backed parameter storage | Implemented |
| Watchdog support | Implemented |
| Dual-core RP2350 dispatch path | Implemented |
| SPI-mode microSD card and AP_Logger logging | Implemented and confirmed working |
| Boot without mandatory barometer | Implemented |
| Boot without mandatory IMU | Implemented |
| Build-time RP2350 pin validation | Implemented |