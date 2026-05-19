# Laurel (RP2350B) ArduPilot Port

The Laurel target runs ArduPilot on a custom RP2350B flight controller
board built around the Raspberry Pi RP2350B (QFN-80, 48 GPIO) running at
150 MHz. Unlike the Pico2 carrier-board reference target, Laurel has its
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

- RP2350B dual-core Cortex-M33 @ 150 MHz
- 520 KB SRAM
- 8 MB main QSPI/XIP flash (`W25Q64`-class layout in current target)
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
- `SERIAL3`: RC input
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
| Buzzer | GPIO5 | Exported as `BUZZER` and mapped as the board buzzer pin |
| 5 V regulator enable | GPIO14 | `BEC_5V_EN`, driven high by default in application firmware |
| 9 V regulator enable | GPIO15 | `BEC_9V_EN`, held low by default for bring-up |

Laurel also has an onboard single-wire RGB LED on `GPIO39`, but serial LED
support remains disabled for RP2350 in this target, so that LED is not yet
bound in `hwdef.dat`.

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

Main parameter storage uses the RP2350 XIP flash, while logs and filesystem
access can use the SPI-mode microSD card on the shared SPI1 bus:

- main flash size: 8 MB
- bootloader region: first 32 KB
- parameter storage region: next 32 KB
- application region: remainder of flash
- logical parameter capacity: 8 KB with `AP_FLASH_STORAGE_QUAD_PAGE 1`

Earlier Laurel board notes indicated a second QSPI flash device for blackbox
logging. That secondary flash is not yet modelled in the current RP2350 hwdef
path, so storage currently consists of the main XIP flash plus the SPI-mode
microSD card.

## Connectors

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

Once the ArduPilot bootloader is installed, normal firmware update is
expected to use the ArduPilot bootloader over USB CDC.  ie uploader.py or './waf copter --upload'

### SWD / OpenOCD path

Laurel exposes SWD through connector `J12`
Physical identification:
- `J12` is the only connector that is not on the PCB edge.
- It is the small 3-pin header near the middle of the board.
- When the PCB is lying flat on the table, this header "points up".

Pinout, with pin 1 being the pin nearest the board edge:
- Pin 1: `SWCLK`
- Pin 2: `GND`
- Pin 3: `SWDIO`

In the hwdef this corresponds to:
- `PC0`: SWCLK
- `PC1`: SWDIO

You can Flash the ELF over SWD with OpenOCD + GDB when doing low-level bring-up or
when the USB boot path is unavailable.

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
| Secondary QSPI blackbox flash | Present in earlier board notes but not yet modelled in hwdef |
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
| Boot without mandatory barometer | Implemented |
| Boot without mandatory IMU | Implemented |
| Build-time RP2350 pin validation | Implemented |