# Laurel hardware

Pin and connector reference for the Laurel board. `hwdef.dat` is the
authority for what the firmware uses; this file also lists the pads the
firmware does not use yet.

## Pin numbering

| Notation | Example | Meaning |
|----------|---------|---------|
| GPIO N | `GPIO12` | RP2350 GPIO number. `hwdef.dat` writes it as `PA<N>`. |
| Package pin N | pin 35 | Physical pin of the RP2350B QFN-80, for tracing the PCB. |
| Net name | `BOOTSW` | Schematic net name. |

Laurel uses the 48-GPIO RP2350B, so GPIOs above 29 are valid and used.

## MCU, flash and buttons

- RP2350B (QFN-80), 12 MHz crystal, run at 225 MHz.
- Winbond QSPI flash holding the bootloader, parameters and firmware. The
  hwdef uses 4 MB of it (`FLASH_SIZE_KB 4096`); its flash timing comments
  name a W25Q64JV, which is 8 MB.
- `RESET` (SW2) is wired to the RP2350 `RUN` pin (package pin 35) with a
  pull-up.
- `BOOT` (SW1) is on the `BOOTSW` net, tied through a resistor to the
  `QSPI_SS` strap (package pin 75). Holding it while pressing `RESET`
  starts the RP2350 ROM USB bootloader.

Neither button is a firmware input.

## GPIO assignments

| GPIO | Function | Notes |
|------|----------|-------|
| 1 | IMU chip select | ICM-42688-P on SPI0 |
| 2 | SPI0 SCK | IMU bus |
| 3 | SPI0 MOSI | IMU bus |
| 4 | SPI0 MISO | IMU bus |
| 5 | Buzzer | PWM2_B, 4 kHz nominal |
| 6 | Blue LED | |
| 7 | Green LED | |
| 8 | UART1 TX | SERIAL2, GPS (J7) |
| 9 | UART1 RX | SERIAL2, GPS (J7) |
| 12 | UART0 TX | SERIAL1, digital VTX (J5) |
| 13 | UART0 RX | SERIAL1, digital VTX (J5) |
| 14 | `BEC_5V_EN` | 5 V rail enable, high at boot |
| 15 | `BEC_9V_EN` | 9 V rail enable, low at boot |
| 17 | OSD chip select | AT7456E footprint, not used by the firmware |
| 20 | PIO UART0 TX | SERIAL3, receiver (J2) |
| 21 | PIO UART0 RX | SERIAL3, receiver (J2); also the PPM input |
| 22 | IMU data ready | |
| 23 | `IMU_CLKIN` | Not used |
| 24 | SPI1 MISO | microSD / OSD bus |
| 25 | microSD chip select | |
| 26 | SPI1 SCK | microSD / OSD bus |
| 27 | SPI1 MOSI | microSD / OSD bus |
| 28 | Motor 1 | PWM6_A |
| 29 | Motor 2 | PWM6_B |
| 30 | Motor 3 | PWM7_A |
| 31 | Motor 4 | PWM7_B |
| 34 | PIO UART1 TX | SERIAL4, spare (J10) |
| 35 | PIO UART1 RX | SERIAL4, spare (J10) |
| 36 | `DVTX_SBUS` | SBUS input on J5, not used by the firmware |
| 37 | `TELEM_RX` | ESC telemetry input on J1, not used by the firmware |
| 38 | LED strip data | J8, not used by the firmware |
| 39 | Onboard RGB LED | Not used by the firmware |
| 40 | Battery voltage | ADC0 |
| 41 | Battery current | ADC1 |
| 42 | Analog RSSI | ADC2 |
| 44 | I2C0 SDA | DPS310 barometer, J6 |
| 45 | I2C0 SCL | DPS310 barometer, J6 |

The DPS310 sits at I2C address 0x76 (its `SDO` is pulled low), and the bus
has pull-ups fitted. SPI1 carries both the microSD socket and the AT7456E
OSD footprint, which cannot be used together; the firmware uses the
microSD card. GPIO36 and GPIO37 are receive-only pads: a serial port on
this target needs a TX and RX pair, so they are not declared.

## Power rails

| Rail | Enable | Default | Feeds |
|------|--------|---------|-------|
| 5 V BEC | GPIO14 | On | J2, J6, J7, J8, J10, J11 |
| 9 V BEC | GPIO15 | Off | J5 pin 1 |

J1 pin 1 is raw battery voltage, not a regulated output.

## Connectors

### J1 - ESC (8-pin)

| Pin | Signal | GPIO |
|-----|--------|------|
| 1 | VBAT | - |
| 2 | GND | - |
| 3 | Battery current sense | 41 |
| 4 | ESC telemetry RX | 37 |
| 5 | Motor 1 | 28 |
| 6 | Motor 2 | 29 |
| 7 | Motor 3 | 30 |
| 8 | Motor 4 | 31 |

### J2 - receiver (4-pin)

| Pin | Signal | GPIO |
|-----|--------|------|
| 1 | 5 V | - |
| 2 | GND | - |
| 3 | FC RX (from receiver) | 21 |
| 4 | FC TX (to receiver) | 20 |

### J3 - microSD socket

SPI mode on SPI1: CS on GPIO25 (card pin 2), MOSI on GPIO27 (pin 3), SCK
on GPIO26 (pin 5), MISO on GPIO24 (pin 7). DAT1, DAT2 and card detect are
not connected to the firmware.

### J5 - digital VTX and SBUS (6-pin)

| Pin | Signal | GPIO |
|-----|--------|------|
| 1 | 9 V (switched by GPIO15) | - |
| 2 | GND | - |
| 3 | FC TX (to VTX) | 12 |
| 4 | FC RX (from VTX) | 13 |
| 5 | GND | - |
| 6 | SBUS input | 36 |

### J6 - I2C (4-pin)

| Pin | Signal | GPIO |
|-----|--------|------|
| 1 | 5 V | - |
| 2 | GND | - |
| 3 | SDA | 44 |
| 4 | SCL | 45 |

Shared with the onboard DPS310 at 0x76.

### J7 - GPS (4-pin)

| Pin | Signal | GPIO |
|-----|--------|------|
| 1 | 5 V | - |
| 2 | GND | - |
| 3 | FC RX (from GPS) | 9 |
| 4 | FC TX (to GPS) | 8 |

### J8 - LED strip (3-pin)

| Pin | Signal | GPIO |
|-----|--------|------|
| 1 | 5 V | - |
| 2 | GND | - |
| 3 | LED data | 38 |

### J10 - spare UART (4-pin)

| Pin | Signal | GPIO |
|-----|--------|------|
| 1 | 5 V | - |
| 2 | GND | - |
| 3 | FC RX | 35 |
| 4 | FC TX | 34 |

### J11 - spare GPIO (6-pin)

5 V, GND, three spare GPIOs and an analog sense input. The GPIO numbers
have not been traced from the schematic.

### J12 - SWD (3-pin)

The only connector away from the board edge: a vertical 3-pin header near
the middle of the board.

| Pin | Signal |
|-----|--------|
| 1 | SWCLK (nearest the board edge) |
| 2 | GND |
| 3 | SWDIO |
