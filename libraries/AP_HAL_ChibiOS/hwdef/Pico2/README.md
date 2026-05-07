# Raspberry Pi Pico 2 (RP2350) ArduPilot Port

The Pico2 target runs ArduPilot on the Raspberry Pi Pico 2 module
(RP2350 Cortex-M33 @ 150 MHz). The hwdef.dat is designed for a
carrier board that provides SPI/I2C sensors and exposes the RP2350's
full GPIO range including pins above GPIO29 (available on the Pico2
castellated edges).

## Pin Numbering Conventions

Three different pin numbering schemes appear in this documentation and in
datasheets. Always check which one is being referred to before wiring:

| Notation | Example | Meaning |
|----------|---------|---------|
| **GPIO N** / **GP N** | `GPIO12`, `GP2` | Logical RP2350 GPIO number. Used in hwdef.dat as `PA<N>`, in firmware registers, and in datasheets. This is the primary naming used throughout ArduPilot code. |
| **board pin N** | `board pin 16` | Physical header pin on the Pico2W PCB (the castellation / through-hole pads). Numbered 1–20 down the **left** column and 21–40 back up the **right** column, when the board is oriented with the USB connector at the top and the component side facing you. Not the same as the GPIO number. |
| **RP2350 package pin N** | `QFN-80 pin 11` | Physical metal pad/leg on the bare RP2350 chip package. Numbered anticlockwise from the marked corner. Two variants exist: **RP2350A** (7×7 QFN-60, 30 GPIO, GPIO0–GPIO29) is used on the standard Pico2 and Pico2W modules; **RP2350B** (10×10 QFN-80, 48 GPIO, GPIO0–GPIO47) is a bare chip used on custom carrier boards that need GPIOs above GPIO29. Package pin numbers appear in the RP2350 datasheet §1.4 (GPIO table) and are only relevant when designing a custom carrier board or interpreting oscilloscope probes on the die pads. |

### Pico2W board pin ↔ GPIO quick-reference

```
board pin 1  = GPIO0    board pin 21 = GPIO16
board pin 2  = GPIO1    board pin 22 = GPIO17
board pin 3  = GND      board pin 23 = GND
board pin 4  = GPIO2    board pin 24 = GPIO18
board pin 5  = GPIO3    board pin 25 = GPIO19
board pin 6  = GPIO4    board pin 26 = GPIO20
board pin 7  = GPIO5    board pin 27 = GPIO21
board pin 8  = GND      board pin 28 = GND
board pin 9  = GPIO6    board pin 29 = GPIO22
board pin 10 = GPIO7    board pin 30 = RUN (reset)
board pin 11 = GPIO8    board pin 31 = GPIO26 / ADC0
board pin 12 = GPIO9    board pin 32 = GPIO27 / ADC1
board pin 13 = GND      board pin 33 = GND
board pin 14 = GPIO10   board pin 34 = GPIO28 / ADC2
board pin 15 = GPIO11   board pin 35 = GPIO29 / ADC3
board pin 16 = GPIO12   board pin 36 = 3V3 (out)
board pin 17 = GPIO13   board pin 37 = 3V3_EN
board pin 18 = GND      board pin 38 = GND
board pin 19 = GPIO14   board pin 39 = VSYS
board pin 20 = GPIO15   board pin 40 = VBUS
```

> **Note:** GPIO30–GPIO47 do **not** exist on the RP2350A (QFN-60) used in the
> standard Pico2 and Pico2W modules — they are only available on the RP2350B
> (QFN-80, 48-GPIO bare chip). The carrier board hwdef uses GPIOs above GPIO29
> (e.g. GPIO32/40/42/43 for SPI buses) and therefore requires an RP2350B-based
> board, not a standard Pico2W module. See the RP2350 datasheet §1.4 for QFN-80
> package pin numbers for those signals.

## Features

- RP2350 dual-core Cortex-M33 @ 150 MHz (ArduPilot uses one core)
- 520 KB SRAM
- 4 MB external QSPI flash (parameter storage in pages 8–15, at 0x10008000–0x1000FFFF)
- USB CDC serial (SERIAL0)
- 2 hardware UARTs + 3 PIO UARTs (5 telemetry/GPS ports)
- 8 PWM outputs (GPIO 0–7, 4 PWM slices × 2 channels)
- SPI0 and SPI1 buses for IMU, barometer, and compass sensors
- I2C1 bus (GPIO 15/18) for external compass / GPS
- 2 ADC channels for battery voltage and current sensing
- RC input via GPIO PAL callback (GPIO 16, PULLDOWN)
- SBUS/inverted UART on UART0/UART1 via hardware INOVER
- Watchdog (2 s timeout)
- IMU temperature monitoring (HEAT log messages)

## UART Mapping

| Serial port | Function | GPIO | Pico2W board pins |
|-------------|----------|------|-------------------|
| SERIAL0 | USB / console | — | USB connector |
| SERIAL1 | Telem1 / GPS | GPIO12 / GPIO13 | TX=board pin 16, RX=board pin 17 |
| SERIAL2 | Telem2 | GPIO10 / GPIO11 | TX=board pin 14, RX=board pin 15 |
| SERIAL3 | GPS / spare (PIOUART0) | GPIO14 / GPIO17 | TX=board pin 19, RX=board pin 22 |
| SERIAL4 | spare (PIOUART1) | GPIO19 / GPIO20 | TX=board pin 25, RX=board pin 26 |
| SERIAL5 | spare (PIOUART2) | GPIO21 / GPIO27 | TX=board pin 27, RX=board pin 32 |
| SERIAL6 | spare (PIOUART3) | GPIO30 / GPIO31 | carrier-board only (no standard Pico2 header pins) |

Hardware RTS/CTS flow control is currently wired for SERIAL1 (UART0) only:
- UART0_CTS: GPIO18 (board pin 24)
- UART0_RTS: GPIO15 (board pin 20)

SBUS (100 kbps, 8E2, inverted) is supported on SERIAL1/SERIAL2 using
the RP2350 GPIO INOVER bit — no external inverter required. Set
`SERIAL_n_PROTOCOL=23` (RC Input) and `SERIAL_n_OPTIONS=3` on the
connected port.

## RC Input

Connect RC receiver signal to **GPIO16** (board pin 21). The pin is
PULLDOWN; for PWM/PPM receivers a 10 kΩ pull-up to 3.3 V may be
needed. SBUS can be wired to **GPIO13** (UART0 RX, SERIAL1, board pin 17).

## PWM / Servo Outputs

| Output | GPIO | Pico2W board pin |
|--------|------|------------------|
| PWM1 | GPIO0 | board pin 1 |
| PWM2 | GPIO1 | board pin 2 |
| PWM3 | GPIO2 | board pin 4 |
| PWM4 | GPIO3 | board pin 5 |
| PWM5 | GPIO4 | board pin 6 |
| PWM6 | GPIO5 | board pin 7 |
| PWM7 | GPIO6 | board pin 9 |
| PWM8 | GPIO7 | board pin 10 |

Standard 50 Hz PWM servo output. DShot is not supported (no
timer DMA on RP2350). PWM outputs can also be used as GPIOs by
setting `BRD_PWM_COUNT` < 8; GPIO numbers start at 50 (PWM1=50 ...
PWM8=57).

## SPI Buses (carrier board only)

Pins above GPIO29 are not accessible on the standard Pico2 breakout
header — they appear as castellated-edge pads used by a flight
controller carrier board.

**SPI0** (barometer bus)

| Signal | GPIO |
|--------|------|
| SCK | GPIO 22 |
| MISO (RX) | GPIO 32 |
| MOSI (TX) | GPIO 35 |
| BARO_EXT_CS | GPIO 25 |

**SPI1** (IMU bus)

| Signal | GPIO |
|--------|------|
| SCK | GPIO 42 |
| MISO | GPIO 40 |
| MOSI | GPIO 43 |
| MPU_CS | GPIO 24 |
| MAG_CS | GPIO 23 |
| GYRO_EXT_CS | GPIO 26 |

## I2C Bus

| Signal | GPIO | Pico2W board pin |
|--------|------|------------------|
| I2C1 SCL | GPIO15 | board pin 20 |
| I2C1 SDA | GPIO18 | board pin 24 |

`AP_COMPASS_PROBING_ENABLED 1`: ArduPilot auto-probes for HMC5883,
IST8310, QMC5883 and other common I2C compasses.

## Battery Monitoring

| Signal | GPIO | Pico2W board pin | Default scale |
|--------|------|------------------|---------------|
| BATT_VOLTAGE_SENS | GPIO28 | board pin 34 | 10.1 V/V |
| BATT_CURRENT_SENS | GPIO29 | board pin 35 | 17.0 A/V |

## Sensor Stack

The hwdef probes the following sensors; missing sensors are silently
skipped via WHOAMI check:

| Sensor | Bus | CS pin |
|--------|-----|--------|
| Invensense gen-1 IMU (mpu6000 / mpu9250) | SPI1 | MPU_CS |
| Invensense gen-2 IMU (ICM20948 / AK09916) | SPI1 | MPU_CS |
| MS5611 barometer | SPI0 | BARO_EXT_CS |
| Compass (I2C auto-probe) | I2C1 | — |
| LSM9DS0 gyro-only | SPI1 | GYRO_EXT_CS |

Note: LSM9DS0 requires two CS lines (gyro + accel/mag). The accel/mag
CS (`ACCEL_EXT_CS`) is not mapped on this carrier; only the gyro
channel can be detected.

## IMU Temperature Monitoring

`HAL_HAVE_IMU_HEATER 1` is enabled. No physical heating element
exists on the standard Pico2; `HAL_HEATER_GPIO_PIN` is not defined.
The subsystem still logs IMU temperature to the `HEAT` log message
(1 Hz) and exposes the `BRD_IMU_TARGTEMP` parameter for arming
temperature checks. An external heater resistor + MOSFET can be added
by defining `HAL_HEATER_GPIO_PIN` in a local hwdef override.

## Parameter Storage

Parameters are stored in a dedicated 32 KB region at 0x10008000–0x1000FFFF (pages 8–15, two 16KB halves) of the 4 MB QSPI flash. `AP_FLASH_STORAGE_QUAD_PAGE 1` aggregates 4 × 4 KB physical pages into one 16 KB logical sector for correct wear-levelling. Storage capacity is 8 KB (vs 16 KB on CubeBlack).

If an external SPI FRAM (ramtron-compatible) is wired to a free
SPI bus, uncomment the `#SPIDEV ramtron` and `#PA_n FRAM_CS CS`
lines in hwdef.dat to use it instead of flash.

## Firmware Building

```bash
this will usually make/setup the mavlink headers and build both targets.
./waf configure --board=Pico2 --debug
./waf copter -j12
./waf configure --board=Pico2 --debug --bootloader
./waf bootloader -j12
```

## Firmware Flashing

```bash
./waf configure --board=Pico2 --bootloader
[ Hold the BOOTSEL button on the Pico2 while plugging in USB,
then release. The board will appear as a mass-storage device.]
./waf bootloader --upload
[ .uf2 bootloader will be uploaded for you and it will reboot, probably staying as a mass-storage device. ]

./waf configure --board=Pico2
./waf copter --upload
[ If the board does not respond within 1-2 seconds, unplug and re-plug the USB connector ]


## Known Limitations

| Feature | Status |
|---------|--------|
| DShot / BLHeli / SerialLED | Not supported — no timer DMA on RP2350 |
| UART RTS/CTS flow control | Not supported — PIOUART has no flow control |
| CAN / DroneCAN | Not feasible — RP2350 has no CAN peripheral |
| IOMCU | Not applicable — Pico2 drives servos directly |
| microSD / FAT logging | Not wired — no SDIO, SPI-mode SD not included |
| I2C2 | Not available — not pinned out on the standard carrier |
| Hardware RTC | GPS time used; RP2350 removed hardware RTC (POWMAN_TIMER) |
| Storage | 8 KB parameter storage (CubeBlack has 16 KB) |
| Gyro FFT / DSP | Deferred — CMSIS-DSP needs ArduPilot integration for RP2350 |

## Reference Documentation

The RP2350 datasheet is not included in this directory as its a huge pdf but google RP-008373-DS-2-rp2350-datasheet.pdf