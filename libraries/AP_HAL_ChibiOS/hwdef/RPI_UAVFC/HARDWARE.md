# RPI_UAVFC hardware

GPIO reference for the RPI_UAVFC board. The pins follow the R2 Rev C
schematic, which wins where it disagrees with the vendor GPIO assignment
sheet (it does on the ESC channel order and the regulator enables).
`hwdef.dat` is the authority for what the firmware uses; this file also
lists the pads it leaves unbound.

RPI_UAVFC is a revision of the Laurel carrier with a different pinout. Only
the clock, flash timing and SMP setup are shared with the `Laurel` target,
so do not take pin numbers from it.

## MCU and flash

- RP2350B (QFN-80), 12 MHz crystal, run at 225 MHz.
- 4 MB QSPI flash, one 64 KB erase block per region: bootloader in block
  0, parameter storage in block 1, firmware from block 2.
- No blackbox flash; logs go to the microSD card.
- SWD is not declared in the hwdef; use an external probe on the SWD
  pads.

## GPIO assignments

| GPIO | Function | Notes |
|------|----------|-------|
| 0 | Blue LED | Active low |
| 1 | Green LED | Active low |
| 2 | LED strip data | J2, output 5, through a 27 ohm resistor |
| 3 | Buzzer | PWM1_B; transducer needs a ~4 kHz drive |
| 5 | ESC telemetry | Unbound: only reaches UART1 RX, which the GPS uses |
| 6 | Motor 4 | PWM3_A |
| 7 | Motor 3 | PWM3_B |
| 8 | Motor 2 | PWM4_A |
| 9 | Motor 1 | PWM4_B |
| 12-14 | GP1-GP3 spare pads | Unbound |
| 16 | PIO UART1 TX | SERIAL4, VID connector |
| 17 | PIO UART1 RX | SERIAL4, VID connector |
| 18 | 5 V rail enable | RELAY2, on at boot |
| 19 | 9 V rail enable | RELAY3, on at boot, feeds the VID connector |
| 21 | `OSD_W` | Analog OSD output |
| 22 | `OSD_EN` | Analog OSD enable |
| 23 | `OSD_SYNC` | Analog OSD sync input |
| 24 | I2C0 SDA | Internal, DPS368 barometer at 0x76 |
| 25 | I2C0 SCL | Internal |
| 26 | IMU data ready | |
| 27 | IMU `CLKIN` | Not used |
| 28 | SPI1 MISO | microSD |
| 29 | microSD chip select | |
| 30 | SPI1 SCK | microSD |
| 31 | SPI1 MOSI | microSD |
| 32 | SPI0 MISO | IMU |
| 33 | IMU chip select | |
| 34 | SPI0 SCK | IMU |
| 35 | SPI0 MOSI | IMU |
| 36 | UART1 TX | SERIAL2, GPS connector |
| 37 | UART1 RX | SERIAL2, GPS connector |
| 38 | I2C1 SDA | External, GPS connector |
| 39 | I2C1 SCL | External, GPS connector |
| 40 | Analog spare | ADC0, AN1 pad, used as RSSI |
| 41 | DVTX SBUS RX | Unbound, see below |
| 42 | PIO UART0 TX | SERIAL3, RADIO connector |
| 43 | PIO UART0 RX | SERIAL3, RADIO connector |
| 44 | UART0 TX | SERIAL1, DVTX connector |
| 45 | UART0 RX | SERIAL1, DVTX connector |
| 46 | Battery voltage | ADC6 |
| 47 | Battery current | ADC7, from the ESC |

The board sheet names the serial ports UART0 to UART3. RP2350 has two
hardware UARTs, so the sheet's UART2 (RADIO) and UART3 (VID) are PIO
UARTs. Betaflight numbers those two PIOUART0 and PIOUART1, which are
ArduPilot's SERIAL3 and SERIAL4.

The ESC connector is wired in descending order: DSHOT1 on the sheet is
GPIO9 and DSHOT4 is GPIO6. Motors 1-2 are PWM slice 4 and motors 3-4
slice 3, so each output group is one slice.

The IMU on the R2 Rev C schematic is an ICM-56686; earlier samples carry an
ICM-42688-P. The Invensensev3 driver detects either, so the same firmware
runs on both.

GPIO41 cannot be a hardware UART receive without taking UART1 from the GPS,
and a third PIO UART would need PIO1, which the analog OSD uses. SBUS works
on the RADIO connector instead.

## LED strip connector J2

J2 is a 3-pin JST-SH compatible right-angle header, with two shield pins
tied to ground. No LED is fitted on the board.

| Pin | Signal |
|-----|--------|
| 1 | 5 V, from the switched peripheral rail |
| 2 | GND |
| 3 | Data, GPIO2 |

## Regulator enables

Both enables drive an MP4334 EN pin with a pull-down, so they are active
high and the rail is off while the pin is low or floating. An earlier board
revision had GPIO18 and GPIO19 the other way round.
