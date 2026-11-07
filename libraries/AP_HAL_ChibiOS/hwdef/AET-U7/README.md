# AET-U7 Flight Controller

The AET-U7 is a flight controller designed and produced by AeroEggTech, based on the STM32H753 microcontroller.

## Features

- STM32H753 (480MHz) microcontroller (ArduPilot builds with the STM32H743xx target)
- Triple IMUs: ICM-45686 + dual ICM-42688 (SPI)
- Dual barometers: ICP-20100 and SPL06 (I2C)
- Onboard IST8310 compass
- 12 PWM / DShot outputs (plus PWM13 dedicated to WS2812 LED)
- 7 UARTs + USB (UART7 has RTS/CTS)
- 1 CAN bus
- USB-C
- MicroSD card slot (SDMMC)
- 2 I2C buses
- Dual analog power monitoring (PMU)
- Analog RSSI and airspeed inputs
- Dual status LEDs (blue/green) and WS2812 on PWM13
- 5V 5A BEC and VX servo BEC (5V or 7.2V, 5A)

## Where to Buy

You can purchase the AET-U7 from the [AeroEggTech Official Store](https://www.aeroeggtech.com/).

## Mechanical

- Dimensions: 85 x 52 x 21 mm

## Pinout

![AET-U7 overall preview](AET-U7_overall_preview.jpg)

![AET-U7 pin definition](AET-U7_pin_definition.jpg)

![AET-U7 pin position](AET-U7_pin_position.jpg)

![AET-U7 power board](AET-U7_power_board.jpg)

## Power supply

The AET-U7 PMU is designed for 12S (50.4V) input, with a transient maximum of 60V. The PMU current sensor is rated up to 120A.

Onboard BECs:

- 5V BEC, 5A
- VX servo rail BEC, selectable 5V or 7.2V, 5A

Please refer to the manufacturer's documentation for exact power distribution and VX voltage selection.

## UART Mapping

All UARTs are DMA capable.

| SERIAL | MCU    | Default protocol | Connectors / notes |
|--------|--------|------------------|--------------------|
| 0      | USB    | MAVLink2         | USB-C |
| 1      | USART1 | MAVLink2         | UART1 connector |
| 2      | USART2 | GPS              | GPS (U2) connector |
| 3      | USART3 | MAVLink2         | UART3 on I2C1/UART3 connector (115200 baud default) |
| 4      | UART4  | GPS              | UART4 connector |
| 5      | USB2   | MAVLink2         | Second USB serial endpoint |
| 6      | USART6 | RC input         | UART6 connector |
| 7      | UART7  | MAVLink2         | UART7 connector with RTS/CTS (115200 baud default) |
| 8      | UART8  | None             | UART8 connector |

## RC Input

RC input defaults to **SERIAL6 (USART6)**. All common serial RC protocols are supported except PPM. There is no dedicated PPM / RCININT pin.

- SBUS / CRSF / ELRS and similar: connect to the UART6 connector (or any free UART with `SERIALn_PROTOCOL` = 23).
- FPort requires `SERIALn_OPTIONS` = `15`.
- CRSF/ELRS typically uses `SERIALn_OPTIONS` = `0`.
- SRXL2 requires `SERIALn_OPTIONS` = `4` and connects only the TX pin.

If RC is moved to another UART, set that port to protocol 23 and change `SERIAL6_PROTOCOL` away from 23.

See [RC systems](https://ardupilot.org/plane/docs/common-rc-systems.html) for more detail.

## PWM Output

The AET-U7 supports 12 PWM outputs for motors/servos. PWM13 is dedicated to WS2812 LED (NeoPixel).

All motor/servo channels support DShot. Bidirectional DShot is not enabled on this board.

Outputs are grouped and every output within a group must use the same output protocol:

1, 2 are Group 1;

3, 4, 5, 6 are Group 2;

7, 8, 9, 10 are Group 3;

11, 12 are Group 4;

13 (LED) is Group 5;

## Battery Monitoring

The board has two voltage sensor inputs and two current sensor inputs (battery 1 via the PMU connector, battery 2 via the VBAT2/CURR2 pads). Default parameters:

- `BATT_MONITOR` = 4 (Analog)
- `BATT_VOLT_PIN` = 10
- `BATT_CURR_PIN` = 11
- `BATT_VOLT_MULT` = 26.0
- `BATT_AMP_PERVLT` = 40.0
- `BATT2_VOLT_PIN` = 18
- `BATT2_CURR_PIN` = 7
- `BATT2_VOLT_MULT` = 26.0

The battery 1 current scale above matches the PMU. For battery 2 current, set `BATT2_AMP_PERVLT` to match your external sensor.

The voltage sensors support up to 12S, consistent with the PMU design.

## Compass

The AET-U7 has an onboard IST8310 compass at I2C address `0x0E`. External I2C compasses can also be autoprobed.

## Analog RSSI

An analog RSSI input is available on the RSSI pad. `RSSI_ANA_PIN` defaults to 8.

## Analog Airspeed

An analog airspeed input is available on the AIRSPD pad. The default airspeed analog pin is 4.

## WS2812 LED (NeoPixel)

PWM output 13 is dedicated to WS2812 LED control. Board and NeoPixel notification defaults are enabled in the firmware. To use it:

1. Connect the WS2812 data line to the LED / PWM13 output.
2. Set `NTF_LED_LEN` to the number of LEDs.
3. `SERVO13_FUNCTION` = 120 (NeoPixel) is set in `defaults.parm`.

## GPIOs

The motor/servo outputs and other pads are available as ArduPilot GPIOs (for relays, etc.).

| Label        | GPIO |
|--------------|------|
| MOTOR1       | 50   |
| MOTOR2       | 51   |
| MOTOR3       | 52   |
| MOTOR4       | 53   |
| MOTOR5       | 54   |
| MOTOR6       | 55   |
| MOTOR7       | 56   |
| MOTOR8       | 57   |
| MOTOR9       | 58   |
| MOTOR10      | 59   |
| MOTOR11      | 60   |
| MOTOR12      | 61   |
| LED / PWM13  | 62   |
| PIO1         | 81   |
| PIO2         | 82   |
| CAN silent   | 70   |

## I2C Buses

- I2C2: onboard barometers and IST8310 (ArduPilot bus 0)
- I2C1: external / expansion (ArduPilot bus 1)

## CAN Bus

One CAN bus (CAN1) is provided. The board has two physical CAN connectors on the same bus for convenient daisy-chaining.

## Storage

A MicroSD card slot is provided for data logging (SDMMC). Flight logs are written under `/APM/` when the card is mounted.

## Firmware

Firmware for the AET-U7 can be found [here](https://firmware.ardupilot.org) in sub-folders labeled `AET-U7`.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "with_bl.hex" firmware using your favorite DFU loading tool (e.g., Mission Planner).

Once the initial firmware is loaded, you can update the firmware using any ArduPilot ground station software. Updates should be done with the `*.apj` firmware files.

## License

This hardware definition is released under the terms of the GNU General Public License v3.

## Credits

- AeroEggTech (AET)
- ArduPilot community
