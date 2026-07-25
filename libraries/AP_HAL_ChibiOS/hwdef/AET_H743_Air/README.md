# AET-H743-Air Flight Controller

The AET-H743-Air is a flight controller designed and produced by AeroEggTech, based on the STM32H743 microcontroller.

## Features

- STM32H743 (480MHz) microcontroller
- Dual BMI270 IMUs
- AT7456E OSD
- 11 PWM / DShot outputs (one dedicated to WS2812 LED)
- 5 UARTs (USART1, USART2, USART3, UART4, UART5) + USB
- 1 CAN bus
- USB-C (virtual COM port)
- DPS310 / SPL06 barometer (I2C)
- Onboard SD NAND (CSNP1GCR01-BOW, 128MB) for log storage
- 2 I2C buses
- 2-way analog camera switch
- RGB status LED and WS2812 on PWM11
- Beeper
- 5V/3.3V power outputs

## Where to Buy

You can purchase the AET-H743-Air from the [AeroEggTech Official Store](https://www.aeroeggtech.com/).

## Mechanical

- Dimensions: 52 x 31 x 20 mm
- Weight: 55g

## Pinout

![AET-H743-Air overview](AET-H743-Air-overview.png)

![AET-H743-Air core board](AET-H743-Air_core_board.png)

![AET-H743-Air power board](AET-H743-Air_power_board.png)

## Power supply

The AET-H743-Air supports 2-6S LiPo battery input. It provides 5V (2A) and 3.3V outputs for peripherals. Please refer to the manufacturer's documentation for exact power distribution.

## UART Mapping

All UARTs are DMA capable. Only SERIAL0-5 exist on this board.

Several connectors share the same MCU UART nets. Connect **one device per UART** only.

| SERIAL | MCU    | Default protocol | Connectors / notes |
|--------|--------|------------------|--------------------|
| 0      | USB    | MAVLink2         | USB-C |
| 1      | USART1 | MAVLink2         | Telemetry/FM30 (UART1 TX/RX) |
| 2      | USART2 | GPS              | GPS/compass (UART2 TX/RX) |
| 3      | USART3 | RC input         | **Shared UART3:** telemetry RX, ELRS (RX+TX), X4, SBUS pad |
| 4      | UART4  | GPS              | External tuning board (UART4 TX/RX) |
| 5      | UART5  | None             | Analog VTX connector (UART5 TX), X4 (TX/RX) |

## RC Input

RC input defaults to **SERIAL3 (USART3)**. The SBUS pad, ELRS connector, telemetry RX pin, and X4 UART3_RX pin are the **same** USART3 RX net. Connect a receiver to **one** of these only.

There is no dedicated PPM or RCININT pin. SBUS is **not** hardware-inverted on this board; `SERIAL3_OPTIONS` defaults to `1` (InvertRX) for SBUS receivers.

- SBUS: connect to the SBUS pad or any connector carrying UART3_RX. Default `SERIAL3_PROTOCOL` = 23 (RC input).
- ELRS/CRSF: connect to the ELRS connector (UART3 TX+RX). Set `SERIAL3_OPTIONS` = `0`.
- FPort requires `SERIAL3_OPTIONS` = `15`.
- SRXL2 requires `SERIAL3_OPTIONS` = `4` and connects only the TX pin.

See [RC systems](https://ardupilot.org/plane/docs/common-rc-systems.html) for more detail.

## OSD Support

The AET-H743-Air supports onboard analog OSD using an AT7456E chip. Connect the analog VTX video input to the VTX connector (VIDEO_OUT). The VTX connector provides analog video and 12V power; it is **not** a digital VTX control port.

## PWM Output

The AET-H743-Air supports up to 11 PWM outputs.

All channels support DShot. Bidirectional DShot is supported on outputs 1, 3, 4, 6, 7, and 9.

Outputs are grouped and every output within a group must use the same output protocol:

1, 2, 3 are Group 1;

4, 5, 6 are Group 2;

7, 8, 9, 10 are Group 3;

11 (LED) is Group 4;

Output 11 is dedicated to WS2812 LED (NeoPixel), not for motor/servo.

## Battery Monitoring

The board has two internal voltage sensors and two current sensor inputs (one onboard, one external). Default parameters:

- `BATT_MONITOR` = 4 (Analog)
- `BATT_VOLT_PIN` = 10
- `BATT_CURR_PIN` = 11
- `BATT_VOLT_MULT` = 11.0
- `BATT_AMP_PERVLT` = 40.0
- `BATT2_VOLT_PIN` = 4
- `BATT2_CURR_PIN` = 9
- `BATT2_VOLT_MULT` = 11.0

The onboard current sensor values above apply to battery 1. For the external current sensor, set `BATT2_AMP_PERVLT` to match your sensor.

The voltage sensors can handle up to 6S LiPo batteries.

## Compass

The AET-H743-Air has no built-in compass. Use an external I2C compass on I2C1. Autoprobing is enabled.

## Analog RSSI

An analog RSSI input is available. `RSSI_ANA_PIN` defaults to 8.

## Camera Switch

GPIO 81 controls the analog camera switch (CAM1/CAM2). `RELAY1_PIN` defaults to 81.

Example (using an RC channel to toggle cameras via Relay 1):

- `RELAY1_FUNCTION` = 1
- `RELAY1_PIN` = 81
- `RCx_OPTION` = 34 (Relay1 Control)

## WS2812 LED (NeoPixel)

PWM output 11 is dedicated to WS2812 LED control. Board and NeoPixel notification is enabled by default. To use it:

1. Connect WS2812 LED strip data line to the PWM11 / LED output.
2. Set `NTF_LED_LEN` to the number of LEDs.
3. `SERVO11_FUNCTION` = 120 (NeoPixel) is set in `defaults.parm`.

The LED will indicate flight status (e.g., red when disarmed, green when armed).

## GPIOs

The motor/servo outputs and other pads are available as ArduPilot GPIOs (for relays, PINIO, etc.).

| Label           | GPIO |
|-----------------|------|
| MOTOR1          | 50   |
| MOTOR2          | 51   |
| MOTOR3          | 52   |
| MOTOR4          | 53   |
| MOTOR5          | 54   |
| MOTOR6          | 55   |
| MOTOR7          | 56   |
| MOTOR8          | 57   |
| MOTOR9          | 58   |
| MOTOR10         | 59   |
| MOTOR11 / LED   | 60   |
| Beeper          | 32   |
| Camera switch   | 81   |
| CAN silent      | 70   |

## I2C Buses

- I2C1: external compass/GPS
- I2C2: onboard barometer (SPL06/DPS310)

## CAN Bus

One CAN interface (CAN1) is available on the CAN connector.

## Storage

The board includes an onboard SD NAND chip (CSNP1GCR01-BOW, 1Gbit / 128MB) for data logging. A FAT filesystem is mounted at `/` for log storage.

Flight logs are written to `/APM/LOGxxxx.BIN` on the onboard storage. If the storage fails to mount at boot, try increasing `BRD_SD_SLOWDOWN`.

## Firmware

Firmware for the AET-H743-Air can be found [here](https://firmware.ardupilot.org) in sub-folders labeled `AET_H743_Air`.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "with_bl.hex" firmware using your favorite DFU loading tool (e.g., Mission Planner).

Once the initial firmware is loaded, you can update the firmware using any ArduPilot ground station software. Updates should be done with the `*.apj` firmware files.

## License

This hardware definition is released under the terms of the GNU General Public License v3.

## Credits

- AeroEggTech (AET)
- ArduPilot community
