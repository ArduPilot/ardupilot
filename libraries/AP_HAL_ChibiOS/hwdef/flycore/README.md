# AMOV Flycore Flight Controller

The AMOV Flycore is an STM32H743-based flight controller manufactured by
[AMOV](https://amovlab.com/). It includes an onboard UM982 GNSS receiver,
dual IMUs, an onboard barometer, 8 PWM outputs, dual CAN, three telemetry
ports, and a standard SD card slot.

![AMOV Flycore](amovlab_flycore_product.jpg "AMOV Flycore")

## Where to Buy

For purchasing information, contact
[shudajun@amovauto.com](mailto:shudajun@amovauto.com).

## Specifications

- Processor
  - STM32H743, Arm Cortex-M7 at 400 MHz
  - 2 MB flash and 1 MB RAM
  - No I/O coprocessor
- Sensors
  - Bosch BMI088 IMU
  - InvenSense ICM-42688P IMU
  - MS5611 barometer
  - UM982 GNSS receiver connected internally to GPS1
  - No onboard compass
- Interfaces
  - 8 PWM outputs
  - Three telemetry ports
  - One external GPS port
  - Two CAN buses
  - Two external I2C buses
  - USB
  - RC input
  - Standard SD card slot
  - Buzzer and SWD pads
- Power
  - XT30 input, 15 V to 28 V
  - DroneCAN battery monitor connection on CAN2
- Mechanical
  - 120 mm x 55 mm x 26.3 mm
  - 60 g
  - 50 mm x 115 mm mounting pattern with 2.5 mm holes

## Pinout

![AMOV Flycore pinout](amovlab_flycore_pinout.jpg "AMOV Flycore pinout")

The external peripheral signal connectors use JST-GH connectors with
1.25 mm pitch unless otherwise noted. The POWER connector is
XT30PW(2+2)-M. USB, the SD card slot, RTK antenna connectors, and the
exposed buzzer, SWD, and debug pads use the interfaces shown in the
pinout image.

Pin 1 starts at the right side of each connector when viewed as shown below.

![AMOV Flycore pin 1 orientation](amovlab_flycore_pin1_orientation.png "Connector pin 1 orientation")

The VCC pins on signal connectors provide 5 V unless noted otherwise.

### CAN1

| Pin | Signal |
| --- | ------ |
| 1 | CAN_L |
| 2 | CAN_H |
| 3 | GND |
| 4 | VCC |

### I2C1

| Pin | Signal |
| --- | ------ |
| 1 | SDA |
| 2 | SCL |
| 3 | GND |
| 4 | VCC |

### I2C4

| Pin | Signal |
| --- | ------ |
| 1 | SDA |
| 2 | SCL |
| 3 | GND |
| 4 | VCC |

### TELEM2

| Pin | Signal |
| --- | ------ |
| 1 | GND |
| 2 | RX |
| 3 | TX |

### RTK COM2

| Pin | Signal |
| --- | ------ |
| 1 | RX |
| 2 | TX |
| 3 | GND |
| 4 | PPS |

### TELEM1

| Pin | Signal |
| --- | ------ |
| 1 | RX |
| 2 | TX |
| 3 | GND |
| 4 | VCC |

### SBUS

| Pin | Signal |
| --- | ------ |
| 1 | RX |
| 2 | TX |
| 3 | GND |
| 4 | VCC |

## RC Input and SBUS Output

The SBUS connector is connected to USART6. It supports all RC protocols
except PPM. Bi-directional protocols require both the RX and TX signals. See
[RC Systems](https://ardupilot.org/copter/docs/common-rc-systems.html) for
more details on setup. PPM is not brought out to an external connector.

RC input is enabled by default on `SERIAL5`. To use the TX pin for SBUS
output, set `SERIAL5_PROTOCOL` to `15` (`SBus1`) and reboot. The output rate
is configured with `SERVO_SBUS_RATE`. RC input and SBUS output are alternative
modes on `SERIAL5` and cannot be used simultaneously.

### GPS2

| Pin | Signal |
| --- | ------ |
| 1 | RX |
| 2 | TX |
| 3 | GND |
| 4 | VCC |

### TELEM3

| Pin | Signal |
| --- | ------ |
| 1 | RX |
| 2 | TX |
| 3 | GND |
| 4 | VCC |

### PWM

| Pin | Signal |
| --- | ------ |
| 1 | GND |
| 2 | BUZZER |
| 3 | CH8 |
| 4 | CH7 |
| 5 | CH6 |
| 6 | CH5 |
| 7 | CH4 |
| 8 | CH3 |
| 9 | CH2 |
| 10 | CH1 |

The board also has exposed pads labelled M0-P/M0-N through M3-P/M3-N.
These pads are not used by ArduPilot on the Flycore.

### POWER

| Pin | Signal |
| --- | ------ |
| 1 | GND |
| 2 | VCC |
| 3 | CAN_L |
| 4 | CAN_H |

The POWER connector is XT30PW(2+2)-M. VCC is the main power input and
supports 15 V to 28 V. CAN_L and CAN_H connect to CAN2, which is enabled by
default for a DroneCAN battery monitor.

### FAN Power

| Pin | Signal |
| --- | ------ |
| 1 | GND |
| 2 | VCC |

The fan power connector provides 5 V.

### Debug and SWD pads

| Pad | Signal |
| --- | ------ |
| 3V3 | 3.3 V |
| GND | GND |
| SWD | SWDIO |
| SWC | SWCLK |
| DRX | UART8 RX |
| DTX | UART8 TX |

SWD and SWC are used for bootloader flashing and debugging. DRX and DTX
are the UART8 debug serial pins.

## UART Mapping

| Serial | UART | Port | Default protocol |
| ------ | ---- | ---- | ---------------- |
| SERIAL0 | OTG1 | USB | MAVLink2 |
| SERIAL1 | USART2 | TELEM1 | MAVLink2 |
| SERIAL2 | USART3 | TELEM2 | MAVLink2 |
| SERIAL3 | UART4 | GPS1, onboard UM982 | GPS |
| SERIAL4 | UART5 | TELEM3 | MAVLink2 |
| SERIAL5 | USART6 | SBUS | RC input |
| SERIAL6 | UART7 | GPS2 | GPS |
| SERIAL7 | UART8 | Debug pads | Disabled |
| SERIAL8 | OTG2 | Not pinned out | Disabled |

GPS1 is connected internally to the onboard UM982 and is not an external
connector. Set `GPS1_TYPE` to `25` (`UnicoreMovingBaselineNMEA`) when using
the UM982 dual-antenna heading output, or to `24` (`UnicoreNMEA`) when using
position output only. GPS2 is the external GPS connector.

## PWM Outputs

The Flycore supports 8 FMU PWM/Dshot outputs on the PWM connector, in three groups:

- CH1-CH4 Group1
- CH5-CH7 Group2
- CH8 Group3

Outputs in the same timer group must use the same output rate and protocol.
The PWM rail is externally powered; the flight controller only supplies the
PWM signals. Standard PWM, unidirectional DShot, and bidirectional DShot are
supported.

## IMU Heater

The Flycore includes an IMU heater. ArduPilot controls the heater
automatically, with a default target temperature of 45 °C.

## Compass

The Flycore has no onboard compass. An external compass can be connected to
the I2C1 or I2C4 connector.

## Firmware

Firmware for the Flycore is available from the
[ArduPilot firmware server](https://firmware.ardupilot.org/) under the
`flycore` target.

## Loading Firmware

The board uses ArduPilot board ID 1218 and ships with the ArduPilot bootloader.
Firmware can be updated with an `.apj` file using an ArduPilot-compatible
ground station.
