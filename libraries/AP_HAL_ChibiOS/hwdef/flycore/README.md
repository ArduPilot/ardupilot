# AMOV Flycore Flight Controller

The AMOV Flycore is an STM32H743-based flight controller manufactured by
[AMOV](https://amovlab.com/). It includes an onboard UM982 GNSS receiver,
dual IMUs, an onboard barometer, 10 PWM outputs, dual CAN, three telemetry
ports, and a standard SD card slot.

![AMOV Flycore](amovlab_flycore_product.jpg "AMOV Flycore")

## Where to Buy

For purchasing information, contact
[shudajun@amovauto.com](mailto:shudajun@amovauto.com).

## Specifications

- Processor
  - STM32H743, Arm Cortex-M7 at 480 MHz
  - 2 MB flash and 1 MB RAM
  - No I/O coprocessor
- Sensors
  - Bosch BMI088 IMU
  - InvenSense ICM-42688P IMU
  - MS5611 barometer
  - UM982 GNSS receiver connected internally to GPS1
  - No onboard compass
- Interfaces
  - 10 FMU PWM outputs
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
  - 3S to 6S LiPo voltage sensing hardware
  - 1 A to 60 A current sensing hardware
- Mechanical
  - 120 mm x 55 mm x 26.3 mm
  - 60 g
  - 50 mm x 115 mm mounting pattern with 2.5 mm holes

## Pinout

![AMOV Flycore pinout](amovlab_flycore_pinout.jpg "AMOV Flycore pinout")

The external peripheral signal connectors use JST-GH connectors with
1.25 mm pitch. The POWER connector is XT30. USB, the SD card slot, RTK
antenna connectors, and the exposed buzzer, SWD, and debug pads use the
interfaces shown in the pinout image.

Pin 1 starts at the right side of each connector when viewed as shown below.

![AMOV Flycore pin 1 orientation](amovlab_flycore_pin1_orientation.png "Connector pin 1 orientation")

### CAN1

| Pin | Signal |
| --- | ------ |
| 1 | CAN_L |
| 2 | CAN_H |
| 3 | GND |
| 4 | VCC |

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

### I2C1 and I2C4

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

### PWM

| Pin | Signal |
| --- | ------ |
| 1 | GND |
| 2 | BUZZER |
| 3 | FMU_CH8 |
| 4 | FMU_CH7 |
| 5 | FMU_CH6 |
| 6 | FMU_CH5 |
| 7 | FMU_CH4 |
| 8 | FMU_CH3 |
| 9 | FMU_CH2 |
| 10 | FMU_CH1 |

## UART Mapping

| Serial | UART | Port | Default protocol |
| ------ | ---- | ---- | ---------------- |
| SERIAL0 | OTG1 | USB | MAVLink2 |
| SERIAL1 | USART2 | TELEM1 | MAVLink2 |
| SERIAL2 | USART3 | TELEM2 | MAVLink2 |
| SERIAL3 | UART4 | GPS1, onboard UM982 | GPS |
| SERIAL4 | UART5 | TELEM3 | MAVLink2 |
| SERIAL5 | USART6 | RC | RC input |
| SERIAL6 | UART7 | GPS2 | GPS |
| SERIAL7 | UART8 | Debug pads | Disabled |

GPS1 is connected internally to the onboard UM982 and is not an external
connector. GPS2 is the external GPS connector.

## RC Input

The RC connector is connected to USART6 and supports serial RC protocols
such as SBUS, DSM/DSMX, CRSF, and GHST. PPM input is available on PC8.
Bi-directional protocols require both the RX and TX signals.

## PWM Outputs

The Flycore supports 10 FMU PWM outputs in three timer groups:

- Outputs 1 to 4 use TIM1.
- Outputs 5 to 7 use TIM3.
- Outputs 8 to 10 use TIM4.

Outputs in the same timer group must use the same output rate and protocol.
Outputs 1 to 8 are on the PWM connector. Outputs 9 and 10 are available on
the separate pads shown in the pinout image.
The PWM rail is externally powered; the flight controller only supplies the
PWM signals.

## GPIOs

| Function | GPIO number |
| -------- | ----------- |
| FMU_CH1 to FMU_CH10 | 50 to 59 |
| Buzzer | 77 |
| IMU heater | 80 |
| Hardware version drive | 82 |
| SBUS output enable | 83 |
| Red LED | 90 |
| Blue LED | 92 |

## Analog Inputs

The board routes RSSI, hardware identification, 5 V sensing, battery voltage,
and battery current signals to ADC pins. ArduPilot currently enables RSSI,
hardware identification, and 5 V sensing. Battery monitoring is not enabled
until the voltage divider and current sensor calibration are verified.

## Compass

The Flycore has no onboard compass. An external compass can be connected to
the I2C1 or I2C4 connector. The GPS1 and GPS2 serial interfaces do not include
compass I2C signals.

## Firmware

Firmware for the Flycore is available from the
[ArduPilot firmware server](https://firmware.ardupilot.org/) under the
`flycore` target.

To build the firmware from source:

```sh
./waf configure --board flycore
./waf copter
```

## Loading Firmware

The board uses ArduPilot board ID 1218. Initial firmware can be loaded using
the Flycore bootloader image. After the bootloader is installed, firmware can
be updated with an `.apj` file using an ArduPilot-compatible ground station.
