# CORVON743V1 Flight Controller

The CORVON743V1 is a flight controller designed and produced by [CORVON]

## Features

- STM32H743 microcontroller
- BMI088/BMI270 dual IMUs
- DPS310 barometer
- IST8310 magnetometer
- AT7456E OSD
- 9V 3A BEC; 5V 3A BEC
- MicroSD Card Slot
- 7 UARTs
- 10 PWM outputs
- 1 CAN
- 1 I2C
- 1 SWD

## Physical

![CORVON743V1 Front View](CORVON743V1_FrontView.jpg)

![CORVON743V1 Back View](CORVON743V1_BackView.jpg)

## UART Mapping

- SERIAL0 -> USB
- SERIAL1 -> UART1 (MAVLink2, DMA-enabled)
- SERIAL2 -> UART2 (DisplayPort, DMA-enabled)
- SERIAL3 -> UART3 (GPS, DMA-enabled)
- SERIAL4 -> UART4 (MAVLink2, DMA-enabled)
- SERIAL5 -> UART6 (RCIN, DMA-enabled)
- SERIAL6 -> UART7 (RX only, ESC Telemetry, DMA-enabled)
- SERIAL7 -> UART8 (User, DMA-enabled)

## RC Input

The default RC input is configured on the UART6. The SBUS pin is inverted and connected to RX6. Non SBUS, single wire serial inputs can be directly tied to RX6 if SBUS pin is left unconnected. RC could  be applied instead to a different UART port such as UART1, UART4 or UART8, and set the protocol to receive RC data `SERIALn_PROTOCOL` = 23 and change [SERIAL5_PROTOCOL](https://ardupilot.org/copter/docs/parameters.html#serial5-protocol-serial5-protocol-selection) to something other than '23'. For rc protocols other than unidirectional, the TX6 pin will need to be used:

- [SERIAL5_PROTOCOL](https://ardupilot.org/copter/docs/parameters.html#serial5-protocol-serial5-protocol-selection) should be set to "23".
- FPort would require [SERIAL5_OPTIONS](https://ardupilot.org/copter/docs/parameters.html#serial5-options-serial5-options) be set to "15".
- CRSF would require [SERIAL5_OPTIONS](https://ardupilot.org/copter/docs/parameters.html#serial5-options-serial5-options) be set to "0".
- SRXL2 would require [SERIAL5_OPTIONS](https://ardupilot.org/copter/docs/parameters.html#serial5-options-serial5-options) be set to "4" and connects only the TX pin.

## OSD Support

The CORVON743V1 supports onboard OSD using OSD_TYPE 1 (MAX7456 driver). Simultaneously, DisplayPort OSD is available on the HD VTX connector, See below.

## VTX Support

The SH1.0-6P connector supports a DJI Air Unit / HD VTX connection. Protocol defaults to DisplayPort. Pin 1 of the connector is 9v so be careful not to connect this to a peripheral that can not tolerate this voltage.

## PWM Output

The CORVON743V1 supports up to 10 PWM outputs. All outputs support PWM, DShot, and Bi-Directional DShot.

PWM outputs are grouped and every group must use the same output protocol:

1, 2, 3, 4 are Group 1;

5, 6 are Group 2;

7, 8, 9, 10 are Group 3;

## Battery Monitoring

The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input.
The voltage sensor can handle up to 6S LiPo batteries.

The default battery parameters are:

- [BATT_MONITOR](https://ardupilot.org/copter/docs/parameters.html#batt-monitor-battery-monitoring) = 4
- [BATT_VOLT_PIN](https://ardupilot.org/copter/docs/parameters.html#batt-volt-pin-ap-battmonitor-analog-battery-voltage-sensing-pin) = 10
- [BATT_CURR_PIN](https://ardupilot.org/copter/docs/parameters.html#batt-curr-pin-ap-battmonitor-analog-battery-current-sensing-pin) = 11 (CURR pin)
- [BATT_VOLT_MULT](https://ardupilot.org/copter/docs/parameters.html#batt-volt-mult-ap-battmonitor-analog-voltage-multiplier) = 21.12
- [BATT_AMP_PERVLT](https://ardupilot.org/copter/docs/parameters.html#batt-amp-pervlt-ap-battmonitor-analog-amps-per-volt) = 40.2

## Compass

The CORVON743V1 has a built-in compass sensor (IST8310), and you can also attach an external compass using I2C on the SDA and SCL connector.

## Mechanical

- Mounting: 30.5 x 30.5mm, Φ4mm
- Dimensions: 36 x 36 x 8 mm
- Weight: 9g

## Ports Connector

![CORVON743V1 Ports Connection](CORVON743V1_PortsConnection.jpg)

## Loading Firmware

Firmware for these boards can be found [here](https://firmware.ardupilot.org) in sub-folders labeled "CORVON743V_1".

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "with_bl.hex" firmware, using your favorite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the "*.apj" firmware files.
