# CORVON405V2.1 Flight Controller

The CORVON405V2.1 is a flight controller produced by CORVON.

## Features

- STM32F405 microcontroller
- BMI088 IMU
- SPL06 barometer
- AT7456E OSD
- 9V 3A BEC; 5V 3A BEC
- SDCard
- 6 UARTs
- 10 PWM outputs
- 1 SWD

## Physical

![CORVON F405 V2.1 Front View](CORVON405V2.1_FrontView.jpg)

![CORVON F405 V2.1 Back View](CORVON405V2.1_BackView.jpg)

## UART Mapping

- SERIAL0 -> USB
- SERIAL1 -> UART1 (MAVLink 2,DMA-enabled)
- SERIAL2 -> UART2 (DJI - VTX, TX only is DMA Enabled)
- SERIAL3 -> UART3 (GPS)
- SERIAL4 -> UART4 (MAVLink2,TX only is DMA Enabled)
- SERIAL5 -> UART5 (ESC Telemetry,RX 5 only pinned out)
- SERIAL6 -> UART6 (RCin, RX6 is inverted from SBUS pin, RX only is DMA Enabled)

## RC Input

The default RC input is configured on the UART6_RX input which is inverted from the SBUS pin. Other RC  protocols  should be applied at UART1 which has DMA, and set the [SERIAL1_PROTOCOL](https://ardupilot.org/copter/docs/parameters.html#serial1-protocol-telem1-protocol-selection) ='23' and change [SERIAL6_PROTOCOL](https://ardupilot.org/copter/docs/parameters.html#serial6-protocol-serial6-protocol-selection) to something other than '23'.

- CRSF/ELRS also requires a TX1 connection, in addition to RX1, and automatically provides telemetry.
- FPort requires connection to TX1 and [SERIAL1_OPTIONS](https://ardupilot.org/copter/docs/parameters.html#serial1-options-telem1-options) set to "7". See [FPort receivers](https://ardupilot.org/copter/docs/common-FPort-receivers.html)
- SRXL2 requires a connection to TX2 and automatically provides telemetry. Set [SERIAL1_OPTIONS](https://ardupilot.org/copter/docs/parameters.html#serial1-options-telem1-options) to "4".

## OSD Support

The CORVON405V2.1 supports  internal analog OSD MAX7456. Simultaneous external HD OSD support is preconfigured on SERIAL6. See [MSP OSD](https://ardupilot.org/copter/docs/common-msp-osd-overview-4.2.html) for more info.

## VTX Support

Both Analog and HD VTX connectors are provided. Pin 1 of the connector is 9v so be careful not to connect this to a peripheral requiring 5v.

## PWM Output

The CORVON405V2.1 supports up to 10 PWM outputs.

Channels 1-8 support DShot. Channels 1-4 support bi-directional DShot.

PWM outputs are grouped and every  group must use the same output protocol:

- 1,2,5,6 are group1
- 3, 4 are group 2
- 7,8 are group 3
- 9,10 are in group 4

## Battery Monitoring

The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input.
The voltage sensor can handle up to 6S LiPo batteries.

The default battery parameters are:

- BATT_MONITOR = 4
- BATT_VOLT_PIN = 10
- BATT_CURR_PIN =1 1
- BATT_VOLT_MULT = 21.2
- BATT_AMP_PERVLT = 40.2

## Compass

The CORVON405V2.1 does not have a built-in compass, but you can attach an external compass using I2C on the SDA and SCL connector.

## Ports Connector

![CORVON405V2.1 Ports Connection](CORVON405V2.1_PortsConnection.jpg)

## Loading Firmware

Firmware for these boards can be found at the [ArduPilot firmware server](https://firmware.ardupilot.org) in sub-folders labeled CORVON405V2.1.

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "with_bl.hex" firmware, using your favorite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the "*.apj" firmware files.
