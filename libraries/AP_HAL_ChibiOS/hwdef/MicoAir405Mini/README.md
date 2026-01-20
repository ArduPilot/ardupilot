# MicoAir405Mini Flight Controller

The MicoAir405Mini is a flight controller designed and produced by [MicoAir Tech.](http://micoair.com/).

## Features

 - STM32F405 microcontroller
 - BMI270 IMU
 - DPS310 barometer
 - AT7456E OSD
 - 9V 2.5A BEC; 5V 2.5A BEC
 - SDCard
 - 6 UARTs
 - 9 PWM outputs

## Physical

![MicoAir F405 Mini V2.1 Front View](MicoAir405Mini_FrontView.jpg)

![MicoAir F405 Mini V2.1 Back View](MicoAir405Mini_BackView.jpg)



## UART Mapping

 - SERIAL0 -> USB
 - SERIAL1 -> UART1 (MAVLink2, DMA-enabled)
 - SERIAL2 -> UART2 (DisplayPort, TX only is DMA Enabled)
 - SERIAL3 -> UART3 (GPS)
 - SERIAL4 -> UART4 (MAVLink2, TX only is DMA Enabled)
 - SERIAL5 -> UART5 (ESC Telemetry)
 - SERIAL6 -> UART6 (RX6 is inverted from SBUS pin, RX only is DMA Enabled)

## RC Input

The default RC input is configured as SBUS via the SBUS pin. Other RC  protocols  should be applied at a UART port such as UART1 or UART4, and set the protocol to receive RC data: `SERIALn_PROTOCOL=23` and change SERIAL6 _Protocol to something other than '23'

## OSD Support

The MicoAir405Mini supports onboard OSD using OSD_TYPE 1 (MAX7456 driver).

## VTX Support

The SH1.0-6P connector supports a DJI O3 Air Unit connection. Pin 1 of the connector is 9v so be careful not to connect this to a peripheral requiring 5v.

## PWM Output

The MicoAir405Mini supports up to 9 PWM outputs.

Channels 1-8 support DShot.

Channels 1-4 support bi-directional DShot.

PWM outputs are grouped and every group must use the same output protocol:

3, 4 are Group 1;

1, 2, 5, 6 are Group 2;

7, 8 are Group 3;

9 is in Group 4;

## Battery Monitoring

The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input.
The voltage sensor can handle up to 6S LiPo batteries.

The default battery parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 10
 - BATT_CURR_PIN 11
 - BATT_VOLT_MULT 21.2
 - BATT_AMP_PERVLT 40.2

## Compass

The MicoAir405Mini does not have a built-in compass, but you can attach an external compass using I2C on the SDA and SCL connector.

## Mechanical

 - Mounting: 20 x 20mm, Φ3.5mm
 - Dimensions: 30 x 30 x 8 mm
 - Weight: 6g

## Ports Connector

![MicoAir F405 Mini V2.1 Ports Connection](MicoAir405Mini_PortsConnection.jpg)

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "with_bl.hex" firmware, using your favorite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the "*.apj" firmware files.
