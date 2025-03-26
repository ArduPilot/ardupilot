# BROTHER HOBBY H743 Flight Controller

The BROTHERHOBBYH743 is a flight controller produced by [BROTHERHOBBY](https://www.brotherhobbystore.com/).

## Features

 - STM32H743 microcontroller
 - ICM42688P IMU
 - SPL06 barometer
 - AT7456E OSD
 - 12V 3A BEC; 5V 3A BEC
 - SDCard
 - 7 UARTs
 - 13 PWM outputs

## UART Mapping

 - SERIAL0 -> USB
 - SERIAL1 -> UART1 (ESC Telemetry)
 - SERIAL2 -> UART2 (RX/SBUS, DMA-enabled)
 - SERIAL3 -> UART3 (Spare, DMA-enabled)
 - SERIAL4 -> UART4 (Spare)
 - SERIAL6 -> UART6 (DJI, DMA-enabled)
 - SERIAL7 -> UART7 (GPS, DMA-enabled)
 - SERIAL8 -> UART8 (Spare)

## RC Input

The default RC input is configured on the UART2_RX inverted from the SBUS pin. Other RC  protocols  should be applied at other UART port such as UART3 or UART8, and set the protocol to receive RC data: `SERIALn_PROTOCOL=23` and change SERIAL2 _Protocol to something other than '23'

## FrSky Telemetry
 
FrSky Telemetry can be supported using the T1 pin (UART1 transmit). You need to set the following parameters to enable support for FrSky S.PORT
 
  - SERIAL1_PROTOCOL 10
  - SERIAL1_OPTIONS 7
  
## OSD Support

The BROTHERHOBBYH743 supports OSD using OSD_TYPE 1 (MAX7456 driver).

## VTX Support

The SH1.0-6P connector supports a standard DJI HD VTX connection. Pin 1 of the connector is 12v so be careful not to connect incorrecly.

## PWM Output

The BROTHERHOBBYH743 supports up to 13 PWM outputs.

Channels 1-8 support bi-directional DShot.

PWM outputs are grouped and every group must use the same output protocol:
 - PWM 1-2   in group1
 - PWM 3-4   in group2
 - PWM 5-6   in group3
 - PWM 7-10  in group4
 - PWM 11-12 in group5 - servo 1/2
 - PWM 13    in group6 - LED strip

## Battery Monitoring

The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input.
The voltage sensor can handle up to 6S LiPo batteries.

The default battery parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 10
 - BATT_CURR_PIN 11
 - BATT_VOLT_MULT 11
 - BATT_CURR_SCALE 40.2

## Compass

The BROTHERHOBBYH743 does not have a built-in compass, but you can attach an external compass using I2C on the SDA and SCL connector.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "with_bl.hex" firmware, using your favorite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the "*.apj" firmware files.
