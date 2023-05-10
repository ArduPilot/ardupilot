# JFB-100 Flight Controller

The JFB-100 flight controller is sold by [JAE](https://www.jae.com/files/user/motion-sense-control/catalog/jfb-100-ja.pdf)

## Features

 - STM32F765 microcontroller
 - Two IMUs: ICM20602 and SCHA63T
 - MS5611 SPI barometer
 - builtin I2C IST8310 magnetometer
 - microSD card slot
 - 6 UARTs plus USB
 - 8 PWM outputs
 - Four I2C and two CAN ports
 - External Buzzer
 - external safety Switch
 - voltage monitoring for servo rail and Vcc
 - two dedicated power input ports for external power bricks

## UART Mapping

 - SERIAL0 -> USB
 - SERIAL1 -> UART2 (Telem1)
 - SERIAL2 -> UART3 (Telem2)
 - SERIAL3 -> UART1 (GPS)
 - SERIAL4 -> UART4 (GPS2, marked UART/I2CB)
 - SERIAL5 -> UART6 (RC)
 - SERIAL6 -> UART7 (spare, debug)

The Telem1 and Telem2 ports have RTS/CTS pins, the other UARTs do not
have RTS/CTS.

The UART7 connector is labelled debug, but is available as a general
purpose UART with ArduPilot.

## RC Input
 
RC input is configured on the port marked DSM/SBUS RC. This connector
supports all RC protocols. Two cables are available for this port. To
use software binding of Spektrum satellite receivers you need to use
the Spektrum satellite cable.

## PWM Output

The JFB-100 supports up to 8 PWM outputs. 
These are directly attached to the STM32F765 and support all
PWM protocols. The first 4 of the PWM outputs support DShot.

The 8 PWM outputs are in 3 groups:

 - PWM 1, 2, 3 and 4 in group1
 - PWM 5 and 6 in group2
 - PWM 7 and 8 in group3

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has two dedicated power monitor ports on 8 pin
connectors. The correct battery setting parameters are dependent on
the type of power brick which is connected.

## Compass

The JFB-100 has a builtin IST8310 compass. Due to potential
interference the board is usually used with an external I2C compass as
part of a GPS/Compass combination.

## GPIOs

The 6 PWM ports can be used as GPIOs (relays, buttons, RPM etc). To
use them you need to limit the number of these pins that is used for
PWM by setting the BRD_PWM_COUNT to a number less than 6. For example
if you set BRD_PWM_COUNT to 4 then PWM5 and PWM6 will be available for
use as GPIOs.

The numbering of the GPIOs for PIN variables in ArduPilot is:

 - AUX1 50
 - AUX2 51
 - AUX3 52
 - AUX4 53
 - AUX5 54
 - AUX6 55

## Analog inputs

The JFB-100 has 7 analog inputs

 - ADC Pin0 -> Battery Voltage
 - ADC Pin1 -> Battery Current Sensor
 - ADC Pin2 -> Battery Voltage 2
 - ADC Pin3 -> Battery Current Sensor 2
 - ADC Pin4 -> ADC port pin 2
 - ADC Pin14 -> ADC port pin 3
 - ADC Pin10 -> ADC 5V Sense
 - ADC Pin11 -> ADC 3.3V Sense
 - ADC Pin103 -> RSSI voltage monitoring

## I2C Buses

 - the internal I2C port is bus 0 in ArduPilot (I2C3 in hardware)
 - the port labelled I2CA is bus 3 in ArduPilot (I2C4 in hardware)
 - the port labelled I2CB is bus 2 in ArduPilot (I2c2 in hardware)
 - the port labelled GPS is bus 1 in ArduPilot (I2c1 in hardware)


## CAN

The JFB-100 has two independent CAN buses, with the following pinouts.

## Debug

The JFB-100 supports SWD debugging on the debug port


## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of *.apj firmware files with any ArduPilot
compatible ground station.
