# Pixhawk4-Mini Flight Controller

The Pixhawk4-Mini flight controller is sold by [Holybro](http://www.holybro.com/product/64)

## Features

 - STM32F765 microcontroller
 - Two IMUs: ICM20689 and BMI055
 - MS5611 SPI barometer
 - builtin SPI IST8310 magnetometer
 - microSD card slot
 - 3 UARTs plus USB
 - 8 PWM outputs
 - Two I2C and one CAN ports
 - External Buzzer
 - external safety Switch
 - dedicated power input port for external power bricks

## Pinout

![Pixhawk4 Mini Board](PH4-mini-pinout.jpg "Pixhawk4 Mini")

## UART Mapping

 - SERIAL0 -> USB
 - SERIAL1 -> UART2 (Telem1)
 - SERIAL2 -> UART3 (Telem2)
 - SERIAL3 -> UART1 (GPS)
 - SERIAL6 -> UART7 (debug)

The Telem1 and Telem2 ports have RTS/CTS pins, the other UARTs do not
have RTS/CTS.

The UART7 connector is inside the case and labelled as debug, but is
available as a general purpose UART with ArduPilot.

## RC Input
 
RC input is configured on the port marked RC IN. This connector
supports all RC protocols. Two cables are available for this port. To
use software binding of Spektrum satellite receivers you need to use
the Spektrum satellite cable.

## PWM Output

The Pixhawk4-Mini supports up to 8 PWM outputs. All 8 outputs support all
normal PWM output formats. Only the first 6 outputs support DShot.

The 8 auxillary PWM outputs are in 3 groups:

 - PWM 1, 2, 3 and 4 in group1
 - PWM 5 and 6 in group2
 - PWM 7 and 8 in group3

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has two dedicated power monitor ports on 6 pin
connectors. The correct battery setting parameters are dependent on
the type of power brick which is connected.

## Compass

The Pixhawk4-Mini has a builtin IST8310 compass. Due to potential
interference the board is usually used with an external I2C compass as
part of a GPS/Compass combination.

## GPIOs

The 8 PWM ports can be used as GPIOs (relays, buttons, RPM etc). To
use them you need to limit the number of these pins that is used for
PWM by setting the BRD_PWM_COUNT to a number less than 8. For example
if you set BRD_PWM_COUNT to 6 then PWM6 and PWM7 will be available for
use as GPIOs.

The numbering of the GPIOs for PIN variables in ArduPilot is:

 - PWM1 50
 - PWM2 51
 - PWM3 52
 - PWM4 53
 - PWM5 54
 - PWM6 55
 - PWM7 56
 - PWM8 57

In addition there are 4 pins on the servo rail marked CAP1 to
CAP4. The first 3 of these are available as GPIOs in ArduPilot using
the following GPIO numbers:

 - CAP1 58
 - CAP2 59
 - CAP3 60

## Analog inputs

The Pixhawk4-Mini has 4 analog inputs

 - ADC Pin0 -> Battery Voltage
 - ADC Pin1 -> Battery Current Sensor
 - ADC Pin10 -> ADC 5V Sense
 - ADC Pin11 -> ADC 3.3V Sense

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of *.apj firmware files with any ArduPilot
compatible ground station.

## Acknowledgements

Thanks to
[PX4](https://docs.px4.io/en/flight_controller/pixhawk4_mini.html) for
images used under the [CC-BY 4.0 license](https://creativecommons.org/licenses/by/4.0/)
