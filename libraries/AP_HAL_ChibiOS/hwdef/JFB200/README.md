# JFB-200 Flight Controller

The JFB-200 flight controller of [JAE](https://www.jae.com/Motion_Sensor_Control/eVTOL/FlightController/)

## Features

- STM32H755 microcontroller
- Three IMUs: ICM45686, ASM330 and IIM42653 SPI IMU
- Two BAROs: two BMP390 SPI barometer
- Two Mags: builtin I2C BMM350 and IST8310 magnetometer
- microSD card slot
- 6 UARTs plus USB (including GPS, RCIN and S.OUT)
- 16 PWM outputs (8 PWM shares GPIO)
- Three I2C and two CAN ports
- Two external Buzzer (Open/Drain and 27V Out)
- external safety Switch
- voltage monitoring for servo rail and Vcc
- two dedicated power input ports for external power bricks
- Ethernet port

## UART Mapping

- SERIAL0 -> USB
- SERIAL1 -> UART7  (Telem1)
- SERIAL2 -> UART5  (Telem2)
- SERIAL3 -> USART3 (Telem3)
- SERIAL4 -> USART1 (GPS1)
- SERIAL5 -> USART2 (GPS2)
- SERIAL6 -> UART8  (RCIN)
- SERIAL7 -> USART6 (S.OUT)
- SERIAL8 -> USB

The Telem1, Telem2 and Telem3 ports have RTS/CTS pins, the other UARTs do not
have RTS/CTS.

## RC Input

RC input is configured on the port marked DSM/SBUS RC. This connector
supports all RC protocols. Two cables are available for this port. To
use software binding of Spektrum satellite receivers you need to use
the Spektrum satellite cable.

## PWM Output

The JFB-200 supports up to 16 PWM outputs.
These are directly attached to the STM32H755 and support all
PWM protocols.

The 16 PWM outputs are in 4 groups:

- PWM  1,  2,  3 and  4 in group1 (TIM1)
- PWM  5,  6,  7 and  8 in group2 (TIM3)
- PWM  9, 10, 11 and 12 in group3 (TIM4)
- PWM 13, 14, 15 and 16 in group4 (TIM8)

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has two dedicated power monitor ports on 8 pin connectors.
The correct battery setting parameters are dependent on the type of
power brick which is connected.
Recomended input voltage is 4.9 to 5.5 volt.

## Compass

The JFB-200 has two builtin compasses BMM350 and IST8310. Due to potential
interference the board is usually used with an external I2C compass as
part of a GPS/Compass combination.

## GPIOs

The 8 PWM ports (PMW9-16) can be used as GPIOs (relays, buttons, RPM etc).

The numbering of the GPIOs for PIN variables in ArduPilot is:

- PWM(9)  50
- PWM(10) 51
- PWM(11) 52
- PWM(12) 53
- PWM(13) 54
- PWM(14) 55
- PWM(15) 56
- PWM(16) 57
- FMU_CAP1 58
- FMU_CAP2 59

## Analog inputs

The JFB-200 has 9 analog inputs

- ADC Pin0  -> not used
- ADC Pin1  -> not used
- ADC Pin2  -> +3.3V Sens
- ADC Pin3  -> not used
- ADC Pin4  -> Battery Voltage 2
- ADC Pin5  -> not used
- ADC Pin6  -> Battery Current Sensor 2
- ADC Pin7  -> not used
- ADC Pin8  -> ADC SPARE 1 (6.6V)
- ADC Pin9  -> not used
- ADC Pin10 -> RSSI voltage monitoring
- ADC Pin11 -> not used
- ADC Pin12 -> not used
- ADC Pin13 -> ADC SPARE 2 (3.3V)
- ADC Pin14 -> SERVORAIL sens
- ADC Pin15 -> 5V sens
- ADC Pin16 -> Battery Voltage
- ADC Pin17 -> not used
- ADC Pin18 -> Battery Current Sensor
- ADC Pin19 -> not used

## I2C Buses

The JFB-200 has 4 I2C interfaces.
I2C 3 is for internal only.

- the internal I2C port  is bus 3 in ArduPilot (I2C3 in hardware)
- the port labelled GPS1 is bus 1 in ArduPilot (I2C1 in hardware)
- the port labelled GPS2 is bus 2 in ArduPilot (I2C2 in hardware)
- the port labelled I2C  is bus 4 in ArduPilot (I2C4 in hardware)

## CAN

The JFB-200 has two independent CAN buses with terminating resistors.

## Dedicated Signal input/output

The JFB-200 has the following dedicated discrete signals

- ARMED status signal output
- Hardware WDT Fail signal output
- HW reset signal input

## Debug

The JFB-200 supports SWD debugging on the debug port

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of *.apj firmware files with any ArduPilot
compatible ground station.
The JFB-200 can be booted into DFU mode using a dedicated adapter.
