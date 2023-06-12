# A6 Flight Controller

The A6 flight controller is manufactured and sold by [YJUAV](http://www.yjuav.net).

The full schematics of the board are available here:

[A6 Flight Controller](http://doc.yjuav.net/index.php/自动驾驶仪/a6飞控/)

![A6_YJUAV Board](A6_YJUAV.jpg "A6_YJUAV")
## Features

 - STM32H743 microcontroller
 - Three IMUs: ICM42688, ICM42688 and IIM42652
 - Internal SPI RM3100 magnetometer
 - Internal DPS310 SPI barometer
 - Internal vibration isolation for IMUs
 - Internal RGB LED
 - microSD card slot port
 - 2 power ports(CAN and Analog)
 - 6 UARTs and USB ports
 - 3 I2C and 3 CAN ports
 - 14 PWM output ports
 - Safety switch port
 - External SPI port
 - Buzzer port

## UART Mapping

 - SERIAL0 -> USB(OTG1)
 - SERIAL1 -> USART1(Telem1)
 - SERIAL2 -> USART2 (Telem2)
 - SERIAL3 -> USART3 (GPS)
 - SERIAL4 -> UART5 (GPS2)
 - SERIAL5 -> UART6 (SBUS)
 - SERIAL6 -> UART7 (spare)
 - SERIAL7 -> USB2(OTG2)

## RC Input

The remote control signal should be connected to the “RC IN” pin, at one side of the servo channels.

This signal pin supports two types of remote control signal inputs, SBUS and PPM signals.

## PWM Output

The A6 supports up to 14 PWM outputs.support all PWM protocols as well as DShot. All 14 PWM outputs have GND on the top row, 5V on the middle row and signal on the bottom row.

The 14 PWM outputs are in 4 groups:

 - PWM 1, 2, 3 and 4 in group1
 - PWM 5, 6, 7 and 8 in group2
 - PWM 9, 10, 11 and 12 in group3
 - PWM 13 and 14 group4

Channels within the same group need to use the same output rate. If any channel in a group uses DShot, then all channels in that group need to use DShot.

## Battery Monitoring

The A6 flight controller has two six-pin power connectors, supporting CAN interface power supply and analog interface power supply.

## Compass

The A6 flight controller built-in industrial-grade electronic compass chip RM3100.

## GPIOs

All 14 PWM channels can be used for GPIO functions (relays, buttons, RPM etc).

The pin numbers for these PWM channels in ArduPilot are shown below:

| PWM Channels | Pin  | PWM Channels | Pin  |
| ------------ | ---- | ------------ | ---- |
| PWM1         | 50   | PWM8         | 57   |
| PWM2         | 51   | PWM9         | 58   |
| PWM3         | 52   | PWM10        | 59   |
| PWM4         | 53   | PWM11        | 60   |
| PWM5         | 54   | PWM12        | 61   |
| PWM6         | 55   | PWM13        | 62   |
| PWM7         | 56   | PWM14        | 63   |

## Analog inputs

The A6 flight controller has 5 analog inputs

 - ADC Pin10 -> Battery Current 
 - ADC Pin11  -> Battery Voltage 
 - ADC Pin4   -> ADC 3V3 Sense
 - ADC Pin8  ->  ADC 5V Sense
 - ADCPin18  ->  RSSI voltage monitoring

## Loading Firmware

The A6 flight controller comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of *.apj firmware files with any ArduPilot compatible ground station.

