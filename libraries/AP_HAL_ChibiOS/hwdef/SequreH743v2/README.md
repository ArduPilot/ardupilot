# Sequre H743 Flight Controller

The Sequre H743 is a flight controller based on the STM32H743 MCU.

## Features

* MCU - STM32H743 32-bit processor running at 480 MHz
* IMU - ICM42688
* Barometer - DPS310
* OSD - AT7456E
* microSD card slot
* 7x UARTs
* CAN support
* 13x PWM Outputs (12 Motor Output, 1 LED)
* Battery input voltage: 2S-6S
* BEC 3.3V 0.5A
* BEC 5V 3A
* BEC 9V 3A for video, GPIO controlled
* Selectable 5V or VBAT pad, for analog VTX, GPIO controlled
* Dual switchable camera inputs

## UART Mapping

* SERIAL0 -> USB (MAVLink2)
* SERIAL1 -> UART1 (RC Input or SBUS)
* SERIAL2 -> UART2 (GPS)
* SERIAL3 -> UART3 (Telem1 / DisplayPort)
* SERIAL4 -> UART4 (Telem2)
* SERIAL5 -> UART5
* SERIAL6 -> UART6 (RC Input / ELRS / CRSF)
* SERIAL7 -> UART7 (ESC Telemetry or debug)

## RC Input

Default RC input is via UART1 or UART6. Supports all serial RC protocols including SBUS, FPort, CRSF, ELRS.

## FrSky Telemetry

Can be enabled on any free UART TX pin:

* SERIALx\_PROTOCOL 10
* SERIALx\_OPTIONS 7

## OSD Support

The Sequre H743 supports OSD using OSD\_TYPE 1 (MAX7456 driver).

## PWM Output

The Sequre H743 supports up to 13 PWM or DShot outputs:

* PWM 1-12: Motor outputs
* PWM 13: LED / Aux output

### PWM Groups

* PWM 1-2 in group1
* PWM 3-4 in group2
* PWM 5-6 in group3
* PWM 7-8 in group4
* PWM 9-10 in group5
* PWM 11-12 in group6
* PWM 13 in group7

Note: Channels in the same group must share timing/protocol (PWM or DShot).

## Battery Monitoring

Primary battery monitoring:

* BATT\_MONITOR 4
* BATT\_VOLT\_PIN 13
* BATT\_CURR\_PIN 12
* BATT\_VOLT\_MULT 11.0
* BATT\_AMP\_PERVLT 40

## Analog RSSI and Airspeed Inputs

* RSSI\_PIN 15
* ARSPD\_PIN 4 (if used)

## Compass

No onboard compass. Use external I2C compass via SDA/SCL.

## VTX Power Control

* GPIO 81: Switchable VTX 5V/VBAT power (RELAY2)
* GPIO 83: Controls 9V regulator (RELAY4)

## Camera Switching

* GPIO 82: Camera input switching (RELAY3)

## Firmware Loading

Use DFU (USB + BOOT button) to flash `with_bl.hex` for first-time installation.
For updates, use `.apj` files via ground control software.
