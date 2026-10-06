# Intel Aero Flight Controller

The flight controller of the Intel Aero Ready to Fly (RTF) drone. The RTF
also has a Linux compute board, which talks to the flight controller over
a serial link, and an FPGA, which routes serial ports and measures battery
voltage.

This firmware is for the FPGA image that makes the flight controller fly
the vehicle (`aero-rtf.jam`), with FPGA version 0xC0. Later FPGA versions
move the GPS to a different UART.

Firmware for this board is not built on the ArduPilot firmware server.
Build it with:

```sh
./waf configure --board aerofc-v1
./waf copter
```

## Features

- STM32F429 microcontroller (only 1MB of flash is usable with the PX4
  bootloader)
- MPU6500 IMU
- MS5607 barometer
- IST8310 compass, in the GPS module, on I2C1
- Four TAP ESCs, driven over a serial port
- Battery voltage from the FPGA
- No USB, no SD card, no PWM outputs, no safety switch

## UART Mapping

- SERIAL0 -> USART2: link to the compute board, MAVLink at 460800
- SERIAL1 -> UART5: telemetry
- SERIAL3 -> USART3: GPS
- SERIAL4 -> UART4: RC input
- SERIAL5 -> USART1: TAP ESCs
- SERIAL6 -> USART6: debug

## RC Input

RC input is on UART4, which has no inverter. The unit this was tested on
has a Spektrum serial receiver there, fitted after manufacture.

## Motor Output

The motors are driven by four TAP ESCs on USART1 (SERIAL5_PROTOCOL 51),
using the outputs for Motor1 to Motor4 (SERVO1_FUNCTION to
SERVO4_FUNCTION 33 to 36).

## Battery Monitoring

Battery voltage is read from an ADC in the FPGA (BATT_MONITOR 33). There
is no current sensor. Calibrate the voltage with BATT_VOLT_MULT.

## Logging

There is no SD card, so logging is off by default. Logs can be sent over
MAVLink (LOG_BACKEND_TYPE 2) if something receives them, for example
MAVProxy's dataflash_logger module.

## Loading Firmware

The board keeps the PX4 bootloader it ships with, which is reached
through the compute board. Copy the .apj file to the compute board and
run:

```sh
/usr/sbin/aerofc-update.sh arducopter.apj
```
