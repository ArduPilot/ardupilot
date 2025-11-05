# CM4PILOT Integrated Flight Controller

The CM4PILOT is a low-cost and compact flight controller which integrated a Raspberry Pi CM4 in the cockpit.

<div align=left><img width="600"  src=CM4pilot_inshell.jpg/></div>
<div align=left><img width="500"  src=CM4Pilot_structure.jpg/></div>


## Features

 - Raspberry Pi CM4 + Ardupilot, Companion Computers in cockpit structure
 - Small Footprint and Lightweight, 58mm X 50mm X 18mm，26g(without shell)
 - Broadcom BCM2711, quad-core Cortex-A72 (ARM v8) 64-bit SoC @ 1.5GHz
 - STM32F405 microcontroller
 - IMU: BMI088
 - Mag: LIS3MDLTR
 - Baro: BMP280
 - RTC: PCF85063
 - 2 2-lane MIPI CSI camera ports
 - 2 microSD card slot port
 - 1 power ports(Analog)
 - 6 UARTs and USB ports for FMU
 - 2 UARTs and 4 USB2.0 and 1 OTG for CM4
 - 1 I2C port
 - 1 CAN port
 - 1 SBUS input and 8 PWM output (all support DShot)
 - External SPI port
 - Buzzer on board 
 - RBG LED on board
 - 128M flash on board for logging

## UART Mapping

 - SERIAL0 -> USB(OTG1)
 - SERIAL1 -> USART1(Telem1)(DMA capable)
 - SERIAL2 -> USART3 (CM4)(DMA capable)
 - SERIAL3 -> UART4 (GPS)(DMA capable)
 - SERIAL4 -> UART6 (GPS2)(DMA capable)
 - SERIAL5 -> USART2 (SBUS)(RC input, no DMA capable)

## RC Input

RC input is configured on the SBUS pin (UART2_RX). It supports all RC protocols except serial protocols

## PWM Output

The CM4PILOT supports up to 8 PWM outputs. All outputs support DShot (No BDshot).
The PWM is in 4 groups:

 - PWM 1~4 in group1
 - PWM 5,6 in group2
 - PWM 7,8 in group3
 - Buzzer on board in group4

## GPIOs

All 8 PWM channels can be used for GPIO functions.
The pin numbers for these PWM channels in ArduPilot are shown below:

| PWM Channels | Pin  | PWM Channels | Pin  |
| ------------ | ---- | ------------ | ---- |
| PWM1         | 50   | PWM8         | 57   |
| PWM2         | 51   | 
| PWM3         | 52   | 
| PWM4         | 53   | 
| PWM5         | 54   | 
| PWM6         | 55   | 
| PWM7         | 56   | 

## Battery Monitoring

The correct battery setting parameters are set by default and are:
 
 - BATT_MONITOR 4
 - BATT_VOLT_PIN 11
 - BATT_CURR_PIN 12
 - BATT_VOLT_SCALE 10.1
 - BATT_AMP_PERVLT 17.0

## Compass

The CM4PILOT has one built-in compass LIS3MDLTR, you can also attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Firmware for these boards can be found at https://firmware.ardupilot.org in sub-folders labeled “ACNS-CM4PILOT”.

Initial firmware load can be done with DFU by plugging in USB with the
boot button pressed. Then you should load the "xxx_bl.hex"
firmware, using your favorite DFU loading tool.

Subsequently, you can update the firmware with Mission Planner.

## Pinout and Size

<div align=left><img width="600"  src=CM4Pilot_Pinout.jpg/></div>
<div align=left><img width="500"  src=CM4pilot_size.jpg/></div>
