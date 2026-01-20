# ACNS-F405AIO Integrated Flight Controller

The ACNS-F405AIO is a low-cost compact flight controller for multi-rotor UAVs which integrated 4 BLheli_s ESCs on board.

## Features

 - STM32F405RET microcontroller
 - IMU: BMI160, ICM42688
 - Mag: LIS3MDLTR
 - Baro: BMP280
 - 1 microSD card slot port
 - 6 UARTs and USB ports
 - 1 I2C port
 - 1 CAN port
 - 1 SBUS input and 8 PWM output (4 Internal ESCs,4 External PWM ports )
 - 1 External SPI port
 - 1 RGB LED on board
 - 128M flash on board
 - 4 BLheli_s ESCs, 3-4s, 30A, The motor order matches the Arducotper X frame type config
 - Small footprint and lightweight, 39mm X 39mm X 10mm, 9g(without shell)
   
## UART Mapping

 - SERIAL0 -> USB (OTG1)
 - SERIAL1 -> USART1(Telem1) (DMA)
 - SERIAL2 -> USART3(Telem2) (DMA)
 - SERIAL3 -> UART4(GPS) (DMA)
 - SERIAL4 -> UART6(GPS2) (DMA)
 - SERIAL5 -> USART2(SBUS) (RC input, NO DMA)

## RC Input

RC input is configured on the SBUS pin (UART2_RX). It supports all RC protocols except serial protocols

## PWM Output

The ACNS-F405AIO supports up to 8  PWM outputs. All outputs support DShot (No BDshot).
The PWM is in 3 groups:

 - PWM 1~4 in group1 (4 Motors)
 - PWM 5,6 in group2 (External PWM)
 - PWM 7,8 in group3 (External PWM)

## GPIOs

4 External PWM channels can be used for GPIO functions.
The pin numbers for these PWM channels in ArduPilot are shown below:

| PWM Channels | Pin  |
| ------------ | ---- | 
| PWM5         | 54   |
| PWM6         | 55   | 
| PWM7         | 56   | 
| PWM8         | 57   | 

## Battery Monitoring

The correct battery setting parameters are set by default and are:
 
 - BATT_MONITOR 4
 - BATT_VOLT_PIN 11
 - BATT_CURR_PIN 12
 - BATT_VOLT_SCALE 9.2
 - BATT_AMP_PERVLT 50.0

## Compass

The ACNS-F405AIO has one built-in compass LIS3MDLTR, you can also attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Firmware for these boards can be found at https://firmware.ardupilot.org in sub-folders labeled “ACNS-F405AIO”.

Initial firmware load can be done with DFU by plugging in USB with the
boot button pressed. Then you should load the "xxx_bl.hex"
firmware, using your favorite DFU loading tool.

Subsequently, you can update the firmware with Mission Planner.

## Pinout<div align=center>
<img width="500" src=F405AIO_top.jpg/>

<img width="500" src=F405AIO_bottom.jpg/>


