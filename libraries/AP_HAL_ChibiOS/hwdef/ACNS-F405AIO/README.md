# ACNS_F405AIO Integrated Flight Controller

The ACNS_F405AIO is a low-cost compact flight controller for multi-rotor UAV which integrated 4 BLheli_s ESCs on board.

## Features

 - STM32F405 microcontroller
 - IMU: BMI160,ICM42688
 - Mag: LIS3MDLTR
 - Baro: BMP280
 - 1 microSD card slot port
 - 6 UARTs and USB ports
 - 1 I2C port
 - 1 CAN port
 - 1 SBUS input and 8 PWM output (4 Internal ESCs,4 External PWM ports )
 - External SPI port
 - GRB LED on board
 - 128M flash on board
 - 4 BLheli_s ESCs, 3-4s, 30A, The motor oder matches the Arducotper X frame type config
 - Small footprint and Light weight, 39mm X 39mm X 10mm, 9g(without shell)
 - 
## UART Mapping

 - SERIAL0 -> USB(OTG1)
 - SERIAL1 -> USART1(Telem1)
 - SERIAL2 -> USART3(Telem2)
 - SERIAL3 -> UART4(GPS)
 - SERIAL4 -> UART6(GPS2)
 - SERIAL5 -> USART2(SBUS)
 
## GPIOs

4 External PWM channels can be used for GPIO functions.
The pin numbers for these PWM channels in ArduPilot are shown below:

| PWM Channels | Pin  |
| ------------ | ---- | 
| PWM5         | 54   |
| PWM6         | 55   | 
| PWM7         | 56   | 
| PWM8         | 57   | 

## Analog inputs

The ACNS_F405AIO flight controller has 3 analog inputs

 - ADC Pin11  -> Battery Current 
 - ADC Pin12  -> Battery Voltage 
 - ADC Pin10  -> RSSI voltage monitoring


![image](https://github.com/robinluojl/ardupilot/blob/add_fmu2/libraries/AP_HAL_ChibiOS/hwdef/ACNS-F405AIO/F405AIO_top.jpg)
![image](https://github.com/robinluojl/ardupilot/blob/add_fmu2/libraries/AP_HAL_ChibiOS/hwdef/ACNS-F405AIO/F405AIO_bottom.jpg)
