# ARK FPV Flight Controller

https://arkelectron.com/product/ark-fpv-flight-controller/


## Features
#### Processor
- STM32H743 32-bit processor
- 480MHz
- 2MB Flash
- 1MB RAM
#### Sensors
- Invensense IIM-42653 Industrial IMU with heater resistor
- Bosch BMP390 Barometer
- ST IIS2MDC Magnetometer
#### Power
- 5.5V - 54V (2S - 12S) input
- 12V, 2A output
- 5V, 2A output. 300ma for main system, 200ma for heater
#### Interfaces
- **Micro SD**
- **USB-C**
  - VBUS In, USB
- **PWM**
  - VBAT In, Analog Current Input, Telem RX, 4x PWM and Bidirectional-DSHOT capable
  - JST-GH 8 Pin
- **PWM EXTRA**
  - 5x PWM and Bidirectional-DSHOT capable
  - JST-SH 6 Pin
- **RC Input**
  - 5V Out, UART
  - JST-GH 4 Pin
- **POWER AUX**
  - 12V Out, VBAT In/Out
  - JST-GH 3 Pin
- **TELEM**
  - 5V Out, UART with flow control
  - JST-GH 6 Pin
- **GPS**
  - 5V Out, UART, I2C
  - JST-GH 6 Pin
- **CAN**
  - 5V Out, CAN
  - JST-GH 4 Pin
- **VTX**
  - 12V Out, UART TX/RX, UART RX
  - JST-GH 6 Pin
- **SPI** (OSD or External IMU)
  - 5V Out, SPI
  - JST-SH 8 Pin
- **DEBUG**
  - 3.3V Out, UART, SWD
  - JST-SH 6 Pin

##### Dimensions
- Size: 3.6 × 3.6 × 0.8 cm
- Weight: 7.5g with MicroSD card

## Pinout
TODO

## UART Mapping

|Name|Function|
|:-|:-|
|SERIAL0|USB|
|SERIAL1|UART7 (Telem)|
|SERIAL2|UART5 (VTX - DJI Air Unit, RX/TXT)|
|SERIAL3|USART1 (GPS1)|
|SERIAL4|USART2 (VTX - DJI Air Unit, RX only)|
|SERIAL5|UART4 (ESC Telem, RX only)|
|SERIAL6|USART6 (RC Input)|
|SERIAL7|OTG2 (SLCAN)|

All UARTS support DMA.

## OSD Support

This flight controller has an MSP-DisplayPort output on a 6-pin DJI-compatible JST SH.

## Motor Output

Motors 1-8 are capable of Bidirectional-DSHOT and PWM.

All outputs in the motor groups below must be either PWM or DShot:
Motors 1-4  Group1 (TIM5)
Motors 5-8  Group2 (TIM8)
