
# Brahma F4 Flight Controller

Brahma F4 is a flight controller manufactured by [Darkmatter®](https://thedarkmatter.in) 

## Features

- MCU: STM32F405RGT6, 168MHz
- Gyro: BMI270
- 32Mb Onboard Flash
- BEC output: 5V, 2A@4V
- Barometer: BMP280
- OSD: AT7456E
- 4 UARTS: (UART1, UART3, UART4, UART6)
- 9 PWM Outputs (includes 1 neopixel out can be used for pwm)
- 5V Power Out: 2.0A max
- 9V Power Out: 2.0A max


## Pinout

![BrahmaF405](BRAHMA_F405-diagram.jpg "DM_BrahmaF4")

## UART Mapping

The UARTs are marked Rx and Tx in the above pinouts.

| Name     | Pin      | Function                |
| :------- | :------- | :------------------------- |
| SERIAL 0 | COMPUTER | USB     |
| SERIAL 1 | RX1/TX1  | USART1  | (DMA)   (Reciever)
| SERIAL 2 | RX3/TX3  | USART 3 | (NODMA) (ESC)
| SERIAL 3 | RX4/TX4  | USART 4 | (NODMA) (Spare)
| SERIAL 4 | RX6/TX6  | USART 6 | (DMA)   (GPS)

USART1 and USART6 supports RX and TX DMA. UART2 and UART4 do not support DMA.


## RC Input

RC input is configured by default on the R1 (UART1_RX) pad.(CRSF or ELRS)
SBUS inverted pad is available  
(PPM is disabled currently)

## OSD

The Darkmatter® Brahma F4 supports OSD using OSD_TYPE 1 (MAX7456 driver).


## PWM Output
The PWM is in 3 groups:

- PWM 1-4 in group1 (DSHOT150 recommended)
- PWM 4-8 in group2
- PWM 9   in group3

Pads for PWM 1-8 are available on bottom side of pcb
ESC port is a jst-sh V-G-C-R3-1-2-3-4

Channels within the same group need to use the same output rate.
All channels in specific Timer groups are configured either as DSHOT or PWM,
mixing of protocols within groups is not possible
Note that channel 9 is configured as NeoPixel can be set for extra pwm.
## Battery Monitoring

The board has a builting voltage sensor. The voltage sensor can handle up to 6S LiPo batteries.

The correct battery setting parameters are:

- BATT_MONITOR 4
- BATT_VOLT_PIN 14
- BATT_VOLT_MULT 11
- BATT_CURR_PIN 13
- BATT_CURR_MULT 37 


## Compass

The Darkmatter® Brahma F4 do not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.
## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "with_bl.hex" firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the *.apj firmware files.
