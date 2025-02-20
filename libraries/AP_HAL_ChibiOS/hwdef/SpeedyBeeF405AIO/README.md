# SpeedyBee F405 AIO 40A Bluejay

https://www.speedybee.com/speedybee-f405-aio-40a-bluejay-25-5x25-5-3-6s-flight-controller

The SpeedyBee F405 AIO is a flight controller produced by [SpeedyBee](https://www.speedybee.com/).

## Features

- MCU: STM32F405 32-bit processor. 1024Kbytes Flash
- IMU: ICM-42688P (SPI)
- Barometer: SPA06-003
- USB VCP Driver (all UARTs usable simultaneously; USB does not take up a UART)
- 4 hardware UARTS (UART3,4,5,6) + SBUS
- Onbord 8Mbytes for Blackbox logging
- 5V Power Out: 2.0A max
- Dimensions: 33x33mm
- Mounting Holes: Standard 25.5/25.5mm square to center of holes 
- Weight: 13.6g

- Built-in 40A BlueJay 4in1 ESC
- Supports Oneshot125, Oneshot42, Multishot, Dshot150, Dshot300, Dshot600
- Input Voltage: 3S-6S Lipo
- Continuous Current: 40A
- Bluejay JH-40 48kHz

## Pinout

![SpeedyBee F405 AIO](SpeedyBeeF405AIO_Pinout.png "SpeedyBee F405 AIO")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
|Name|Pin|Function|
|:-|:-|:-|
|SERIAL0|COMPUTER|USB|
|SERIAL1|RX1/TX1|USART1 (WiFi, not usable by ArduPilot)|
|SERIAL2|RX2|USART2 (SBUS, Inverted)|
|SERIAL3|TX3/RX3|USART3 (VTX)|
|SERIAL4|TX4/RX4|UART4 (User)|
|SERIAL5|TX5/RX5|UART5 (GPS)|
|SERIAL6|TX6/RX6|UART6 (RX, DMA-enabled)|

USART6 supports RX and TX DMA.

## RC Input
 
RC input is configured on UART2 or UART6, which supports serial RC protocols. This board does not support PPM.
  
## OSD Support

The SpeedyBee F405 AIO supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

The SpeedyBee F405 AIO supports up to 5 PWM outputs. The pads for motor output ESC1 to ESC4 on the above diagram are the first 4 outputs.All 5 outputs support DShot.

The PWM are in 3 groups:

PWM 1-2: Group 1
PWM 3-4: Group 2
LED: Group 3

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. PWM 1-4 support bidirectional dshot.

## Battery Monitoring

The board has a builtin voltage sensor. The voltage sensor can handle 2S to 6S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 10
 - BATT_VOLT_MULT around 11
 - BATT_CURR_PIN 12
 - BATT_CURR_MULT around 40

These are set by default in the firmware and shouldn't need to be adjusted

## Compass

The SpeedyBee F405 AIO does not have a builtin compass but it does have an external I2C connector.

## NeoPixel LED

The board includes a NeoPixel LED pad.

## Loading Firmware (you will need to compile your own firmware)

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
