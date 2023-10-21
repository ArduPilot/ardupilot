# GOKU F405 Pro

https://flywoo.net/products/goku-versatile-f405-pro-fc-20x20-30-5x30-5

The Flywoo GOKU F405 Pro is a flight controller produced by [Flywoo](https://flywoo.net/).

## Features

- MCU: STM32F405 32-bit processor. 1024Kbytes Flash
- IMU: ICM42688 (SPI)
- Barometer: BMP280
- 6 hardware UARTS (UART1,2,3,4,5,6)
- Onbord 16Mbytes for Blackbox logging
- 5V Power Out: 2.0A max
- 10V Power Out: 2.0A max
- 9 PWM outputs
- Mounting Holes: Standard 20x20 or 30.5/30.5mm square to center of holes 
- Weight: 5.5g
- Input Voltage: 2-6S Lipo

## Pinout

![GOKU F405 Pro](GOKUF405Pro_Pinout.PNG "GOKU F405 Pro")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
|Name|Pin|Function|
|:-|:-|:-|
|SERIAL0|COMPUTER|USB|
|SERIAL1|RX1/TX1|USART1 (GPS)|
|SERIAL2|RX2|USART2 (ESC Telemetry)|
|SERIAL3|TX3/RX3|USART3 (DJI, DMA-enabled)|
|SERIAL4|TX4/RX4|UART4 (RX/SBUS)|
|SERIAL5|TX5/RX5|UART5|
|SERIAL6|TX6/RX6|UART6 (RX, DMA-enabled)|

USART3 and USART6 supports RX and TX DMA. UART4 supports TX DMA. UART1, UART2 and UART5 do not support DMA.

## RC Input
 
RC input is configured on UART3, UART4 or UART6 which support serial RC protocols. SBUS *must* be used on UART4 which has hardware inversion, UART5 also has hardware inversion but no DMA so use for SBUS is not recommended.
  
## OSD Support

The GOKU F405 Pro supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

The GOKU F405 Pro supports up to 9 PWM outputs. The pads for motor output ESC1 to ESC4 on the above diagram are the first 4 outputs.All 9 outputs support DShot.

The PWM are in 5 groups:

PWM 1-2: Group 1
PWM 3-4: Group 2
PWM 5,7: Group 3
PWM 6,8: Group 4
LED: Group 5

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. PWM 1-4 support bidirectional dshot.

## Battery Monitoring

The board has a builtin voltage sensor. The voltage sensor can handle 2S to 6S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 12
 - BATT_VOLT_MULT around 11
 - BATT_CURR_PIN 13
 - BATT_CURR_MULT around 59

These are set by default in the firmware and shouldn't need to be adjusted

## Compass

The GOKU F405 Pro does not have a builtin compass but it does have an external I2C connector.

## NeoPixel LED

The board includes a NeoPixel LED pad.

## Loading Firmware (you will need to compile your own firmware)

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
