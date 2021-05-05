# Flywoo GOKU GN 745 40A AIO

https://flywoo.net/products/goku-gn745-40a-aio-bl_32-mpu6000-25-5-x-25-5

The Flywoo GOKU GN 745 AIO is a flight controller produced by [Flywoo](https://flywoo.net/).

## Features

- MCU: STM32F745 32-bit processor,216MHz, 512Kbytes Flash
- IMU: MPU6000 (SPI)
- Barometer: BMP280
- Onboard LED：WS2812*4
- USB VCP Driver (all UARTs usable simultaneously; USB does not take up a UART)
- 7 hardware UARTS (UART1,2,3,4,5,6,7)
- Supports serial receivers (SBUS, iBus, Spektrum, Crossfire) only.
- PPM and PWM receivers are not supported.
- Onbord 8Mbytes for Blackbox logging
- 9V Power Out: 1.5A max
- 5V Power Out: 2.0A max
- 3.3V Power Out: 0.5A max
- Dimensions: 33.5x33.5mm
- Mounting Holes: Standard 25.5/26.5mm square to center of holes 
- Weight: 8.5g

- Built-in 40A BL_32 4in1 ESC
- Support BLheli / BLHELI_32
- Support PWM, Oneshot125, Oneshot42, Multishot, Dshot150, Dshot300, Dshot600,Dshot1200
- Input Voltage: 2-6S Lipo
- Continuous Current: 40A
- Firmware: BLHELI_32

## Pinout

![GOKU GN 745 40A AIO](GOKUGN745AIO_Pinout.png "GOKU GN 745 40A AIO")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
|Name|Pin|Function|
|:-|:-|:-|
|SERIAL0|COMPUTER|USB|
|SERIAL1|RX1/TX1|UART1 (TELEM)|
|SERIAL2|TX2/RX2|UART2 (TELEM)|
|SERIAL3|TX3/RX3|UART3 (RC Input/Output)|
|SERIAL4|TX4/RX4|UART4|
|SERIAL5|TX5/RX5|UART5|
|SERIAL6|TX6/RX6|UART6 (GPS)|
|SERIAL7|TX7/RX7|UART7|

UART3 supports RX and TX DMA. UART1, UART2, UART4, and UART6 supports TX DMA. UART5 and UART7 do not support DMA.

## RC Input
 
RC input is configured on the UART3, which supports serial RC protocols. This board does not support PPM.
  
## OSD Support

The GOKU GN 745 AIO supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

The GOKU GN 745 AIO supports up to 8 PWM outputs. The pads for motor output ESC1 to ESC4 on the above diagram are the first 4 outputs, there are four additional pads for PWM 5-8. All 8 outputs support DShot as well as all PWM types.

The PWM are in 5 groups:

PWM 6: Group 1
PWM 1-2 and 7-8: Group 2
LED: Group 3
PWM: Group 4
PWM 3-4: Group 5


Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. PWM 1-4 support bidirectional dshot.

## Battery Monitoring

The board has a builtin voltage sensor. The voltage sensor can handle 2S to 6S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 12
 - BATT_VOLT_MULT around 10.9
 - BATT_CURR_PIN 13
 - BATT_CURR_MULT around 28.5

These are set by default in the firmware and shouldn't need to be adjusted

## Compass

The GOKU GN 745 AIO does not have a builtin compass but it does have an external I2C connector.

## NeoPixel LED

The board includes a NeoPixel LED pad.

## Loading Firmware (you will need to compile your own firmware)

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
