# JHEMCUF405PRO (aka GHF405AIO-HD) Flight Controller

The JHEMCUF405PRO is an AIO flight controller produced by [JHEMCU](https://jhemcu.com/).

There's a different version of the board that doesn't have I2C pads exposed. IT's not that one.

## Features

 - MCU - STM32F405 32-bit processor running at 168 MHz
 - IMU - dual ICM42605
 - Barometer - DPS310
 - Voltage & current sensor
 - OSD - AT7456E
 - Onboard Flash: 8MB
 - 5x UARTs (1,2,3,4,6)
 - 5x PWM Outputs (4 Motor Output, 1 LED)
 - Battery input voltage: 2S-6S
 - BEC 5V/2.5A, 10V/2.0A
 - I2C: exposed pins
 - ESCs: BlheliS 40A (advertised)
 - LED strip: Supported
 - Buzzer: supported

## Pinout

![JHEMCUF405PRO Board](JHEMCUF405PRO.png "JHEMCUF405PRO")

## UART Mapping

The UARTs are marked RXn and TXn in the above pinouts. The RXn pin is the
receive pin for UARTn. The TXn pin is the transmit pin for UARTn.

In addition to pinouts, the board also has SH6P 1mm connector for digital FPV systems.
Please note that the board will not enter DFU mode if the receiver (eg. CRSF/ELRS is connected to either USART1/USART3/USART4). If you have it soldered there due to DMA requirement you need to temprarily desolder the wire on the FC's RX pad and solder it back after flashing

Bidirectional Dshot version
 - SERIAL0 -> USB
 - SERIAL1 -> USART1 (DMA-enabled)
 - SERIAL2 -> USART2 (DMA-enabled only on Tx)
 - SERIAL3 -> USART3 (DMA-enabled)
 - SERIAL4 -> UART4  (DMA-enabled only on Tx)
 - SERIAL6 -> USART6 (DMA-enabled)

Normal version
 - SERIAL0 -> USB
 - SERIAL1 -> USART1 (DMA-enabled)
 - SERIAL2 -> USART2 (DMA-enabled only on Tx)
 - SERIAL3 -> USART3 (DMA-enabled only on Tx)
 - SERIAL4 -> UART4  (DMA-enabled only on Tx)
 - SERIAL6 -> USART6 (DMA-enabled)

These statements are based on the current implementation of the dma_resolver.py script at the time of writing.

## DMA
The Bidirectional Dshot version of the firmware has the advantage of full DMA on USART3 (and bidirectional dshot obviously).

The Normal version of the firmware has the advantage of SPI3 (Blackbox and OSD) being on a dedicated DMA channel which might improve max logging speed

## I2C
The SCL/SDA pads are exposed as two tiny circles which you can connect a magnetometer to for example

## RC Input

RC input is best configured on the RX1/TX1 (USART1_RX/USART1_TX) pins due to having full DMA capability and being mostly easy to access for desoldering to enable DFU mode

## OSD Support

JHEMCUF405PRO supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

JHEMCUF405PRO supports up to 5 PWM outputs. 4 motors and 1 LED strip or another PWM output.
Channels 1-8 support bi-directional dshot if compiled the -bdshot version.

## Battery Monitoring

The board has a built-in voltage and current sensor. The current
sensor's max Amps is not specified. The voltage sensor can handle up to 6S
LiPo batteries.

## Compass

JHEMCUF405PRO does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.

If you're having trouble entering DFU mode then try desoldering the wire coming from the receiver to the board's Rx pad.