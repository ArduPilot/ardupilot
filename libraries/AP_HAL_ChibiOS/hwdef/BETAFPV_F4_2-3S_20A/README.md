# BETAFPV F4 2-3S 20A AIO FC V1 Flight Controller

The F4 2-3S 20A AIO FC V1 is a flight controller produced by [BETAFPV](https://betafpv.com/products/f4-2-3s-20a-aio-fc-v1).

## Features

 - MCU: STM32F405RGT6, 168MHz
 - Gyro: ICM42688-P
 - 16Mb Onboard Flash
 - BEC outputs: independent 5@3A and 9V@2A
 - Barometer: BMP280/DSP310
 - 4 UARTS: (UART1, UART3, UART4, UART6)
 - 5 PWM outputs (4 motor outputs used internally for integrated 4-in-1 ESC and 1 integrated LED)
 - Integrated 4-in-1 BlueJay ESC 20A
 - Works propperly on 2S, the 5V BEC is buck boost
 - I2C broken out
 - Integerated ELRS

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
|Name|Pin|Function|
|:-|:-|:-|
|SERIAL0|COMPUTER|USB|
|SERIAL1|RX1/TX1|UART1 |
|SERIAL3|TX3/RX3|UART3 (ELRS, internal), ELRS can be disconnected|
|SERIAL4|TX4/RX4|UART4 |
|SERIAL6|RX6|UART6 |

## RC Input

RC input is configured on the on-board ELRS, but it can be disabled and converts to Sbus

## OSD Support

This board has no OSD

## PWM Output

The BETAFPV F405 AIO has 4 PWM outputs internally connected to its 4-in-1 ESC. The pads for motor output are M1 to M4 on the board. All 4 outputs support bi-directional DShot and DShot, as well as all PWM types. The default configuration is for bi-directional DShot using the already installed BlueJay firmware.

The PWM are in in two groups:

 - PWM 1-2 in group1
 - PWM 3-4 in group2
 - PWM 5 in group3
 - PWM 6 in group4
 - PWM 7 in group5

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has a builtin voltage sensor and a current sensor inputs.


## Compass

This board does not have a builtin compass, but has I2C broken out that is configured to accept a compass

## NeoPixel LED

The board includes 2 NeoPixel LED on the underside/upside which is pre-configured to output ArduPilot sequences. This is the seventh PWM output.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
