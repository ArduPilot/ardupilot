# DAKEFPV F405 Flight Controller

The DAKEFPV F405 is a flight controller produced by [DAKEFPV](https://www.dakefpv.com/).

## Features

 - MCU - STM32F405 32-bit processor. 1024Kbytes Flash
 - IMU - Dual ICM42688
 - Barometer - SPL06
 - OSD - AT7456E
 - Onboard Flash: 16MByte
 - 8x UARTs
 - 13x PWM Outputs (12 Motor Output, 1 LED)
 - Battery input voltage: 4S-12S
 - BEC 3.3V 0.5A
 - BEC 5V 3A
 - BEC 12V 3A for video, gpio controlled
 - Dual switchable camera inputs

## Pinout

![DAKEFPV F405 Board Top](Top.jpg "DAKEFPV F405 Top")
![DAKEFPV F405 Board Bottom](Bottom.jpg "DAKEFPV F405 Bottom")

## DAKEFPV F405 Wiring Diagram

![DAKEFPV F405 Wiring Diagram Top]( WiringDiagramTop.jpg "DAKEFPV F405 Wiring Diagram Top")
![DAKEFPV F405 Wiring Diagram Bottom](WiringDiagramBottom.jpg "DAKEFPV F405 Wiring Diagram Bottom")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART1 (GPS, DMA-enabled)
 - SERIAL2 -> UART2 (ELRS)
 - SERIAL3 -> UART3 (ESC Telemetry)
 - SERIAL4 -> UART4 (VTX)
 - SERIAL5 -> UART5 (User)
 - SERIAL6 -> UART6 (User)

## RC Input

RC input is configured by default via the USART5 RX input. It supports all serial RC protocols except PPM.

Note: If the receiver is FPort the receiver must be tied to the USART5 TX pin , RSSI_TYPE set to 3,
and SERIAL5_OPTIONS must be set to 7 (invert TX/RX, half duplex). For full duplex like CRSF/ELRS use both
RX5 and TX5 and set RSSI_TYPE also to 3.

## FrSky Telemetry

FrSky Telemetry is supported using an unused UART, such as the T5 pin (UART5 transmit).
You need to set the following parameters to enable support for FrSky S.PORT:

  - SERIAL5_PROTOCOL 10
  - SERIAL5_OPTIONS 7

## PWM Output

The DAKEFPV F405 supports up to 6 PWM or DShot outputs. The pads for motor output
M1 to M6 are provided on both the motor connectors and on separate pads.

The PWM is in 3 groups:

 - PWM 1-2   in group1
 - PWM 3-4   in group2
 - PWM 5-6   in group3


Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. Channels 1-10 support bi-directional dshot.

## Battery Monitoring

The board has a built-in voltage sensor and external current sensor input. The current
sensor can read up to 130 Amps. The voltage sensor can handle up to 12S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 11
 - BATT_CURR_PIN 13
 - BATT_VOLT_MULT 11.1
 - BATT_AMP_PERVLT 40

## Compass

The DAKEFPV F405 does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Camera control

GPIO 82 controls the camera output to the connectors marked "CAM1" and "CAM2". Setting this GPIO low switches the video output from CAM1 to CAM2. By default RELAY3 is configured to control this pin and sets the GPIO high.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
\*.apj firmware files.
