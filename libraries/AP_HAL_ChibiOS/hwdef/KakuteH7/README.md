# KakuteH7 Flight Controller

The KakuteH7 is a flight controller produced by [Holybro](http://www.holybro.com/).

## Features

 - STM32H743 microcontroller
 - ICM20689 IMU
 - BMP280 barometer
 - microSD card slot
 - AT7456E OSD
 - 6 UARTs
 - 9 PWM outputs

## Pinout

![KakuteH7 Board](Kakkute_H7_Board_Top__Bottom2.jpg "KakuteH7")
![KakuteH7 Board](Kakkute_H7_Board_Top__Bottom3.jpg "KakuteH7")
![KakuteH7 Board](Kakkute_H7_Board_Top__Bottom4.jpg "KakuteH7")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART1 (Telem1)
 - SERIAL2 -> UART2 (Telem2) (connected to internal BT module)
 - SERIAL3 -> UART3 (GPS)
 - SERIAL4 -> UART4
 - SERIAL5 -> not available
 - SERIAL6 -> UART6
 - SERIAL7 -> UART7

## RC Input

RC input is configured on the R6 (UART6_RX) pin. It supports all RC
protocols. For protocols requiring half-duplex serial to transmit
telemetry (such as FPort) you should set BRD_ALT_CONFIG=1 and setup
SERIAL6 as an RC input serial port, with half-duplex, pin-swap
and inversion enabled.
 
## FrSky Telemetry
 
FrSky Telemetry is supported using the Tx pin of any UART including SERIAL6/UART6 . You need to set the following parameters to enable support for FrSky S.PORT (example shows SERIAL6). Note this assumes the RC input is using default (ALT_BRD_CONFIG =0). Obviously, if using ALT_BRD_CONFIG = 1 for full duplex RC prtocols, you must a different UART for FrSky Telemetry.
 
  - SERIAL6_PROTOCOL 10
  - SERIAL6_OPTIONS 7
  
## OSD Support

The KakuteH7 supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

The KakuteH7 supports up to 9 PWM outputs. The pads for motor output
M1 to M8 on the two motor connectors, plus M9 for LED strip or another
PWM output.

The PWM is in 5 groups:

 - PWM 1, 2 in group1
 - PWM 3, 4 in group2
 - PWM 5, 6 in group3
 - PWM 7, 8 in group4
 - PWM 9 in group5

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has a builting voltage and current sensor. The current
sensor can read up to 130 Amps. The voltage sensor can handle up to 6S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 10
 - BATT_CURR_PIN 11
 - BATT_VOLT_MULT 10.1
 - BATT_AMP_PERVLT 17.0

## Compass

The KakuteH7 does not have a builting compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.

