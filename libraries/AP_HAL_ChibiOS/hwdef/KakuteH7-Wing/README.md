# KakuteH7-Wing Flight Controller

The KakuteH7-Wing is a flight controller produced by [Holybro](http://www.holybro.com/).

## Features
    Processor
        STM32H743 32-bit processor
        AT7456E OSD
    Sensors
        ICM42688 Acc/Gyro
        BMP280 barometer
    Power
        2S - 8S Lipo input voltage with voltage monitoring
        9V/12V, 1.5A BEC for powering Video Transmitter
        6V/7.2V, ?A BEC for servos
        3.3V, 1A BEC
    Interfaces
        14x PWM outputs DShot capable, 4 outputs BiDirDShot capable
        1x RC input
        6x UARTs/serial for GPS and other peripherals
        2x I2C ports for external compass, airspeed, etc.
        USB-C port
        Switchable 9V/12V VTX power
        2 Switchable Camera inputs
        All UARTS support hardware inversion. SBUS, SmartPort, and other inverted protocols work on any UART without “uninvert hack”
        Input for second battery monitor

## Pinout

![KakuteH7-Wing Board](KakuteH7-Wing-pinout.jpg "KakuteH7-Wing)

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART7 (TELEM1) with CTS/RTS DMA Enabled
 - SERIAL2 -> UART2 (Telem2) DMA Enabled
 - SERIAL3 -> UART1 (GPS) DMA Enabled
 - SERIAL4 -> USART3 (GPS2)
 - SERIAL5 -> UART5 (User) DMA Enabled
 - SERIAL6 -> USART6 (RX normally RCIN, BRD_ALT_CONFIG =1 for normal UART RX)
 - SERIAL7 -> UART8 (User) DMA Enabled

## RC Input

RC input is configured on the R6 (UART6_RX) pin. It supports all RC
protocols. For protocols requiring half-duplex serial to transmit
telemetry (such as FPort) you should set BRD_ALT_CONFIG=1 and setup
SERIAL6 as an RC input serial port, with half-duplex, pin-swap
and inversion enabled.
   
## OSD Support

The KakuteH7 supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

The KakuteH7 supports up to 14 PWM outputs. Outputs 1-10 support DShot. Outputs 5-8 support BiDirDshot.

The PWM is in 5 groups:

 - PWM 1-4 in group1
 - PWM 5,6 in group2
 - PWM 7,8 in group3
 - PWM 9,10 in group4
 - PWM 11-13 in group5
 - PWM 14 in group6

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has a built-in voltage and current sensor. The current
sensor can read up to ?? Amps. The voltage sensor can handle up to 6S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 8
 - BATT_CURR_PIN 4
 - BATT_VOLT_MULT 18.18
 - BATT_AMP_PERVLT 36.6
 
## Switchable Vidoe Supply and Camera Inputs

The camera input defaults to CAM1 but can be switched to CAM2 by setting a relay set to pin 81. Both cameras should be the same video format.

The 9V/12V video supply can be switched off (default is on) using a relay set topin 82.

## Compass

The KakuteH7-Wing does not have a built-in compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware
Firmware for these boards can be found at https://firmware.ardupilot.org in sub-folders labeled “KakuteH7-Wing”.

Initial firmware load can be done with DFU by plugging in USB with the
boot button pressed. Then you should load the "KakuteH7-Wing_bl.hex"
firmware, using your favourite DFU loading tool.

Subsequently, you can update firmware with Mission Planner.


