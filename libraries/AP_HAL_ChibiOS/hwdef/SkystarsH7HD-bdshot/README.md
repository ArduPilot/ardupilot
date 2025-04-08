# Skystars H7 Flight Controller

The Skystars H7 is a flight controller produced by [Skystars](http://www.skystars-rc.com/).

## Features

 - STM32H743 microcontroller
 - BMI270 IMU x2
 - BMP280 barometer
 - AT7456E OSD
 - 8 UARTs
 - 9 PWM outputs

## Pinout

![Skystars H7HD Board](SkystarsH7HD.jpg.jpg "Skystars H7HD")

## UART Mapping

The UARTs are marked RX and TX in the above pinouts. The RX pin is the
receive pin for UARTn. The TX pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART1 (RX, DMA-enabled)
 - SERIAL2 -> UART2 (DMA-enabled)
 - SERIAL3 -> UART3 (ESC Telem)
 - SERIAL4 -> UART4 (GPS, DMA-enabled)
 - SERIAL5 -> UART5 (VTX)
 - SERIAL6 -> UART6 (DJI FPV, DMA-enabled)
 - SERIAL7 -> UART7 (DMA-enabled)
 - SERIAL8 -> UART8

## RC Input

RC input is configured on the R1 (UART1_RX) pin in the DJI connector or via the R1/T1 (UART1) pads.
It supports all serial RC protocols. For protocols requiring half-duplex serial to transmit
telemetry (such as FPort) you should setup SERIAL1 with half-duplex and connect to T1 or use
R1 with pin-swap. You also need to set inversion enabled if using FPort.

## OSD Support

The Skystars H7 supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

The Skystars H7 supports up to 9 PWM outputs. The pads for motor output
M1 to M8 on the two motor connectors, plus M9 for LED strip or another
PWM output.

The PWM is in 5 groups:

 - PWM 1, 2 in group1
 - PWM 3, 4 in group2
 - PWM 5-8  in group3
 - PWM 9    in group4

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. Outputs 1-8 support bi-directional dshot.

## Battery Monitoring

The board has a builtin voltage and current sensor. The current
sensor can read up to 130 Amps. The voltage sensor can handle up to 6S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 10
 - BATT_CURR_PIN 11
 - BATT_VOLT_MULT 10.1
 - BATT_AMP_PERVLT 17.0

## Compass

The Skystars H7 does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
