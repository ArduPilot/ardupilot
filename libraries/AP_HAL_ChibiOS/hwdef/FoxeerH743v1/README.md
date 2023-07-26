# FoxeerH743 Flight Controller

The FoxeerH743 is a flight controller produced by [Foxeer](https://www.foxeer.com/).

## Features

 - MCU - STM32H743 32-bit processor running at 480 MHz
 - IMU - MPU6000
 - Barometer - DPS310
 - OSD - AT7456E
 - Onboard Flash: 128Mbit
 - 7x UARTs
 - 9x PWM Outputs (8 Motor Output, 1 LED)
 - Battery input voltage: 2S-8S
 - BEC 3.3V 0.5A
 - BEC 5V 2A
 - BEC 10V 2A
 - DJI Connector

## Pinout

![FoxeerH743 Board](FoxeerH743_Board.JPG "FoxeerH743")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART1 (RX/SBUS, DMA-enabled)
 - SERIAL2 -> UART2 (VTX)
 - SERIAL3 -> UART3 (DMA-enabled)
 - SERIAL4 -> UART4 (DJI RX/SBUS, DMA-enabled)
 - SERIAL6 -> UART6 (GPS, DMA-enabled)
 - SERIAL7 -> UART7 (DJI OSD, DMA-enabled)
 - SERIAL8 -> UART8 (ESC Telemetry)

## RC Input

RC input is configured by default on the R4 (UART4_RX) pin in the DJI connector. It supports all serial RC
protocols. For protocols requiring separate half-duplex serial to transmit
telemetry (such as FPort) you should setup SERIAL1 as an RC input serial port,
with half-duplex, pin-swap and inversion enabled. You can also use the soldered pads on UART1 by setting SERIAL4_PROTOCOL to -1 and SERIAL1_PROTOCOL to 23. For PPM support on UART1_RX set BRD_ALT_CONFIG to 1.
 
## FrSky Telemetry
 
FrSky Telemetry can be supported using the T1 pin (UART1 transmit). You need to set the following parameters to enable support for FrSky S.PORT
 
  - SERIAL1_PROTOCOL 10
  - SERIAL1_OPTIONS 7
  
## OSD Support

The FoxeerH743 supports OSD using OSD_TYPE 1 (MAX7456 driver) or OSD_TYPE 3 if using DJI OSD

## PWM Output

The FoxeerH743 supports up to 9 PWM outputs. The pads for motor output
M1 to M8 are provided on both the motor connectors and on separate pads, plus
M9 on a separate pad for LED strip or another PWM output.

The PWM is in 4 groups:

 - PWM 1-4 in group1
 - PWM 5,6 in group2
 - PWM 7,8 in group3
 - PWM 9   in group4

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. Channels 1-8 support bi-directional dshot.

## Battery Monitoring

The board has a builting voltage and current sensor. The current
sensor can read up to 130 Amps. The voltage sensor can handle up to 8S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 13
 - BATT_CURR_PIN 12
 - BATT_VOLT_MULT 11
 - BATT_AMP_PERVLT 35.4

## Compass

The FoxeerH743 does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.

