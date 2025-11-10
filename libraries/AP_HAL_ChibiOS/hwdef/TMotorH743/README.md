# T-Motor H7 Mini Flight Controller

The T-Motor H7 Mini is a flight controller produced by [T-Motor](https://store.tmotor.com/goods-1295-H7+MINI.html).

## Features

 - MCU - STM32H743 32-bit processor running at 480 MHz
 - IMU - ICM42688/BMI270
 - Barometer - DPS310
 - OSD - AT7456E
 - Onboard Flash: 128Mbits
 - 6x UARTs (1,5,6,7,9)
 - 9x PWM Outputs (8 Motor Output, 1 LED)
 - Battery input voltage: 2S-6S
 - BEC 5V/2A, 10V/1.5A

## Pinout

![T-Motor H7 Mini Board](TMotorH7Mini_Board.JPG "T-Motor H7 Mini")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART1 (ESC Telemetry)
 - SERIAL3 -> UART3
 - SERIAL5 -> UART5 (GPS, DMA-enabled)
 - SERIAL6 -> UART6 (RX, DMA-enabled)
 - SERIAL7 -> UART7 (DJI VTX, DMA-enabled)
 - SERIAL8 -> UART8 (SBUS, DMA-enabled)

## RC Input

RC input is configured on the R6/TX6 (UART6_RX/UART6_TX) pins. It supports all serial RC
protocols. For protocols requiring half-duplex serial to transmit
telemetry (such as FPort) you should setup SERIAL6 with half-duplex, pin-swap and inversion enabled.

## OSD Support

The T-Motor H7 Mini supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

The T-Motor H7 Mini supports up to 5 PWM outputs. The pads for motor output
M1 to M4 on the motor connectors and separate pads, plus
M5 for LED strip or another PWM output.

The PWM is in 2 groups:

 - PWM 1-4 in group1
 - PWM 5 in group2

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. Channels 1-4 support bi-directional dshot.

## Battery Monitoring

The board has a built-in voltage and current sensor. The current
sensor can read up to 130 Amps. The voltage sensor can handle up to 6S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 11
 - BATT_CURR_PIN 13
 - BATT_VOLT_MULT 11.0
 - BATT_AMP_PERVLT 50.0

## Compass

The T-Motor H7 Mini does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.

