# Ewing Aerospace H743QF Flight Controller

The Ewing Aerospace H743QF is a flight controller produced by [Ewing Aerospace](https://ewingaerospace.com/).

## Features

 - STM32H743 microcontroller
 - ICM42688 IMU
 - BMP390 barometer
 - MAX7456 OSD
 - 6 UARTs
 - 6 PWM outputs

## Pinout

![Ewing Aerospace H743QF v1](Ewing_Aeropsace_H743QF_Board_Top.png "Ewing Aerospace H743QF v1")
![Ewing Aerospace H743QF v1](Ewing_Aeropsace_H743QF_Board_Bottom.png "Ewing Aerospace H743QF v1")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART1 (GPS)
 - SERIAL2 -> UART2 (HD OSD) 
 - SERIAL3 -> UART3 (ANALOG OSD)
 - SERIAL4 -> UART4 (SIK Radio GCSD)
 - SERIAL5 -> UART5 (Open use (NODMA))
 - SERIAL6 -> UART6 (RX-only from HD VTX)

## RC Input

RC input is configured on the R4 (UART4_RX) pin for most RC unidirectional protocols
 
## OSD Support

The Ewing Aerospace H743QF supports OSD using OSD_TYPE 1 (MAX7456 driver).

## VTX Support

The JST-GH-6P connector supports a standard DJI HD VTX connection. Pin 1 of the connector is 9v so be careful not to connect
this to a peripheral requiring 5v.

## PWM Output

The Ewing Aerospace H743QF supports up to 6 PWM outputs.

The PWM is in 3 groups:

 - PWM 1-2 in group1
 - PWM 3-4 in group2
 - PWM 5-6 in group3

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. Channels 1-4 support bi-directional DShot.

## Battery Monitoring

The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input.
The voltage sensor can handle up to 6S.
LiPo batteries.

The default battery parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 10
 - BATT_CURR_PIN 11
 - BATT_VOLT_MULT 11.1
 - BATT_AMP_PERVLT 100 (will need to be adjusted for whichever current sensor is attached)

## Compass

The Ewing Aerospace H743QF does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favorite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.

