# RadiolinkF405 Flight Controller

The RadiolinkF405 is a flight controller produced by [RadioLink](https://www.radiolink.com/).

## Features

 - STM32F405 microcontroller
 - ICM42688 IMU
 - SPL06 barometer
 - W25N01G flash
 - AT7456E OSD
 - 5 UARTs
 - 7 PWM outputs

## Pinout

![RadiolinkF405](RadiolinkF405_Board_Top.jpg "RadiolinkF405")
![RadiolinkF405](RadiolinkF405_Board_Bottom.jpg "RadiolinkF405")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART1 (DJI-VTX)
 - SERIAL2 -> UART2 (RCIN, DMA-enabled) 
 - SERIAL3 -> UART3 (ESC Telemetry, DMA-enabled)
 - SERIAL4 -> UART4 (IDEL)
 - SERIAL5 -> UART5 (GPS)

## RC Input

RC input is configured on the R2 (UART2_RX) pin for most RC unidirectional protocols except SBUS which should be applied at the SBUS pin. PPM is not supported.
For CRSF/ELRS/SRXL2 connection of the receiver to T2 will also be required.
  
## OSD Support

The RadiolinkF405 supports OSD using OSD_TYPE 1 (MAX7456 driver).

## VTX Support

The JST-GH-6P connector supports a standard DJI HD VTX connection. Pin 1 of the connector is 9v so be careful not to connect
this to a peripheral requiring 5v.

## PWM Output

The RadiolinkF405 supports up to 7 PWM outputs. The pads for motor output
M1 to M4 on the motor connector, M5 M6 for servo or another PWM output, LED pads for led strip.

The PWM is in 3 groups:

 - PWM 1-4 in group1
 - PWM 5-6 in group2
 - PWM 7   in group3

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. Channels 1-4 support bi-directional DShot.

## Pin IO

- PINIO1: 9V DCDC control (HIGH:on; LOW:off)

*Note: DCDC default is enabled.*

## Battery Monitoring

The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input.
The voltage sensor can handle up to 6S.
LiPo batteries.

The default battery parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 12
 - BATT_CURR_PIN 11
 - BATT_VOLT_MULT 11
 - BATT_AMP_PERVLT 25 (will need to be adjusted for whichever current sensor is attached)

## Compass

The RadiolinkF405 does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.

