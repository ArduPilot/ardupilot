# ZeroOneX9 Series Flight Controller

The ZeroOne X9 is a series of flight controllers manufactured by ZeroOne, which is based on the open-source FMU v6X architecture and highly integrates all the features in a smaller volume.

![X9 Series](X9series.jpg)

## Features

- Separate flight control core design.
- MCU

   STM32H753IIK6 32-bit processor running at 480MHz
   2MB Flash
   1MB RAM

- IO MCU

   STM32F103

- Input voltage

   4.5V to 5.4V

- X9 Core

   Dimensions: 38.8mm x 38.8mm x 26.5mm
   Weight: 57.5g
   Interface baseboard: None

- X9 Cube

   Dimensions: 38.8mm x 38.8mm x 38.8mm
   Weight: 72g
   Interfaces: 2 CAN, 2 TELEM, 2 GPS, 1 I2C, 1 Ethernet,
   1 SBUS input/output connector, USB-C, microSD, 2 power inputs,
   10 servo-rail PWM outputs and 4 expansion PWM outputs

- Sensors
  - IMU:

       Internal Vibration Isolation for IMUs
       IMU constant temperature heating(2W heating power).
       With Triple Synced IMUs, BalancedGyro technology, low noise and more shock-resistant:

    - **X9**:

       IMU1-IIM42652(With vibration isolation)
       IMU2-IIM42652(With vibration isolation)
       IMU3-IIM42652(No vibration isolation)

  - Baro:

       Two barometers: 2 x ICP20100 or BMP581+SPL06

  - Magnetometer:

       Builtin RM3100 magnetometer

## Pinout

![ZeroOneX9 Pinout](ZeroOneX9Pinout.jpg "ZeroOneX9")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinout, where n is the ArduPilot
SERIAL port number, not the STM32 UART number. The Rn pin is the receive pin
for SERIALn and the Tn pin is its transmit pin.

| Port | UART | Protocol | Available on | DMA |
| --- | --- | --- | --- | --- |
| SERIAL0 | USB | MAVLink2 | USB-C | |
| SERIAL1 | UART7 | MAVLink2 | TELEM1 (R1/T1) | Yes |
| SERIAL2 | UART5 | MAVLink2 | TELEM2 (R2/T2) | Yes |
| SERIAL3 | USART1 | GPS | GPS1 (R3/T3) | Yes |
| SERIAL4 | UART8 | GPS | GPS2 (R4/T4) | Yes |
| SERIAL5 | USART2 | MAVLink2 | Core connector only | Yes |
| SERIAL6 | UART4 | None | Core connector only (EXT2) | Yes |
| SERIAL7 | USART3 | None | Core connector only (FMU debug) | Yes |
| SERIAL8 | USB | SLCAN | USB-C | |

The CTS/RTS signals for SERIAL1, SERIAL2 and SERIAL5 are available only on the
core connectors. The Mini Baseboard TELEM1 and TELEM2 connectors expose GND,
RX, TX and 5V only.

SERIAL3 is configured for a DroneCAN GPS by default (`GPS1_TYPE=9`). Set
`GPS1_TYPE` to the appropriate serial GPS type when using the GPS1 USART port.

## RC Input

The SBUS IN pin can be used for all ArduPilot-supported receiver protocols
except CRSF/ELRS and SRXL2, which require a full UART. FPort on this pin provides
RC input without telemetry. Use a Mini Baseboard UART such as TELEM2 (SERIAL2)
for protocols requiring a full UART. See [RC systems](https://ardupilot.org/copter/docs/common-rc-systems.html)
for configuration details.

The SBUS OUT pin on the same connector provides an SBUS output controlled by
the `BRD_SBUS_OUT` parameter.

## PWM Output

The X9 supports up to 16 PWM outputs. The Mini Baseboard exposes 14 of them:
Main 1-8 on the servo rail, AUX 1-2 on the servo rail and AUX 3-6 on the A3-A6
connector. AUX 7 and AUX 8 are available on the core connector only.

Main 1-8 are controlled by the STM32F103 IO controller. AUX 1-8 are directly
attached to the STM32H753 FMU controller. All outputs support normal PWM. Main
1-8 and AUX 1-6 support DShot and bi-directional DShot; AUX 7-8 do not support
DShot.

The 8 IO PWM outputs are in 3 groups:

- Main 1 and 2 in group1
- Main 3 and 4 in group2
- Main 5, 6, 7 and 8 in group3

The 8 FMU PWM outputs are in 3 groups:

- AUX 1, 2, 3 and 4 in group1 (TIM5)
- AUX 5 and 6 in group2 (TIM4)
- AUX 7 and 8 in group3 (TIM12)

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

## GPIOs

All PWM outputs can be used as GPIOs (relays, camera, RPM etc). To use them you need to set the output’s SERVOx_FUNCTION to -1. The numbering of the GPIOs for PIN variables in ArduPilot is:

| Pad | Output | GPIO |
| --- | --- | ---: |
| Main 1 | MainOut1 | 101 |
| Main 2 | MainOut2 | 102 |
| Main 3 | MainOut3 | 103 |
| Main 4 | MainOut4 | 104 |
| Main 5 | MainOut5 | 105 |
| Main 6 | MainOut6 | 106 |
| Main 7 | MainOut7 | 107 |
| Main 8 | MainOut8 | 108 |
| AUX 1 | AuxOut1 | 50 |
| AUX 2 | AuxOut2 | 51 |
| AUX 3 | AuxOut3 | 52 |
| AUX 4 | AuxOut4 | 53 |
| AUX 5 | AuxOut5 | 54 |
| AUX 6 | AuxOut6 | 55 |
| AUX 7 (core only) | AuxOut7 | 56 |
| AUX 8 (core only) | AuxOut8 | 57 |
| FCU CAP | Capture input | 58 |

## Battery Monitoring

The X9 flight controller has two power connectors supporting CAN power modules.
Battery monitoring defaults to DroneCAN (`BATT_MONITOR=8`), so a DroneCAN-capable
power module is required. Change `BATT_MONITOR` when using another monitor type.

## CAN

The X9 provides two CAN FD-capable interfaces. Both ports are enabled by default
with `CAN_P1_DRIVER=1` and `CAN_P2_DRIVER=1`.

## Ethernet

The X9 provides a 100 Mbps Ethernet interface using a LAN8742A RMII PHY.

## Compass

The X9 has a built-in RM3100 compass. Due to potential interference, the
autopilot is usually used with an external I2C compass as part of a GPS/Compass
combination, connected to the I2C1 connector.

## Analog Inputs

The X9 has 3 analog inputs. These signals are available on the X9 Core
connectors only and are not exposed on the X9 Cube baseboard.

- ADC Pin12 -> ADC 6.6V Sense
- ADC Pin13 -> ADC 3.3V Sense
- RSSI input pin = 103

## 5V PWM Voltage

The X9 flight controller supports switching between 5V and 3.3V PWM levels. Switch PWM output pulse level by configuring parameter BRD_PWM_VOL_SEL. 0 for 3.3V and 1 for 5V output.

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of xxxxxx.apj firmware files with any ArduPilot
compatible ground station.
Firmware for these boards can be found [here](https://firmware.ardupilot.org) in  sub-folders labeled "ZeroOneX9".

## Where to Buy

[ZeroOne X9 product page](https://01aero.com/product/x9/)
