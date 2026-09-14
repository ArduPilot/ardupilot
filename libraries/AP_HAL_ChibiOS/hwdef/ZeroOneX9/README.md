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

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
| Name    | Function | MCU PINS |   DMA   |
| :-----: | :------: | :------: | :------:|
| SERIAL0 | OTG1     | USB      |
| SERIAL1 | Telem1   | UART7    |DMA Enabled |
| SERIAL2 | Telem2   | UART5    |DMA Enabled |
| SERIAL3 | GPS1     | USART1   |DMA Enabled |
| SERIAL4 | GPS2     | UART8    |DMA Enabled |
| SERIAL5 | Telem3   | USART2   |DMA Enabled |
| SERIAL6 | UART4    | UART4    |DMA Enabled |
| SERIAL7 |FMU DEBUG | USART3   |DMA Enabled |
| SERIAL8 | OTG-SLCAN| USB      |

SERIAL3 is configured for a DroneCAN GPS by default (`GPS1_TYPE=9`). Set
`GPS1_TYPE` to the appropriate serial GPS type when using the GPS1 USART port.

## RC Input

The SBUS pin, can be used for all ArduPilot supported receiver protocols, except CRSF/ELRS and SRXL2 which require a true UART connection. However, FPort, when connected in this manner, will only provide RC without telemetry.
To allow CRSF and embedded telemetry available in Fport, CRSF, and SRXL2 receivers, a full UART, such as SERIAL6 (UART4) would need to be used for receiver connections. Below are setups using Serial6.

- [SERIAL6_PROTOCOL](https://ardupilot.org/copter/docs/parameters.html#serial6-protocol-serial6-protocol-selection) should be set to "23".
- FPort would require [SERIAL6_OPTIONS](https://ardupilot.org/copter/docs/parameters.html#serial6-options-serial6-options) be set to "15".
- CRSF/ELRS would require [SERIAL6_OPTIONS](https://ardupilot.org/copter/docs/parameters.html#serial6-options-serial6-options) be set to "0".
- SRXL2 would require [SERIAL6_OPTIONS](https://ardupilot.org/copter/docs/parameters.html#serial6-options-serial6-options) be set to "4" and connects only the TX pin.

Any UART can be used for RC system connections in ArduPilot also, and is compatible with all protocols except PPM. See [RC systems](https://ardupilot.org/copter/docs/common-rc-systems.html) for details.

## PWM Output

The X9 flight controller supports up to 16 PWM outputs.
First 8 outputs (labelled 1 to 8) are controlled by a dedicated STM32F103 IO controller.
The remaining 8 outputs (labelled 9 to 16) are the "auxiliary" outputs. These are directly attached to the STM32H753 FMU controller .
All 16 outputs support normal PWM output formats. All 16 outputs support DShot, except 15 and 16.

The 8 IO PWM outputs are in 3 groups:

- Outputs 1 and 2 in group1
- Outputs 3 and 4 in group2
- Outputs 5, 6, 7 and 8 in group3

The 8 FMU PWM outputs are in 3 groups:

- Outputs 1, 2, 3 and 4 in group1
- Outputs 5 and 6 in group2
- Outputs 7 and 8 in group3

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

## GPIOs

All PWM outputs can be used as GPIOs (relays, camera, RPM etc). To use them you need to set the output’s SERVOx_FUNCTION to -1. The numbering of the GPIOs for PIN variables in ArduPilot is:

| Pad | Output | GPIO |
| --- | --- | ---: |
| M1 | MainOut1 | 101 |
| M2 | MainOut2 | 102 |
| M3 | MainOut3 | 103 |
| M4 | MainOut4 | 104 |
| M5 | MainOut5 | 105 |
| M6 | MainOut6 | 106 |
| M7 | MainOut7 | 107 |
| M8 | MainOut8 | 108 |
| M9 | AuxOut1 | 50 |
| M10 | AuxOut2 | 51 |
| M11 | AuxOut3 | 52 |
| M12 | AuxOut4 | 53 |
| M13 | AuxOut5 | 54 |
| M14 | AuxOut6 | 55 |
| M15 | AuxOut7 | 56 |
| M16 | AuxOut8 | 57 |
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

The X9 flight controller built-in industrial-grade electronic compass chip RM3100.

## Analog Inputs

The X9 flight controller has 3 analog inputs.

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
