# DroneerX6 Flight Controller

The DroneerX6 is a series of flight controllers manufactured by Droneer, which is based on the open-source FMU v6X architecture and Pixhawk Autopilot Bus open source specifications.
![DroneerX6](https://github.com/Droneer-uav/Droneerx6/blob/main/DroneerX6_V1.png "DroneerX6")

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

    - **X6**:

       IMU1-ICM45686(With vibration isolation)
       IMU2-ICM45686(With vibration isolation)
       IMU3-ICM45686(No vibration isolation)

  - Baro:

       Two barometers:  2 x ICP20100

  - Magnetometer:

       Builtin RM3100 magnetometer

## Pinout

![DroneerX6 Pinout](https://github.com/Droneer-uav/Droneerx6/blob/main/DroneerX6_Pinout.png  "DroneerX6")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
| Name    | LABEL  | UART/Default Protocol |   DMA   |
| :-----: | :------: | :------: | :------:|
| SERIAL0 | USB      | VCP/MAVLink2| |
| SERIAL1 | Telem1   | UART7/MAVLink2    |DMA Enabled |
| SERIAL2 | Telem2   | UART5/MAVLink2    |DMA Enabled |
| SERIAL3 | GPS1     | USART1   |DMA Enabled |
| SERIAL4 | GPS2     | UART8    |DMA Enabled |
| SERIAL5 | Telem3   | USART2/MAVLink2   |DMA Enabled |
| SERIAL6 | UART4    | UART4    |DMA Enabled |
| SERIAL7 | FMU DEBUG| USART3   |DMA Enabled |
| SERIAL8 | OTG-SLCAN| USB      | |

## RC Input

The SBUS RC pin, can be used for all ArduPilot supported unidirectional receiver protocols. Bi-directional protocols, such as CRSF/ELRS and SRXL2, which require a true UART connection.If  FPort, is connected to SBUS RC, RC will work, but not telemetry.
To allow CRSF and embedded telemetry available in Fport, CRSF, and SRXL2 receivers, a full UART, such as SERIAL6 (UART4) would need to be used for receiver connections. Below are setups using Serial6.

- `SERIAL6_PROTOCOL` should be set to "23".
- FPort would require `SERIAL6_OPTIONS` be set to "15".
- SRXL2 would require `SERIAL6_OPTIONS` to be set to "4" (half-duplex) and connects only the TX pin.

Any UART can be used for RC system connections in ArduPilot also, and is compatible with all protocols except PPM. See [RC systems](https://ardupilot.org/copter/docs/common-rc-systems.html) for details.

## PWM Output

The X6 flight controller supports up to 16 PWM outputs.
First 8 outputs (labelled MAIN1 to MAIN8) are controlled by a dedicated STM32F103 IO controller.
The remaining 8 outputs (labelled AUX1 to AUX6, and TIM12_CH1 and TIM12_CH2) are the "auxiliary" outputs. These are directly attached to the STM32H753 FMU controller . Note that AUX1 is associated with `SERVO9_x` parameters, AUX2 with `SERVO10_x`, etc.  While TIM12_CH1 is really PWM output 15 and is associated with `SERVO15_x` params, and TIM12_CH2 is `SERVO16_` params.
All 16 outputs support normal PWM output formats. All 16 outputs support DShot, except 15 and 16.

The 8 MAIN PWM outputs are in 4 groups:

- Outputs 1 and 2 in group1
- Outputs 3 and 4 in group2
- Outputs 5, 6, 7 and 8 in group3

The 8 AUX PWM outputs are in 4 groups:

- Outputs AUX1-4 in group1
- Outputs AUX5-6 in group2
- Outputs 7 and 8 in group3 (TIM12_CH1/2 has no DMA, so only can be PWM output protocols)

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

## GPIOs

All PWM outputs can be used as GPIOs (relays, camera, RPM etc). To use them you need to set the output’s SERVOx_FUNCTION to -1. The numbering of the GPIOs for PIN variables in ArduPilot is:

| IO Pins |  | FMU Pins |
| --- | --- | --- |
| Name | Value | Option |
| M1 | 101 | MainOut1 | 
| M2 | 102 | MainOut2 |
| M3 | 103 | MainOut3 |
| M4 | 104 | MainOut4 |
| M5 | 105 | MainOut5 |
| M6 | 106 | MainOut6 |
| M7 | 107 | MainOut7 |
| M8 | 108 | MainOut8 |
| M9 | 50 | AuxOut1 | 
| M10 | 51 | AuxOut2 |
| M11 | 52 |AuxOut3 | 
| M12 | 53 | AuxOut4 |
| M13 | 54 | AuxOut5 |
| M14 | 55 | AuxOut6 |
| M15 | 56 | TIM12_CH1 |
| M16 | 57 | TIM12_CH2 |
| FMU_CAP1 | 58 | FCU CAP|
| NFC_GPIO | 60 ||

## Battery Monitoring

The X6 flight controller has two six-pin power connectors, supporting DroneCAN power modules.
These are set by default in the firmware and shouldn't need to be adjusted.

## Compass

The X6 flight controller built-in industrial-grade electronic compass chip RM3100. Users often have to disable the internal compass and use an external , remotely located compass to avoid power system interference.

## Analog Inputs

The X6 flight controller has 2 analog inputs.

- Analog Pin12 -> ADC 6.6V Sense
- Analog Pin13 -> ADC 3.3V Sense
- RSSI analog input pin = 103

## 5V PWM Voltage

The X6 flight controller supports switching between 5V and 3.3V PWM levels. Switch PWM output pulse level by configuring parameter BRD_PWM_VOL_SEL. 0 for 3.3V(default) and 1 for 5V output.

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of xxxxxx.apj firmware files with any ArduPilot
compatible ground station.
Firmware for these boards can be found [here](https://firmware.ardupilot.org) in  sub-folders labeled "DroneerX6".

## Where to Buy

[Droneer](https://www.droneer.com)
