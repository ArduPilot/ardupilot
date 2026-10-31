# ZeroOneM9 Flight Controller

The ZeroOne M9 is a series of flight controllers designed and manufactured by ZeroOne.
Integrated with dual-redundant high-precision differential airspeed sensors, anti-interference RTK positioning module, and adopts 900MHz band data transmission link with a maximum communication range of 60km.
![M9Series](M9series.jpg)

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

       IMU1-SCH16T-K10(With vibration isolation and external FIFO)
       IMU2-ADIS16507(With vibration isolation)
       IMU3-IIM42653(No vibration isolation)

  - Baro:

       Three barometers(Version 1):  2 x ICP20100 + AUAV L30D
       Three barometers(Version 2):  BMP581+SPL07 + AUAV L30D

  - Magnetometer:

       Builtin RM3100 magnetometer(SPI)

  - Airspeed

      Two Airspeed Sensors: AUAV-L30D(I2C2) + AUAV-L30D(CAN1)

  - RTK

      Anti-interference RTK positioning module(CAN1)

  - Data Link

      900MHz band data transmission link with a maximum communication range of 60km(Telem1)

## Pinout

![ZeroOneM9 Pinout](ZeroOneM9Pinout.jpg)

## UART Mapping

The Cable Hub labels serial connectors by function. TX and RX are relative to the flight controller.

| Port    | UART   | Protocol | Connector      | DMA | Type  |
| ------- | ------ | -------- | -------------- | --- | ----- |
| SERIAL0 | USB    | MAVLink2 | Type-C         |     |       |
| SERIAL1 | UART7  | MAVLink2 | TELEM1-TTL, also internal 900MHz data link | Yes | TTL |
| SERIAL2 | UART5  | MAVLink2 | TELEM2-TTL     | Yes | TTL   |
| SERIAL3 | USART1 | GPS      | GPS1-TTL       | Yes | TTL   |
| SERIAL4 | UART8  | GPS      | GPS2-TTL       | Yes | TTL   |
| SERIAL5 | USART2 | MAVLink2 | TELEM3-RS232   | Yes | RS232 |
| SERIAL6 | UART4  | None     | UART4-RS422    | Yes | RS422 |
| SERIAL7 | USART3 | None     | FMU debug      | No  | TTL   |
| SERIAL8 | USB    | SLCAN    | Type-C         |     |       |

## RC Input

The SBUS pin, can be used for all ArduPilot supported receiver protocols, except CRSF/ELRS and SRXL2 which require a true UART connection. However, FPort, when connected in this manner, will only provide RC without telemetry.
For CRSF/ELRS, FPort telemetry, or SRXL2, use a full TTL UART such as TELEM1, TELEM2, GPS1, or GPS2. TELEM3 is RS232 and UART4 is RS422, so neither is electrically compatible with a TTL receiver.

Any UART can be used for RC system connections in ArduPilot also, and is compatible with all protocols except PPM. See [RC systems](https://ardupilot.org/copter/docs/common-rc-systems.html) for details.

## Safety Switch

The M9 does not expose an external safety switch connection. The safety switch
requirement is disabled by default (`BRD_SAFETY_DEFLT=0`).
If an existing configuration has `BRD_SAFETY_DEFLT=1`, set it to 0.

## PWM Output

The M9 flight controller supports up to 16 PWM outputs.
M1-M8 are controlled by a dedicated STM32F103 IO controller.
A1-A8 are the "auxiliary" outputs, directly attached to the STM32H753 FMU controller.
All 16 outputs support normal PWM output formats. All outputs support DShot, except A7 and A8.
M1-M4 support bi-directional DShot. M7 and M8 also support it when M5 and M6 are not configured for bi-directional DShot.

The 8 IO PWM outputs are in 3 groups:

- M1 and M2 in group1
- M3 and M4 in group2
- M5, M6, M7 and M8 in group3

The 8 FMU PWM outputs are in 3 groups:

- A1, A2, A3 and A4 in group1 (TIM5)
- A5 and A6 in group2 (TIM4)
- A7 and A8 in group3 (TIM12)

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

## GPIOs

All PWM outputs can be used as GPIOs (relays, camera, RPM etc). To use them you need to set the output’s SERVOx_FUNCTION to -1. The numbering of the GPIOs for PIN variables in ArduPilot is:

| IO Pins |       |          | FMU Pins |       |         |
| ---     | ---   | ---      | ---      | ---   | ---     |
| Name    | Value | Option   | Name     | Value | Option  |
| M1      | 101   | MainOut1 | A1       | 50    | AuxOut1 |
| M2      | 102   | MainOut2 | A2       | 51    | AuxOut2 |
| M3      | 103   | MainOut3 | A3       | 52    | AuxOut3 |
| M4      | 104   | MainOut4 | A4       | 53    | AuxOut4 |
| M5      | 105   | MainOut5 | A5       | 54    | AuxOut5 |
| M6      | 106   | MainOut6 | A6       | 55    | AuxOut6 |
| M7      | 107   | MainOut7 | A7       | 56    | AuxOut7 |
| M8      | 108   | MainOut8 | A8       | 57    | AuxOut8 |

## 5V PWM Voltage

The M9 flight controller supports switching between 5V and 3.3V PWM levels. Switch PWM output pulse level by configuring parameter BRD_PWM_VOLT_SEL. 0 for 3.3V and 1 for 5V output.

## Compass

The M9 has a built-in RM3100 compass. Due to potential interference, the autopilot is usually used with an external DroneCAN compass as part of a GPS/Compass combination, connected to the CAN connector.

## Airspeed

To configure the two AUAV-L30D airspeed sensors, set the following parameters and reboot:

| Parameter | Value | Function |
| --------- | ----- | -------- |
| ARSPD_TYPE | 19 | AUAV 30in airspeed sensor |
| ARSPD_BUS | 2 | I2C bus for the first airspeed sensor |
| ARSPD2_TYPE | 8 | DroneCAN airspeed sensor |

## Ethernet

The Cable Hub provides an Ethernet connector connected to the LAN8742A RMII interface. Ethernet power is enabled when the board starts. Set NET_ENABLE to 1 and reboot the board to enable networking.

## SBUS Output

The SBUS-OUT connector is attached to the IO processor. Set BRD_SBUS_OUT to enable servo-channel output over SBUS.

## Analog Inputs

The M9 flight controller has 2 analog inputs.

- ADC Pin12 -> ADC 6.6V Sense
- ADC Pin13 -> ADC 3.3V Sense

## Battery Monitoring

The M9 flight controller has two four-pin global power connectors, supporting CAN interface power supply.
The BATT_MONITOR default is 8, so a DroneCAN-capable power module is required.

The M9 flight controller has a XT60 FC power connector, supporting analog voltage detection.
This is monitored by the internal CAN peripheral "ZeroOneM9_Periph".

The integrated DroneCAN RTK receiver is the primary GPS by default (GPS1_TYPE is 9). Change GPS1_TYPE before using a serial GPS as the primary receiver on GPS1-TTL.

## Power Input

| Name    | Function | Voltage range |
| :-----: | :------: | :------: |
| Power1 | Global Power(Apart from Datalink) | 5.0-5.3V |
| Power2 | Global Power(Apart from Datalink) | 5.0-5.3V |
| FC Power | FC Power | 16-36V |
| Servo Power | Servo Power| 0-9.9V |

## Log Download

The M9 flight controller has a built-in independent log card reader, which can be connected via the USB Drive interface and recognized as a storage disk (do not connect other power interfaces when connecting the USB Drive interface).

## Dimensions and Weight

- Flight controller: 128mm x 78mm x 46mm, 585g
- Cable Hub: 133mm x 53.5mm x 22mm, 238g
- Combined weight: 823g
- External interfaces: 6 UARTs, dual CAN, Ethernet, I2C, ADC, USB Type-C, USB Drive, SBUS input/output, and 16 PWM outputs

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of xxxxxx.apj firmware files with any ArduPilot
compatible ground station.
Firmware for these boards can be found [here](https://firmware.ardupilot.org) in  sub-folders labeled "ZeroOneM9".

## Where to Buy

[ZeroOne M9 product page](https://01aero.com/product/m9/)
