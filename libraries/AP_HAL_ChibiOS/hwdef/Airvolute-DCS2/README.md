# Onboard FMU on Airvolute DCS2.Pilot board

DroneCore 2.0 is a modular AI-driven open architecture autopilot designed for complex use cases that combines high computational processing power, redundant connectivity, small size, and low weight.The autopilot represents a one-stop-solution for developers integrating the functionality of carrier board, companion computer, and power distribution board into a single compact form factor.
This system usually uses a "CUBE" autopilot as its primary FMU, but can use an onboard STM32H743 as the FMU. This board definition and firmware on `the ArduPilot firmware server <https://firmware.ardupilot.org>`__ is for this secondary FMU
For more information on DCS2.Pilot board see:
[Airvolute documentation](https://docs.airvolute.com/dronecore-autopilot/dcs2)

## Where To Buy

info@airvolute.com

## Features

- MCU: STM32H743
- IMU: BMI088
- Barometer: BMP390
- 2 UARTS
- 2 CAN buses
- 4 PWM outputs
- PPM (RC input)
- external SPI and I2C
- SD card connector
- USB connection onboard with Jetson Host
- Ethernet

## DCS2.Pilot peripherals diagram

![DC2 Pilot peripherals](https://github.com/vrsanskytom/ardupilot/blob/hwdef_for_airvolute_dcs2/libraries/AP_HAL_ChibiOS/hwdef/Airvolute-DCS2/DC2.Pilot%20peripherals.png)

## DCS2.Pilot onboard FMU related connectors pinout

### Top side

![DCS2 Pilot Top Side](https://github.com/vrsanskytom/ardupilot/blob/hwdef_for_airvolute_dcs2/libraries/AP_HAL_ChibiOS/hwdef/Airvolute-DCS2/DCS2.Pilot_TopSide.png)

#### PPM connector (RC input)

JST GH 1.25mm pitch, 3-Pin

Matching connector JST GHR-03V-S.

RC input is configured on the PPM_SBUS_PROT pin as part of the PPM connector. Pin is connected to UART3_RX and also to analog input on TIM3_CH1. This pin supports all unidirectional RC protocols, but for it to be enabled, it is necessary to set SERIAL3_PROTOCOL as RCIN. Also RC input is shared with primary FMU, so it is default disabled on this secondary FMU.

5V supply is limited to 1A by internal current limiter.
| Pin | Signal |
| --- | --- |
| 1 | GND |
| 2 | 5V |
| 3 | PPM |

### Bottom side

![DCS2 Pilot Bottom Side](https://github.com/vrsanskytom/ardupilot/blob/hwdef_for_airvolute_dcs2/libraries/AP_HAL_ChibiOS/hwdef/Airvolute-DCS2/DCS2.Pilot_BottomSide.png)

#### FMU SEC. connector

JST GH 1.25mm pitch, 12-Pin

Matching connector JST GHR-12V-S.

The DCS2 Onboard FMU supports up to 4 PWM outputs. These are directly attached to the STM32H743 and support all PWM protocols as well as DShot and bi-directional DShot.
The 4 PWM outputs are in 2 groups:
PWM 1,2 in group1
PWM 3,4 in group2
Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

5V supply is limited to 1A by internal current limiter.
| Pin | Signal |
| --- | --- |
| 1 | GND |
| 2 | GND |
| 3 | GPIO/PWM output 4 |
| 4 | GPIO/PWM output 3 |
| 5 | GPIO/PWM output 2 |
| 6 | GPIO/PWM output 1 |
| 7 | Serial 1 RX |
| 8 | Serial 1 TX |
| 9 | Serial 2 RX |
| 10 | Serial 2 TX |
| 11 | 5V |
| 12 | 5V |

#### EXT. SENS. connector

   BM23PF0.8-10DS-0.35V connector

   Matching connector BM23PF0.8-10DP-0.35V

   This connector allows connecting external IMU with I2C and SPI data buses.

   5V supply is limited to 1.9A by internal current limiter.
| Pin | Signal |
| --- | --- |
| 1 | SPI_MOSI |
| 2 | SPI_MISO |
| 3 | SPI_SCK |
| 4 | SPI_CS0 |
| 5 | SPI_CS1 |
| 6 | SPI_CS2 |
| 7 | SPI_CS3 |
| 8 | IMU_DRDY_EXT |
| 9 | I2C_SE_SDA |
| 10 | I2C_SE_SCL |
| MP1 | 5V |
| MP2 | 5V |
| MP3 | GND |
| MP4 | GND |

#### ETH EXP. connector

   505110-1692 connector

   Ethernet connector is routed to FMU through onboard switch.

   The onboard FMU is connected via the RMII bus with a speed of 100 Mbits.

#### SD card connector

   MEM2085-00-115-00-A connector

   Connector for standard microSD memory card. This card is primarily used to store flight data and logs.

## Other connectors

### CAN 1, CAN 2 connectors

The board contains two CAN buses - CAN1 and CAN 2. The buses support speeds up to 1 Mbits and in FD mode up to 8 Mbits.

These connectors are not part of DCS2.Pilot board, but they are routed on DCS2.Adapter_board. This board (DCS2.Adapter_board) is fully modular and can be modified according to the customer's requirements. For more information see the [Airvolute documentation](https://docs.airvolute.com/dronecore-autopilot/dcs2/adapter-extension-boards/dcs2.-adapter-default-v1.0/connectors-and-pinouts)

JST GH 1.25mm pitch, 4-Pin

Matching connector JST GHR-04V-S.

5V supply is limited to 1.9A by internal current limiter.
| Pin | Signal |
| --- | --- |
| 1 | 5V |
| 2 | CAN_H |
| 3 | CAN_L |
| 4 | GND |

## UART Mapping

- SERIAL0 -> USB (Default baud: 115200)
- SERIAL1 -> UART1 (FMU SEC) (Default baud: 57600, Default protocol: Mavlink2 (2))
- SERIAL2 -> UART2 (FMU SEC) (Default baud: 57600, Default protocol: Mavlink2 (2))
- SERIAL3 -> UART3 (RX pin only labeled as PPM on PPM connector) (Since this is a secondary FMU, default protocol is set to NONE instead of RCIN (23))

UARTs do not have RTS/CTS. UARTs 1 and 2 are routed to FMU_SEC. connector.

## Loading Firmware

Initial bootloader load is achievable only by SDW interface. Then it is possible to flash firmware through onboard USB connection with Jetson host.
