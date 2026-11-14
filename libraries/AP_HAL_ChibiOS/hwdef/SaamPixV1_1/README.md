# SaamPixV1_1

The SaamPixV1_1 is an STM32F427-based flight controller produced by
[Saam Drones](https://www.saamdrones.com/). It features dual SPI IMUs,
an onboard barometer, dual compasses, CAN, microSD logging, and
RAMTRON parameter storage. All 16 PWM outputs are driven directly by
the FMU (no separate IO co-processor), which suits VTOL and other
multi-actuator airframes. It supports multirotor, fixed-wing, VTOL,
and hybrid UAV applications.

![SaamPixV1_1 Board](SaamPixV1_1-board.jpg)

## Features/Specifications

- MCU: STM32F427xx ARM Cortex-M4 with FPU at 168 MHz
- Flash: 2048 KB internal
- SRAM: 256 KB
- IMU 1: ICM20602
- IMU 2: MPU9250 (with onboard AK8963 compass)
- Barometer: MS5611
- Compass 1: LIS3MDL (external SPI)
- Compass 2: AK8963 (via MPU9250)
- RAMTRON parameter storage (16 KB)
- MicroSD card (SDIO interface)
- CAN bus: 1x CAN1
- UARTs: 6 (USART1, USART2, USART3, UART4, UART7, UART8)
  - USB OTG
- I2C: 2 buses (I2C1, I2C2)
- PWM Outputs: 16 channels
- RC Input: PPM, SBUS, DSM/DSM2/DSMX via dedicated pin or
  UART
- USB: micro-USB (OTG1)
- Safety switch: not present

## Where to Buy

Available from [Saam Drones](https://www.saamdrones.com/).

## Pinout

![SaamPixV1_1 Pinout](SaamPixV1_1-pinout.png)

![SaamPixV1_1 Top View](SaamPixV1_1-top.png)

## UART Mapping

| Name | Pin | Function | Notes |
| --- | --- | --- | --- |
| SERIAL0 | USB OTG1 | USB Console | |
| SERIAL1 | USART2 | Telemetry 1/MAVLink2 | CTS/RTS on PD3/PD4 |
| SERIAL2 | USART3 | MAVLink2 (Companion computer, USB) | Onboard CP2104 USB bridge |
| SERIAL3 | UART4 | GPS 1 | |
| SERIAL4 | UART8 | GPS 2 (default) | |
| SERIAL5 | USART1 | User | |
| SERIAL6 | UART7 | Debug UART | |

Note: SERIAL2 (USART3) is wired to an onboard CP2104 USB-to-UART
bridge exposed on the secondary USB connector for companion-computer
connection, not to an external telemetry header. SERIAL0 (USB OTG)
is the primary USB console.

## CAN

The SaamPixV1_1 exposes one CAN interface:

- CAN1 RX: PD0
- CAN1 TX: PD1

## I2C

The external I2C connector is on the I2C2 bus (SCL on PB10, SDA on
PB11), shared with the GPS connector's I2C pins.

## PWM Output

The SaamPixV1_1 has 16 PWM outputs. All outputs support PWM and
DShot protocols.

| Outputs | Timer | Notes |
| --- | --- | --- |
| 1-4 | TIM1 | GPIO(50-53) |
| 5-8 | TIM4 | GPIO(54-57) |
| 9-12 | TIM3 | GPIO(58-61) |
| 13-16 | TIM2 | GPIO(62-65) |

Note: All outputs within the same timer group must use the same
protocol.

## RC Input

RC input is configured on the RC/PPM input by default and supports
all ArduPilot compatible unidirectional protocols, including SBUS,
which is connected to the RC/PPM input. Bi-directional protocols, such
as CRSF/ELRS/FPort/SRXL2, need to utilize a full UART. See
[RC systems](https://ardupilot.org/copter/docs/common-rc-systems.html)
for more information.

## RSSI

RSSI is available as a digital input on GPIO(66). To use it, set
`RSSI_TYPE` = 4 and `RSSI_ANA_PIN` = 66.

## GPIOs

| GPIO | Function |
| --- | --- |
| GPIO(0) | RUN_LED |
| GPIO(32) | ALARM (TIM14) |
| GPIO(50) | Servo1 |
| GPIO(51) | Servo2 |
| GPIO(52) | Servo3 |
| GPIO(53) | Servo4 |
| GPIO(54) | Servo5 |
| GPIO(55) | Servo6 |
| GPIO(56) | Servo7 |
| GPIO(57) | Servo8 |
| GPIO(58) | Servo9 |
| GPIO(59) | Servo10 |
| GPIO(60) | Servo11 |
| GPIO(61) | Servo12 |
| GPIO(62) | Servo13 |
| GPIO(63) | Servo14 |
| GPIO(64) | Servo15 |
| GPIO(65) | Servo16 |
| GPIO(66) | RSSI digital input |

## Battery Monitor

The board has an internal voltage and current sensor. The default
battery parameters are:

- `BATT_VOLT_PIN` = 12
- `BATT_CURR_PIN` = 10
- `BATT_VOLT_MULT` = 10.1
- `BATT_AMP_PERVLT` = 17.0

These are set in the hwdef. Adjust `BATT_AMP_PERVLT` to match
your current sensor if using an external one.

## Compass

The SaamPixV1_1 has two built-in compasses: LIS3MDL and AK8963
(via MPU9250).

Due to potential interference from the power supply and PWM
circuits, disabling this and using an external I2C compass as part of a GPS/Compass
module is recommended for best performance.

## Firmware

Firmware for the SaamPixV1_1 can be found at
[firmware.ardupilot.org](https://firmware.ardupilot.org)
in sub-folders labeled "SaamPixV1_1".

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible
bootloader, allowing the loading of `*.apj` firmware files with
any ArduPilot compatible ground station (Mission Planner,
QGroundControl, MAVProxy).

For first-time firmware installation via DFU or STLink, use the
`*_with_bl.hex` file. See
[Loading Firmware onto ChibiOS boards](https://ardupilot.org/copter/docs/common-loading-firmware-onto-chibios-only-boards.html)
for instructions.
