# SaamPixV1_1

The SaamPixV1_1 is an STM32F427-based flight controller produced by [Saam Drones](https://www.saamdrones.com/). It features dual SPI IMUs, an onboard barometer, dual compasses, CAN, microSD logging, and RAMTRON parameter storage. It supports multirotor, fixed-wing, VTOL, and hybrid UAV applications.

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
- UARTs: 7 (USART1, USART2, USART3, UART4, UART7, UART8, USART6) + USB OTG
- I2C: 2 buses (I2C1, I2C2)
- PWM Outputs: 16 channels
- RC Input: PPM, SBUS, DSM/DSM2/DSMX via dedicated pin or UART
- USB: micro-USB (OTG1)
- Safety switch: not present

## Where to Buy

Available from [Saam Drones](https://www.saamdrones.com/).

## Pinout

![SaamPixV1_1 Pinout](SaamPixV1_1-pinout.png)

![SaamPixV1_1 Top View](SaamPixV1_1-top.png)

## UART Mapping

| Name | Pin | Function | Notes |
| ------ | ----- | ---------- | ------- |
| SERIAL0 | USB OTG1 | USB Console | |
| SERIAL1 | USART2 | Telemetry 1 | CTS/RTS on PD3/PD4 |
| SERIAL2 | USART3 | Telemetry 2 | |
| SERIAL3 | UART4 | GPS 1 | |
| SERIAL4 | UART8 | GPS 2 (default) | TX invertible via GPIO(78) |
| SERIAL5 | USART1 | SBUS input | |
| SERIAL6 | UART7 | Debug UART | |
| SERIAL7 | USART6 | RC Input alt | ALT config only |

## CAN

The SaamPixV1_1 exposes one CAN interface:

- CAN1 RX: PD0
- CAN1 TX: PD1

## PWM Output

The SaamPixV1_1 has 16 PWM outputs. All outputs support PWM and DShot protocols.

| Outputs | Timer | Pins | Notes |
| --------- | ------- | ------ | ------- |
| 1-4 | TIM1 | PE9, PE11, PE13, PE14 | |
| 5-8 | TIM4 | PD12, PD13, PD14, PD15 | |
| 9-12 | TIM3 | PB4, PC7, PB0, PB1 | GPIO(50-53) |
| 13-16 | TIM2 | PA15, PB3, PA2, PA3 | GPIO(54-55) on 13-14 |

Note: All outputs within the same timer group must use the same protocol.

## RC Input

RC input is configured on PC6 by default using TIM8_CH1 and supports the following unidirectional protocols: PPM, SBUS, DSM, DSM2, DSMX, IBUS, SUM-D.

For SBUS, connect to SERIAL5 (USART1 RX on PB7). An SBUS signal inverter is available on PA10.

For bidirectional protocols (CRSF/ELRS, SRXL2, FPort), connect to any free UART and set the corresponding `SERIALn_PROTOCOL`. See [RC Systems](https://ardupilot.org/copter/docs/common-rc-systems.html) for setup details.

An alternate RC input path via USART6 on PC6 is available using ALT config 1.

## GPIOs

| GPIO | Pin | Function |
| ------ | ----- | ---------- |
| GPIO(0) | PA8 | RUN_LED |
| GPIO(32) | PA7 | ALARM (TIM14) |
| GPIO(50) | PB4 | Servo9 |
| GPIO(51) | PC7 | Servo10 |
| GPIO(52) | PB0 | Servo11 |
| GPIO(53) | PB1 | Servo12 |
| GPIO(54) | PA15 | Servo13 |
| GPIO(55) | PB3 | Servo14 |
| GPIO(78) | PB12 | UART8 TX invert |

## RSSI and Analog Pins

| Pin | GPIO | Function |
| ----- | ------ | ---------- |
| PC1 | FMU_GPIO3 | RSSI digital input |
| PC3 | - | ADC1 (FMU_ADC1) |
| PC4 | - | ADC2 (FMU_ADC2) |
| PC5 | - | ADC3 (FMU_ADC3) |
| PA4 | - | 5V sensor rail monitor (SCALE 2) |

## Battery Monitor

The board has an internal voltage and current sensor. The default battery parameters are:

- `BATT_VOLT_PIN` = 12 (PC2)
- `BATT_CURR_PIN` = 10 (PC0)
- `BATT_VOLT_MULT` = 10.1
- `BATT_AMP_PERVLT` = 17.0

These are set in the hwdef. Adjust `BATT_AMP_PERVLT` to match your current sensor if using an external one.

## Compass

The SaamPixV1_1 has two built-in compasses:

- LIS3MDL on SPI4 (ROTATION_NONE)
- AK8963 via MPU9250 on SPI4 (ROTATION_ROLL_180_YAW_90)

Due to potential interference from the power supply and PWM circuits, an external I2C compass as part of a GPS/Compass module is recommended for best performance.

## Firmware

Firmware for the SaamPixV1_1 can be found at [firmware.ardupilot.org](https://firmware.ardupilot.org) in sub-folders labeled "SaamPixV1_1".

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of `*.apj` firmware files with any ArduPilot compatible ground station (Mission Planner, QGroundControl, MAVProxy).

For first-time firmware installation via DFU or STLink, use the `*_with_bl.hex` file. See [Loading Firmware onto ChibiOS boards](https://ardupilot.org/copter/docs/common-loading-firmware-onto-chibios-only-boards.html) for instructions.
