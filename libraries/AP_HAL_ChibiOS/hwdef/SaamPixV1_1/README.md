# SaamPixV1_1

SaamPixV1_1 is an STM32F427-based flight controller with dual SPI IMUs, onboard barometer and compasses, CAN, SD card, and RAMTRON parameter storage. Public Saam Drones product material describes the board as a high-performance open-source controller for multirotor, fixed-wing, VTOL, and hybrid UAV applications.

## Overview

- MCU: STM32F427xx
- CPU: ARM Cortex-M4 with FPU at 168 MHz
- Flash: 2048 KB
- SRAM: 256 KB
- Bootloader reserve: 16 KB
- Oscillator: 8 MHz
- Board ID token: `AP_HW_SAAMPIXV1_1`
- APJ board name: `SaamPixV1_1`
- USB manufacturer string: `Saam Drones`
- USB product string: `SaamPixV1_1 Flight Controller`
- Storage: RAMTRON (`HAL_WITH_RAMTRON 1`)
- Filesystem: FATFS enabled
- Safety switch: disabled by default (`HAL_HAVE_SAFETY_SWITCH 0`)

## Sensors

- IMU 1: ICM20602 on SPI4, rotation `ROLL_180_YAW_90`
- IMU 2: MPU9250 on SPI4, rotation `ROLL_180_YAW_90`
- Barometer: MS5611 on SPI4
- Compass 1: LIS3MDL on SPI4
- Compass 2: AK8963 via MPU9250

## Bus Layout

- I2C buses: `I2C1`, `I2C2`
- SPI buses:
  - `SPI1`: RAMTRON
  - `SPI4`: main sensors
- CAN: `CAN1`
- SD card: SDIO interface present

## Serial Port Mapping

Serial order is:

`OTG1 USART2 USART3 UART4 UART8 USART1 UART7 USART6`

- `SERIAL0`: USB OTG1
- `SERIAL1`: USART2, Telemetry 1, CTS/RTS on PD3/PD4
- `SERIAL2`: USART3, Telemetry 2
- `SERIAL3`: UART4, GPS
- `SERIAL4`: UART8, FrSky telemetry
- `SERIAL5`: USART1, SBUS
- `SERIAL6`: UART7, debug UART
- `SERIAL7`: USART6, alternate RC input path

## RC Input and PWM Outputs

- Default RC input: TIM8 on PC6
- Alternate RC input: USART6 on PC6
- Public product description: 16 PWM-capable channels, arranged as 8 main and 8 auxiliary outputs
- PWM outputs:
  - Outputs 1-4: TIM1 on PE9, PE11, PE13, PE14
  - Outputs 5-8: TIM4 on PD12, PD13, PD14, PD15
  - Outputs 9-12: TIM3 on PB4, PC7, PB0, PB1
  - Outputs 13-16: TIM2 on PA15, PB3, PA2, PA3

Receiver compatibility called out in public product information:

- PPM
- SBUS
- DSM / DSM2 / DSMX
- FrSky D / S.Port telemetry

## Analog and Power Inputs

- Battery current sense: PC0, ADC1
- Battery voltage sense: PC2, ADC1
- 5V sensor rail monitor: PA4, ADC1
- Additional ADC inputs: PC3, PC4, PC5

Default battery configuration from `hwdef.dat`:

- Voltage pin: 12
- Current pin: 10
- Voltage scale: 10.1
- Current scale: 17.0

## LEDs and Misc GPIO

- Run LED: PA8
- Tone alarm: PA7 using TIM14
- USB VBUS detect: PA9
- SBUS inverter control: PA10

## Build

From repository root:

```bash
./waf configure --board SaamPixV1_1
./waf copter
```

Common targets:

```bash
./waf plane
./waf rover
```

Output artifacts are generated under `build/SaamPixV1_1/bin/`.

## Intended Usage

Public Saam Drones material describes the board as supporting:

- multirotor aircraft
- fixed-wing aircraft
- VTOL configurations
- hybrid UAV platforms

It also advertises support for ArduPilot-compatible workflows using Mission Planner and QGroundControl.

## Bootloader

Build the dedicated bootloader with:

```bash
Tools/scripts/build_bootloaders.py SaamPixV1_1
```

Generated bootloader artifacts are placed in `Tools/bootloaders/`.

## Flashing

To build and upload directly:

```bash
./waf configure --board SaamPixV1_1
./waf copter --upload
```

For manual flashing or first-time programming, use the generated bootloader image and the vehicle firmware `*_with_bl.hex` file.

## Default Parameters

Board defaults include:

- `BRD_SAFETY_DEFLT = 0`
- `BRD_BOOT_DELAY = 500`
- `BATT_MONITOR = 3`
- `BATT_LOW_VOLT = 21.6`
- `RTL_ALT = 5000`
- `AHRS_ORIENTATION = 8`

See `defaults.parm` for the full default parameter set.

## Notes

- External SPI radio pins are present in the hwdef but currently disabled.
- The board uses RAMTRON for parameter persistence.
- Public Saam Drones material references micro-USB connection, TELEM1/TELEM2/UART expansion, GPS port usage, CAN, I2C, and microSD logging.
- If this file is submitted upstream, it should be augmented with annotated board photos and connector pinout images.
