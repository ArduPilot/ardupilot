# CipherWing F4 (STM32F405)

## Introduction
The **CipherWing F4** is a custom autopilot board designed around the STM32F405 MCU.  
It is hardware-compatible with the OmnibusF4 family and supports multiple firmwares including **INAV**, **PX4**, and **ArduPilot** (tested with this PR).  
This board is under early development and testing by **Jeevesh Vishwavijay**, for educational and prototype use.

## Features / Specifications
- **Processor:** STM32F405RGT6 (168 MHz ARM Cortex-M4F)
- **Flash:** 1MB internal flash
- **IMUs:** MPU6000 (SPI1), BMI270 (SPI1, shared bus)
- **Barometer:** BMP280 (SPI3)
- **OSD:** MAX7456-compatible (SPI3)
- **Power:** VBAT & RSSI analog inputs
- **USB:** Micro-USB (OTG1)
- **UARTs:** Up to 6 hardware serial ports
- **PWM outputs:** 8 motor/servo outputs (M1–M8)
- **No onboard SD card**
- **Supported firmware:** INAV, PX4, ArduPilot

## Pinout
Follows the OmnibusF4 layout with modifications for CipherWing:
- **RSSI input:** PC1  
- **VBAT:** PC2  
- **PWM1–8:** PB0, PB1, PC8, PC9, PB14, PA8, PB15, PB7  
- **IMU:** SPI1  
- **OSD & BARO:** SPI3  

## UART Mapping
| Serial # | Function | Port | Notes |
|-----------|-----------|------|-------|
| SERIAL0 | USB | OTG1 | Default console |
| SERIAL1 | USART1 | PA9 / PA10 | Telemetry |
| SERIAL2 | USART2 | PA2 / PA3 | GPS1 |
| SERIAL3 | USART3 | PB10 / PB11 | GPS2 / Telem2 |
| SERIAL4 | UART4 | PA0 / PA1 | Optional |
| SERIAL5 | UART5 | PD2 (RX only) | Optional |
| SERIAL6 | USART6 | PC6 / PC7 | ESC telemetry |

## PWM Outputs
| PWM Channel | MCU Pin | Timer | Function |
|--------------|----------|--------|-----------|
| M1 | PB0 | TIM3_CH3 | Motor 1 |
| M2 | PB1 | TIM3_CH4 | Motor 2 |
| M3 | PC8 | TIM8_CH3 | Motor 3 |
| M4 | PC9 | TIM8_CH4 | Motor 4 |
| M5 | PB14 | TIM1_CH2N | Motor 5 |
| M6 | PA8 | TIM1_CH1 | Motor 6 |
| M7 | PB15 | TIM1_CH3N | Motor 7 |
| M8 | PB7 | TIM4_CH2 | Motor 8 |

## RC Input
Supports **SBUS**, **CRSF/ELRS**, and other unidirectional protocols on any available UART RX pin.  
Set appropriate `SERIALx_PROTOCOL` to `23` (CRSF) or `4` (SBUS).

## OSD Support
Onboard analog **MAX7456 OSD** supported (SPI3).  
Set `OSD_TYPE=1` for analog OSD, or `OSD_TYPE=5` for DisplayPort.

## Battery Monitor
Default analog monitor configured:
- Voltage: `PC2`  
- RSSI: `PC1`  
- Parameters:
  - `BATT_MONITOR = 4`
  - `BATT_VOLT_PIN = 12`
  - `BATT_VOLT_MULT = 11`
  - `BATT_CURR_PIN = -1` (disabled, no current sense)
  - `RSSI_ANA_PIN = PC1`

## Compass
No onboard compass.  
Use external I²C compass via SDA (PB9) and SCL (PB8).

## Firmware
Firmware binaries will be available under:  
`https://firmware.ardupilot.org/CipherWingF4/`

## Loading Firmware
This board supports ArduPilot `.apj` firmware upload via **Mission Planner** or **QGroundControl** once bootloader is flashed.  
To flash bootloader manually:
```bash
Tools/scripts/build_bootloaders.py CIPHERWINGF4
Notes
Designed for development and educational purposes.

Future versions may include SD card and CAN.

Board tested for compatibility with ArduCopter and ArduPlane builds.

Author: Jeevesh Vishwavijay
Project: CipherWing F4
