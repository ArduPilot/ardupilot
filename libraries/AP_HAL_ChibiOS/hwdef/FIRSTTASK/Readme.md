# FIRSTTASK Flight Controller (STM32H743)

## Overview
FIRSTTASK is an STM32H743-based flight controller derived from the Holybro Pixhawk 6C hardware.  
It includes the same core MCU, similar peripheral routing, and a custom HWDEF implementation.  
This board is intended for internal development and testing only.

---

## MCU / System
- **MCU:** STM32H743VGT6 (480MHz, 2MB Flash, 512KB RAM)
- **Crystal:** 16 MHz  
- **Boot method:** PX4-compatible ALT RAM MAP enabled  
- **Flash reserved:** 128 KB (bootloader)

---

## Connectivity Summary

### ✔ USB
- Full-speed USB on **PA11/PA12**

---

### ✔ UART / Serial Ports
Based on `SERIAL_ORDER OTG1 UART7 UART5 USART1 UART8 USART2 USART3 OTG2`:

| SERIALx | Interface | Pins |
|--------|-----------|-------|
| SERIAL0 | USB (OTG1) | PA11/PA12 |
| SERIAL1 | UART7 | PE8 / PE7 / PE9 / PE10 |
| SERIAL2 | UART5 | PC12 / PD2 / PC8 / PC9 |
| SERIAL3 | USART1 | PB6 / PA10 |
| SERIAL4 | UART8 | PE1 / PE0 |
| SERIAL5 | USART2 | PD5 / PA3 |
| SERIAL6 | USART3 | PD8 / PD9 |
| SERIAL7 | USB (OTG2) | — |

USART6 is dedicated for **IOMCU** and not exposed as a normal serial port.

---

### ✔ I2C Buses
| Bus | Pins | Purpose |
|-----|-------|----------|
| I2C1 | PB7 / PB8 | GPS + MAG |
| I2C2 | PB10 / PB11 | GPS2 + MAG |
| I2C4 | PD12 / PD13 | Internal bus |

Internal bus mask is enabled.

---

### ✔ SPI Buses
| Bus | Pins | Devices |
|-----|-------|----------|
| SPI1 | PA5/PA6/PA7 | ICM42688, BMI055/BMI088 |
| SPI2 | PD3/PC2/PC3 | FRAM |

---

### ✔ PWM Outputs (8 channels)
| PWM | Timer/Channel | Pin |
|------|-------------------|------|
| 1 | TIM1_CH1 | PA8 |
| 2 | TIM1_CH2 | PE11 |
| 3 | TIM1_CH3 | PE13 |
| 4 | TIM1_CH4 | PE14 |
| 5 | TIM4_CH3 | PD14 |
| 6 | TIM4_CH4 | PD15 |
| 7 | TIM5_CH1 | PA0 |
| 8 | TIM5_CH2 | PA1 |

---

### ✔ CAN Buses
- CAN1: PD0 / PD1  
- CAN2: PB5 / PB13

---

### ✔ SD Card
- SDMMC2 interface (PD6, PD7, PB14, PB15, PB3, PB4)

---

### ✔ Power / ADC
- Battery voltage & current (2 sets)
- 5V sensing
- Over-current pins
- Power rail control (hipower/periph)

---

### ✔ IMUs / Sensors
- ICM42688 (SPI1)
- BMI055 or BMI088 (SPI1)
- MS5611 (two addresses)
- BMP388
- IST8310 compass

Mag heater offset and IMU heater parameters included.

---

### ✔ LEDs & Buzzer
- Red LED on PD10  
- Blue LED on PD11  
- Buzzer on PB0 (TIM3_CH3)

---

## Bootloader
A custom bootloader is generated from:

```
hwdef-bl.dat
```

This provides PX4-compatible board ID flashing and ABIN bootloader support.

---

## Defaults
The file:

```
defaults.parm
```

contains the recommended startup parameters such as serial mappings, safety settings, and PWM configuration.

---

## Building Firmware

### Configure:
```
./waf configure --board FIRSTTASK
```

### Build:
```
./waf build
```

### Build Bootloader:
```
./waf --bootloader --board FIRSTTASK
```

### Upload (if supported):
```
./waf --upload --board FIRSTTASK
```

---

## Notes
- This board is **not intended for public sale**.  
- Includes custom RAMTRON storage, IMU heater, dual-baro, and PX4-style bootloader support.  
- Readme is only for technical documentation required for ArduPilot integration.


