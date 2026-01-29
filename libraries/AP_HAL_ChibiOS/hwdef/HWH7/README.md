# HWH7 (HWH7-Basic)

## Introduction
HWH7 is an STM32H743-based flight controller target for ArduPilot (ChibiOS).  
This README is derived from the board hwdef files in this directory.

## Features/Specifications
- MCU: STM32H743xx (STM32H7xx family)
- Clock:
  - External crystal: 8 MHz
  - CPU clock: 480 MHz
- Flash: 2048 KB
- Bootloader / application layout:
  - Bootloader load address: 128 KB
  - Application reserves first 128 KB (firmware starts after bootloader region)
- Safety switch: disabled by default (BOARD_SAFETY_ENABLE_DEFAULT = 0)
- Default GPIO state: OUTPUT LOW with PULLDOWN (to reduce ESD risk)

## Where to Buy
TBD.

## USB
- USB OTG1 (FS):
  - PA11: OTG_FS_DM
  - PA12: OTG_FS_DP
- USB manufacturer string: "ArduPilot"

## Debug (SWD)
- PA13: SWDIO
- PA14: SWCLK

## Power / Battery Monitoring
Battery monitor defaults:
- Voltage sense:
  - Pin: PC0 (ADC1 INP10)
  - HAL_BATT_VOLT_PIN: 10
  - HAL_BATT_VOLT_SCALE: 11.0 (11:1 divider)
- Current sense:
  - Pin: PC1 (ADC1 INP11)
  - HAL_BATT_CURR_PIN: 11
  - HAL_BATT_CURR_SCALE: 5.882 A/V (170 mV/A => 1 / 0.17)

## LEDs
- Status LED:
  - PB2 (GPIO 62), named LED_RED in hwdef
  - Note: bootloader defines HAL_LED_ON = 0 (LED active level may be inverted)

## Buzzer
- PA1 (TIM5_CH2, GPIO 61)
- HAL_BUZZER_PIN = 61
- AP_NOTIFY_BUZZER_ENABLED = 1

## IMU (SPI)
Two SPI IMU buses are defined. Each bus is configured to support either a Bosch BMI270
or an InvenSense ICM-42688-class device (Invensensev3 driver), depending on hardware population.

### SPI1 (IMU1)
- SPI1:
  - PA5: SCK
  - PA6: MISO
  - PA7: MOSI
- IMU1 CS: PA4 (IMU1_CS)
- SPIDEV:
  - bmi270_0: SPI1 MODE3, 1 MHz init / 10 MHz max
  - icm42688_0: SPI1 MODE3, 2 MHz init / 16 MHz max
- IMU declarations:
  - BMI270 SPI:bmi270_0 ROTATION_YAW_270
  - Invensensev3 SPI:icm42688_0 ROTATION_NONE

### SPI2 (IMU2)
- SPI2:
  - PD3: SCK
  - PC2: MISO
  - PC3: MOSI
- IMU2 CS: PB12 (IMU2_CS)
- SPIDEV:
  - bmi270_1: SPI2 MODE3, 1 MHz init / 10 MHz max
  - icm42688_1: SPI2 MODE3, 2 MHz init / 16 MHz max
- IMU declarations:
  - BMI270 SPI:bmi270_1 ROTATION_YAW_180
  - Invensensev3 SPI:icm42688_1 ROTATION_YAW_270

Notes:
- HAL_DEFAULT_INS_FAST_SAMPLE = 3

## OSD (SPI3)
- MAX7456 (AT7456E compatible) on SPI3:
  - PB3: SPI3_SCK
  - PB4: SPI3_MISO
  - PD6: SPI3_MOSI
  - CS: PA15 (MAX7456_CS)
- OSD:
  - OSD_ENABLED = 1
  - HAL_OSD_TYPE_DEFAULT = 1
  - Fonts: ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

## I2C
Only one internal I2C bus is defined:
- I2C2:
  - PB10: SCL
  - PB11: SDA

### Barometer
A barometer is expected on I2C2 address 0x76 (one of the following, depending on population):
- SPL06 @ 0x76
- DPS310 @ 0x76

## Compass
- ALLOW_ARM_NO_COMPASS enabled
- External I2C compasses probing enabled:
  - HAL_PROBE_EXTERNAL_I2C_COMPASSES = 1
  - HAL_I2C_INTERNAL_MASK = 0
  - HAL_COMPASS_AUTO_ROT_DEFAULT = 2

## microSD (SDMMC1) / FATFS
- SDMMC1:
  - PC12: CK
  - PD2:  CMD
  - PC8:  D0
  - PC9:  D1
  - PC10: D2
  - PC11: D3
- FATFS:
  - FATFS_HAL_DEVICE = SDCD1
  - HAL_OS_FATFS_IO = 1

Bootloader supports flashing firmware from SD card:
- AP_BOOTLOADER_FLASH_FROM_SD_ENABLED = 1

## UART Mapping
SERIAL_ORDER is:
OTG1, USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8

- SERIAL0: USB OTG1
- SERIAL1: USART1 (PA9 TX, PA10 RX)
- SERIAL2: USART2 (PA2 TX, PA3 RX)
- SERIAL3: USART3 (PD8 TX, PD9 RX)
- SERIAL4: UART4  (PD1 TX, PD0 RX)
- SERIAL5: UART5  (PB6 TX, PB5 RX)
- SERIAL6: USART6 (PC6 TX, PC7 RX)
- SERIAL7: UART7  (PE8 TX, PE7 RX)
- SERIAL8: UART8  (PE1 TX, PE0 RX)

Default serial functions:
- DEFAULT_SERIAL1_PROTOCOL = RCIN
- DEFAULT_SERIAL4_PROTOCOL = GPS
- DEFAULT_SERIAL8_PROTOCOL = ESCTelemetry

## PWM Outputs
12 outputs are defined:

### Outputs 1-8 (Bidirectional-capable)
1. PE14: TIM1_CH4 (GPIO50)  - BIDIR
2. PE13: TIM1_CH3 (GPIO51)  - BIDIR
3. PE11: TIM1_CH2 (GPIO52)  - BIDIR
4. PE9 : TIM1_CH1 (GPIO53)  - BIDIR
5. PD12: TIM4_CH1 (GPIO54)  - BIDIR
6. PD13: TIM4_CH2 (GPIO55)  - BIDIR
7. PD14: TIM4_CH3 (GPIO56)  - BIDIR
8. PB1 : TIM3_CH4 (GPIO57)  - BIDIR

### Outputs 9-12
9.  PE5: TIM15_CH1 (GPIO58)
10. PE6: TIM15_CH2 (GPIO59)
11. PA0: TIM2_CH1  (GPIO60)  (LED pad)
12. PA1: TIM5_CH2  (GPIO61)  (Buzzer)

## User GPIO
- PE2: CAM_SWITCH (GPIO80) OUTPUT LOW
- PE4: VTX_POWER  (GPIO81) OUTPUT HIGH
- PD10: USER_GPIO3 (GPIO82) OUTPUT HIGH (e.g. Bluetooth control)

## Notes
- DMA_NOSHARE is set for SPI1* and SPI2* (DMA sharing restrictions).
- Frame type default is set for BF migration (HAL_FRAME_TYPE_DEFAULT = 12).