The PRIMEH7 is an STM32H743-based flight controller designed for compact drones that require high performance with a reduced connector layout.
This board is fully supported by ArduPilot via the ChibiOS hardware definition files.

Hardware Overview

MCU: STM32H743
Flash: 2MB
RAM Map: Alternate RAM map enabled for PX4-bootloader compatibility
Crystal: 16 MHz
Sensors:
ICM42688 (IMU)
BMI055 / BMI088 (IMU set)
MS5611 / BMP388 (Barometer options)
IST8310 (Compass)
Storage: FRAM (SPI2)
MicroSD: SDMMC2 interface
IOMCU: USART6 connection
PWM Outputs: 8 channels with bi-directional DShot support
USB: OTG FS
Power Sensing:
BATT_VOLT/BATT_CURR
BATT2_VOLT/BATT2_CURR

Bootloader

A dedicated hwdef-bl.dat is provided for generating the bootloader.

Bootloader features:
Flash-from-SD support
Separate LED indications
FRAM support
Loads firmware at 128KB offset

UART Mapping

Port Interface Pins

TELEM1 UART7 PE8 / PE7
TELEM2 UART5 PC12 / PD2
GPS1 USART1 PB6 / PA10
GPS2 UART8 PE1 / PE0
TELEM3 USART2 PD5 / PA3
DEBUG USART3 PD8 / PD9


USB: OTG1 (PA11/PA12/PA9)

I2C Buses

I2C1: GPS/MAG
I2C2: GPS2/MAG
I2C4: Internal sensors

CAN Buses

CAN1: PD0/PD1
CAN2: PB5/PB13

SPI Devices

SPI1: IMUs (ICM42688, BMI055/BMI088)
SPI2: FRAM

PWM Outputs

8 PWM outputs using:
TIM1 CH1–4
TIM4 CH3–4
TIM5 CH1–2

DShot support is enabled on IOMCU.

LEDs

Red (PD10)
Blue (PD11)
BoardLED2 configuration is enabled.

Power Control

Sensor power enable
5V peripheral & high-power enable
Spektrum power control (active high)

Default Parameters

Included in defaults.parm:
Correct battery monitoring setup
BATT_VOLT and BATT_CURR scaling
Support for dual-battery sensing
