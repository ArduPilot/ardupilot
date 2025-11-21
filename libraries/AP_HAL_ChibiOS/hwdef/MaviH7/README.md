# MaviH7 Flight Controller Board

## Overview
STM32H743-based flight controller.

## Board Features
MCU: STM32H743xx, Flash: 2MB, RAM: 1MB
MicroSD (SDMMC2), Dual battery monitoring
LEDs: Red PD10, Blue PD11
Spektrum power: PH2

## UART Mapping
UART7: Telem1 (PE8/PE7)
UART5: Telem2 (PC12/PD2)
USART3: Debug (PD8/PD9)
USART6: IOMCU (PC6/PC7)
UART8: GPS2 (PE1/PE0)
USART1: GPS1 (PB6/PA10)
USART2: Telem3 (PD5/PA3)

## PWM Outputs
1: PA8, 2: PE11, 3: PE13, 4: PE14
5: PD14, 6: PD15, 7: PA0, 8: PA1

## I2C Buses
I2C1: GPS1 + MAG, I2C2: GPS2 + MAG, I2C4: Internal

## SPI Devices / IMUs
SPI1: ICM42688, BMI055/BMI088
SPI2: FRAM

## ADC / Battery Monitoring
BATT1: PC5 (V), PC4 (I)
BATT2: PB1 (V), PA2 (I)

## CAN Bus
CAN1: PD0/PD1, CAN2: PB5/PB13

## Notes
Uses alternate RAM mapping for PX4 bootloader
DMA configured for multiple UARTs
