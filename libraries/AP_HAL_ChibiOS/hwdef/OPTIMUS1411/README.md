# OPTIMUS1411 (AP_HW_OPTIMUS1411)

OPTIMUS1411 is a custom flight controller based on the STM32H743 MCU, used for ArduPilot development and testing.

# MCU

STM32H743
480 MHz CPU
2 MB Flash, 1 MB RAM

# Main Features

8 PWM outputs 
PWM1,3,5,7 configured as **BIDIR** and suitable for DShot
2× CAN buses
Multiple UART ports for telemetry and GPS
I2C and SPI buses for external peripherals and sensors
microSD (SDMMC2) for logging
Onboard IMUs and barometer as defined in `hwdef.dat`

# Port Mapping

**TELEM1:** UART7 (PE8/PE7, RTS/CTS on PE9/PE10) 
**TELEM2:** UART5 (PC12/PD2, RTS/CTS on PC8/PC9) 
**GPS1:** USART1 (PB6 / PA10) 
**GPS2:** UART8 (PE1 / PE0) 
**Debug UARTs:** USART2 / USART3 
**CAN1:** PD0 / PD1 
**CAN2:** PB5 / PB13 
**microSD:** SDMMC2 pins as defined in `hwdef.dat`

# ArduPilot Integration

**Board ID:** 1208 (`AP_HW_OPTIMUS1411`)
**Bootloader:** `Tools/bootloaders/OPTIMUS1411_bl.bin`
**Build target:** `--board OPTIMUS1411`

This board is a custom design primarily intended for development and experimentation with ArduPilot.
