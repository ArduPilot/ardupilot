# Sierra-F1 Autopilot by Sierra Aerospace
https://www.sierraaerospace.in/

## Features
# Main Board:
 - STM32H743 microcontroller
 - 32MB QSPI flash
 - BMI088+ICM42688 IMUs
 - DPS310 barometer on SPI 
 - SDMMC support
 - 4 UARTs- 2 with HW flow control
 - 6 PWM outputs with DSHOT capability
 - Dual FDCAN
 - RCIN port
 - Dual power input with 2xI2C
 - No co-processor

# Daughter board:
 -Ethernet PHY
 -ICM42688/ICM45686/ADIS16507
 -DPS310
 -More PWMs with ALT function

## Physical spec:
 -Weight: 15g, Dimension: 35x45mm

## UART Mapping
 - SERIAL0 -> USB
 - SERIAL1 -> UART2 (Telem1)
 - SERIAL2 -> UART8 (Telem2)
 - SERIAL3 -> UART7 (GPS)
 - SERIAL4 -> UART6 (UART or RCIN)
 - SERIAL7 -> USB2 (virtual port on same connector)

The Telem1 & Telem2 port has RTS/CTS pins, the other UARTs do not have RTS/CTS.

## Connectors

 -All JST-GH 6 and 4 pin connectors with FMUv6 standard pinout
 -USB-C

## RC Input
 
RC input is configured on UART6 Rx pin.

## PWM Output

The Sierra-F1 supports up to 6 PWM outputs and DSHOT. Additional PWMs can be accessed via daugter board with ALT function.
The 6 PWM outputs are in 2 groups:

 - PWM 1, 2, 3 and 4 in group1
 - PWM 5, 6 in group2

# PWM channels
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50)
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52)
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)
PB0  TIM3_CH3 TIM3 PWM(5) GPIO(54)
PB1  TIM3_CH4 TIM3 PWM(6) GPIO(55)

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has a dedicated power monitor ports on a 6 pin
connector. The pinout on PWR1 and PWR2 follows FMUv6 standard. 
Vcc,Vcc,I2Cx_SCL, I2Cx_SDA, GND, GND

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of *.apj firmware files with any ArduPilot
compatible ground station.
