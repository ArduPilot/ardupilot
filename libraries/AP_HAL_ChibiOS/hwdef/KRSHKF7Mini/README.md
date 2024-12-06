# KRSHKF7Mini Flight Controller
The first ever flight controller by Karshak Drones.

## Features

 - STM32F745VGTx microcontroller
 - IMU: MPU6000
 - Baro: MS5611
 - Hardware UARTS:8(2 shared with external OSD SPI)
 - 3 Analog Pin (1 Analog RSSI Input Pin) for external use
 - USB ports for FMU with BOOT0 jumper (solder for DFU mode)
 - 1 I2C port(external)
 - 1 CAN port(deactivated)
 - 6-pwm inputs (PPM on input 1)
 - 10 outputs (8 active PWM output)(DSHOT supported on seven of them)
 - 3 SPI Port, 2 External SPI port(shared with UART4 and UART5)
 - Buzzer (not on board) 
 - 2 LED on board
 - SDCARD connector(slot) on board for logging

## UART Mapping

 - SERIAL0 -> USB     (OTG1)
 - SERIAL1 -> USART1  (Telem1)(DMA capable)
 - SERIAL2 -> USART3  (GPS)   (DMA capable)
 - SERIAL3 -> UART8   (Telem2)   
 - SERIAL4 -> UART6   (Extra UART)
 - SERIAL5 -> UART7   (Radio Receiver, GPS/Telemetry, Digital FPV)
 - SERIAL6 -> USART2  (SBUS)  (RC input, no DMA capable)

## RC Input

RC input is configured on the SBUS pin (UART2_RX). It supports all RC protocols 
except serial protocols. Input1 can be used for PPM.

## PWM Output

The KRSHKF7Mini supports up to 8 PWM outputs. Seven of the outputs support DShot.

## GPIOs

All 8 PWM channels can be used for GPIO functions.
The pin numbers for these PWM channels in ArduPilot are shown below:

| PWM Channels | Pin  | DSHOT |
| ------------ | ---- | ----- |
| PWM10        | 50   |  YES  |
| PWM6         | 51   |  YES  |
| PWM2         | 52   |  YES  |
| PWM1         | 53   |  YES  |
| PWM4         | 54   |  YES  |
| PWM7         | 55   |  YES  |
| PWM5         | 56   |  NO   |
| PWM9         | 57   |  YES  |

## Battery Monitoring

The correct battery setting parameters are set by default and are:
 
 - BATT_MONITOR 4
 - BATT_VOLT_PIN 10
 - BATT_CURR_PIN 11
 - BATT_VOLT_SCALE 11
 - BATT_CURR_SCALE 25.0

## Compass

The KRSHKF7Mini has no built-in compass, you can attach an external compass using 
I2C on the SDA and SCL pads.

## Firmware
Firmware for these boards can be found here: https://github.com/ArduPilot/ardupilot
in the sub-folders labelled “KRSHKF7Mini”.

### Initial Firmware Load
1. **Prepare for DFU Mode:** Plug in the USB with the boot jumper soldered.

2. **Load Firmware:** Use your preferred DFU loading tool (e.g., dfuse, STM32CubeProgrammer) to load the ```KRSHKF7Mini_bl.hex``` firmware.

### Firmware Updates
After the initial load, you can update the firmware using Mission Planner or by flashing with the ```.apj``` file if the bootloader is already installed. 
The bootloader for this board can be found in the ```Tools/bootloaders``` folder under the name ```KRSHKF7Mini_bl.bin```.
