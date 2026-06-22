# TBS LUCID H7 AIO Flight Controller

The TBS LUCID H7 AIO (LAIO6S, "AIO One 6S") is an all-in-one board from
Team BlackSheep combining an STM32H743 flight controller with an integrated
4in1 ESC, an onboard Crossfire 2.0 receiver and an analog OSD.

## Features

- STM32H743VIH6 microcontroller
- ICM-42688-P IMU
- DPS368 barometer
- W25Q NOR flash for logging
- Integrated 4in1 BLHeli ESC (DShot, bidirectional) rated for 6S
- Onboard TBS Crossfire 2.0 receiver
- Onboard analog OSD (driven over MSP DisplayPort)
- 5V BEC, voltage and current sensing
- DJI O4 / HD VTX connector with SBUS

## Pinout

### UART Mapping

- SERIAL0 -> USB
- SERIAL1 -> USART1 (4in1 ESC telemetry)
- SERIAL2 -> USART2 (HD VTX / SmartAudio connector)
- SERIAL3 -> USART3 (onboard Crossfire 2.0 receiver, RCIN)
- SERIAL4 -> UART4 (SBUS in, RX only)
- SERIAL5 -> UART5 (onboard analog OSD, MSP DisplayPort)
- SERIAL6 -> UART7 (spare / SmartAudio)
- SERIAL7 -> UART8 (GPS)

### RC Input

RC defaults to the onboard Crossfire 2.0 receiver on USART3 (SERIAL3, RCIN).
The SBUS pad on the DJI O4 connector is UART4 RX; set SERIAL4_PROTOCOL = 23
(RCIN) to use it instead.

### OSD Support

The board has an onboard analog OSD addressed over MSP DisplayPort on SERIAL5.
A digital HD VTX (DJI O4) can be driven over MSP DisplayPort on SERIAL2 instead.

### Battery Monitoring

Voltage and current sensing are built in and enabled by default.
- Voltage divider 120k/10k (scale 13.0)
- Current via 0.25mOhm shunt + INA180A2 x50 gain (scale 80.0)

### Motor Outputs

The four outputs drive the integrated 4in1 ESC and are all on TIM8, supporting
bidirectional DShot. Motor order follows Betaflight X.

### LEDs

- Blue and green status LEDs
- WS2812 serial LED strip output (output 5, SERVO5_FUNCTION = 120)

## Loading Firmware

Firmware is provided as ardu*.apj files. Initial loading is done with DFU over
USB; subsequent updates can be flashed with a ground station over MAVLink.
