# HGLRC H743 EVO Flight Controller

The HGLRC H743 EVO is an STM32H743-based flight controller designed for FPV
multirotor applications, featuring dual IMUs, an on-board analog OSD, blackbox
dataflash, a microSD slot, and a wide-input integrated power supply.

## Features

- STM32H743VIT6 microcontroller (480 MHz, 2 MB flash)
- Dual IMU: ICM-42688-P (SPI1) and BMI270 (SPI4)
- DPS368 barometer (I2C2)
- IST8310 compass (I2C2)
- AT7456E analog OSD (SPI2)
- microSD card slot (SDMMC, 4-bit) for logging
- 1x CAN port (FDCAN1)
- 8 motor outputs plus 2 auxiliary PWM outputs and a serial LED output
- Built-in voltage and current sensing
- Wide-input BEC: 12V 2A, 5V 3A, dual 3.3V 1A rails (up to 33.6V input)
- USB Type-C

## Pinout

### UART Mapping

| Name    | Function           | TX Pin | RX Pin |
|---------|--------------------|--------|--------|
| SERIAL0 | USB                | —      | —      |
| SERIAL1 | VTX                | PA9    | PA10   |
| SERIAL2 | RC input (CRSF)    | PD5    | PD6    |
| SERIAL3 | Spare              | PD8    | PD9    |
| SERIAL4 | RC input (SBUS)    | PD1    | PA1    |
| SERIAL5 | Telemetry          | PB6    | PB5    |
| SERIAL6 | GPS                | PE8    | PE7    |
| SERIAL7 | Spare              | PE1    | PE0    |

All UARTs have full DMA capability.

### RC Input

RC input is configured on SERIAL2 (CRSF/ELRS) by default. SBUS receivers can be
connected to SERIAL4. Set the `SERIALn_PROTOCOL` of the chosen port to `23`
(RCIN).

### Motor / Servo Outputs

| Output | Pin  | Timer    | DShot |
|--------|------|----------|-------|
| 1      | PE9  | TIM1_CH1 | Yes   |
| 2      | PE11 | TIM1_CH2 | Yes   |
| 3      | PE13 | TIM1_CH3 | Yes   |
| 4      | PE14 | TIM1_CH4 | Yes   |
| 5      | PD12 | TIM4_CH1 | Yes   |
| 6      | PD13 | TIM4_CH2 | Yes   |
| 7      | PC6  | TIM8_CH1 | Yes   |
| 8      | PC7  | TIM8_CH2 | Yes   |
| 9      | PA2  | TIM2_CH3 | Yes   |
| 10     | PA3  | TIM2_CH4 | Yes   |

Channels that share a timer must use the same output protocol and update rate.

A serial (WS2812) LED output is available on PA0 (TIM5).

## Battery Monitoring

The board has a built-in voltage and current sensor. Default parameters:

- `BATT_MONITOR` `4` (analog voltage and current)
- Voltage sense on ADC1 (PC0)
- Current sense on ADC1 (PC1)

The voltage and current scaling parameters (`BATT_VOLT_MULT`,
`BATT_AMP_PERVLT`) should be calibrated against a reference meter.

## Compass

The board includes an on-board IST8310 compass on I2C2. An external compass can
be attached to the I2C1 pads and will be auto-detected.

## OSD

The board has an on-board AT7456E analog OSD, enabled by default. MSP
DisplayPort OSD on the VTX UART is also supported.

## Loading Firmware

Firmware for this board can be loaded with any compatible ground station, or
built from source:

```bash
./waf configure --board HGLRC_H743_EVO
./waf copter
