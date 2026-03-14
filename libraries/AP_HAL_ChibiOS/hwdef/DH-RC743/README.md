# DH-RC743 Flight Controller

The DH-RC743 is a flight controller designed for ArduPilot, based on the STM32H743 microcontroller.

## Features

- **MCU:** STM32H743, 480 MHz, 2 MB Flash
- **IMU:** ICM-42688P (SPI3)
- **Barometer:** BMP388 (I2C)
- **Magnetometer:** IST8310 (I2C, internal with YAW_90 rotation; external probe supported)
- **OSD:** AT7456E (SPI1)
- **MicroSD Card Slot** (SDMMC 4-bit)
- **RGB LED** (active-low, accent red/green/blue)
- **10 PWM Outputs** (4 with bidirectional DShot support on M1, M3, M5, M7)
- **USB-C** (OTG)

## Pinout / Connectors

### UARTs

| Function   | UART   | TX Pin | RX Pin | Default Protocol         |
|------------|--------|--------|--------|--------------------------|
| USB        | OTG1   | PA11   | PA12   | MAVLink (SERIAL0)        |
| TELEM1     | USART1 | PA9    | PA10   | MAVLink (SERIAL1)        |
| DJI O3     | USART2 | PA2    | PA3    | DJI FPV / MSP (SERIAL2)  |
| GPS        | USART3 | PD8    | PD9    | MAVLink2 (SERIAL3)       |
| TELEM2     | UART4  | PA0    | PA1    | MAVLink2 (SERIAL4)       |
| RC Input   | USART6 | PC6    | PC7    | RCin (SERIAL5)           |
| ESC Telem  | UART7  | —      | PE7    | ESC Telemetry (SERIAL6)  |
| TELEM3     | UART8  | PE1    | PE0    | (SERIAL7)                |

### Default Serial Protocols

| Parameter        | Value | Protocol           |
|------------------|-------|--------------------|
| SERIAL2_PROTOCOL | 42    | DJI FPV (MSP DisplayPort) |
| SERIAL4_PROTOCOL | 2     | MAVLink2           |
| SERIAL5_PROTOCOL | 23    | RCin               |
| SERIAL6_PROTOCOL | 16    | ESC Telemetry      |

### I2C Buses

| Bus  | Function           | SCL  | SDA  |
|------|---------------------|------|------|
| I2C1 | External Connector  | PB8  | PB9  |
| I2C2 | Baro & Mag (Internal) | PB10 | PB11 |

> **Note:** I2C bus ordering is `I2C2, I2C1`, so I2C2 is bus index 0 (internal) and I2C1 is bus index 1 (external).

### PWM Outputs

| Output | Pin  | Timer     | Bidirectional DShot |
|--------|------|-----------|---------------------|
| M1     | PE14 | TIM1_CH4  | Yes                 |
| M2     | PE13 | TIM1_CH3  | No                  |
| M3     | PE11 | TIM1_CH2  | Yes                 |
| M4     | PE9  | TIM1_CH1  | No                  |
| M5     | PB1  | TIM3_CH4  | Yes                 |
| M6     | PB0  | TIM3_CH3  | No                  |
| M7     | PD12 | TIM4_CH1  | Yes                 |
| M8     | PD13 | TIM4_CH2  | No                  |
| M9     | PE5  | TIM15_CH1 | No                  |
| M10    | PE6  | TIM15_CH2 | No                  |

Motors 1–4 share TIM1, Motors 5–6 share TIM3, Motors 7–8 share TIM4, and Motors 9–10 share TIM15.

### Power Sensing (ADC)

| Function        | Pin | ADC Channel | Scale  |
|-----------------|-----|-------------|--------|
| Battery Voltage | PC0 | ADC1 (10)   | 21.12  |
| Battery Current | PC1 | ADC1 (11)   | 40.2   |

Battery monitoring is enabled by default (`BATT_MONITOR = 4`).

### SBUS Inversion

An SBUS inverter is controlled via PD0 (`SBUS_INV`, active low).

### SPI Buses

| Bus  | Function  | SCK | MISO | MOSI |
|------|-----------|-----|------|------|
| SPI1 | OSD (AT7456E) | PA5 | PA6  | PA7  |
| SPI2 | (Reserved/BMI088) | PD3 | PC2  | PC3  |
| SPI3 | IMU (ICM-42688P) | PB3 | PB4  | PD6  |

### LEDs

| Color | Pin | GPIO |
|-------|-----|------|
| Red   | PE2 | 0    |
| Green | PE3 | 1    |
| Blue  | PE4 | 2    |

LEDs are active-low. RGB notification is enabled by default.

## Bootloader

The bootloader occupies the first 128 KB of flash. During bootloader mode:

- **Green LED (PE3):** Activity indicator
- **Blue LED (PE4):** Bootloader status indicator

Bootloader UART access is available on USART1 (PA9/PA10) and USB OTG1.

## Default Frame Type

The default frame type is set to **12** (`HAL_FRAME_TYPE_DEFAULT`), corresponding to BetaFlight Quad X motor order for easy BetaFlight migration.

## OSD

Dual OSD output is supported:

- **OSD_TYPE (default 1):** AT7456E analog OSD
- **OSD_TYPE2 (default 5):** MSP DisplayPort (for DJI O3 via SERIAL2)

## Building Firmware

```bash
./waf configure --board DH-RC743
./waf copter
```

## Board ID

`AP_HW_DH-RC743`

## UART Wiring Summary

```text
       ┌─────────────────────────┐
       │       DH-RC743          │
       │                         │
USB ───┤ OTG1                    │
       │                         │
T1  ───┤ USART1  (TX:PA9/RX:PA10)│── Telemetry / GCS
DJI ───┤ USART2  (TX:PA2/RX:PA3) │── DJI O3 Air Unit
GPS ───┤ USART3  (TX:PD8/RX:PD9) │── GPS Module
T2  ───┤ UART4   (TX:PA0/RX:PA1) │── Telemetry 2
RC  ───┤ USART6  (TX:PC6/RX:PC7) │── Receiver
ESC ───┤ UART7   (RX:PE7)        │── ESC Telemetry
T3  ───┤ UART8   (TX:PE1/RX:PE0) │── Telemetry 3
       └─────────────────────────┘
```

## Firmware

Firmware for this board can be found here in sub-folders labeled “DH-RC743”.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
\*.apj firmware files.
