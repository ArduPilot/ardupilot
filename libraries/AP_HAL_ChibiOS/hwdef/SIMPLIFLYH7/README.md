# SimpliFly H7 Flight Controller

The SimpliFly H7 is a flight controller manufactured by [MACFOS](https://www.robu.in).

**Purchase link:** [SimpliFly H7 on robu.in](https://www.robu.in)

## Features

- STM32H743 microcontroller running at 480 MHz
- ICM42688P IMU (SPI, mounted on bottom of PCB)
- BMP280 barometer (I2C)
- AT7456E (MAX7456) OSD chip
- W25Q128FV 128 Mbit (16 MB) onboard dataflash for logging
- 5 UARTs (UART1, UART3, UART4, UART5[RX-only], UART7)
- 4 motor outputs (BDshot capable, all on TIM8)
- 4 servo outputs (TIM4)
- 1 LED strip output (NeoPixel, TIM3)
- I2C for external compass
- Analog RSSI input
- Battery voltage and current sensing
- Buzzer support (inverted, open-drain)
- 9V,3A regulator (EN: PB2)
- Camera control output (PB3)
- MicroSD card not present; uses onboard dataflash for logging

## Pinout

![SimpliFly H7 Top](TopView_SIMPLIFLYH7.png "SimpliFly H7 Top View")
![SimpliFly H7 Bottom](BottomView_SIMPLIFLYH7.png "SimpliFly H7 Bottom View")

### ESC Connector Wiring

The ESC interface is an **8-pin connector** located on the bottom of the board. Pin 1 is on the left when viewed from the bottom with the connector facing you.

| Pin | Label | MCU Pin | Function                        | Notes                     |
|-----|-------|---------|---------------------------------|---------------------------|
| 1   | M4    | PC9     | Motor 4 signal                  | TIM8_CH4                  |
| 2   | M3    | PC8     | Motor 3 signal                  | BDshot capable (TIM8_CH3) |
| 3   | M2    | PC7     | Motor 2 signal                  | TIM8_CH2                  |
| 4   | M1    | PC6     | Motor 1 signal                  | BDshot capable (TIM8_CH1) |
| 5   | T     | PD2     | ESC Telemetry RX (UART5)        | NODMA, RX only            |
| 6   | C     | PC1     | Current sense (ADC1)            | BATT_CURR_PIN 11          |
| 7   | G     | —       | Ground                          |                           |
| 8   | V     | PC2     | Battery voltage sense (ADC1)    | BATT_VOLT_PIN          |

> **Note:** All 4 motor signal pins (M1–M4) share TIM8 and must use the same output protocol (PWM / DShot / BDshot). ESC telemetry is on SERIAL5 — set `SERIAL5_PROTOCOL = 16` (ESC Telemetry) in Mission Planner.

## UART Mapping

| Name      | Pin   | Function           | Notes           |
|-----------|-------|--------------------|-----------------|
| SERIAL0   | USB   | USB                |                 |
| SERIAL1   | UART1 | TX: PA9, RX: PA10  | VTX |
| SERIAL3   | UART3 | TX: PB10, RX: PB11 | General purpose |
| SERIAL4   | UART4 | TX: PA0, RX: PA1   | Receiver input  |
| SERIAL5   | UART5 | RX: PD2 only       | ESC Telemetry |
| SERIAL7   | UART7 | TX: PE8, RX: PE7   | GPS              |

## PWM Output

The SimpliFly H7 supports up to 9 PWM outputs.

| Output | Pin  | Timer    | BDshot | Function |
|--------|------|----------|--------|----------|
| PWM1   | PC6  | TIM8_CH1 | Yes    | Motor 1  |
| PWM2   | PC7  | TIM8_CH2 | Yes    | Motor 2  |
| PWM3   | PC8  | TIM8_CH3 | Yes    | Motor 3  |
| PWM4   | PC9  | TIM8_CH4 | Yes    | Motor 4  |
| PWM5   | PB6  | TIM4_CH1 | No     | Servo 1  |
| PWM6   | PB7  | TIM4_CH2 | No     | Servo 2  |
| PWM7   | PD14 | TIM4_CH3 | No     | Servo 3  |
| PWM8   | PD15 | TIM4_CH4 | No     | Servo 4  |
| PWM9   | PB5  | TIM3_CH2 | No     | LED Strip (NeoPixel) |

**Timer groups** — all outputs in the same timer group must use the same protocol:

- Group 1: PWM1, PWM2, PWM3, PWM4 (TIM8) — supports PWM, DShot, BDshot
- Group 2: PWM5, PWM6, PWM7, PWM8 (TIM4) — supports PWM, DShot
- Group 3: PWM9 (TIM3) — LED strip (NeoPixel/WS2812)

The motor ordering follows the Betaflight/X layout (HAL_FRAME_TYPE 12).

## RC Input

RC input is configured on pin PA1, which is shared with UART4_RX.

- **PPM**: Connect to the RX1 pad. Supported via timer capture (TIM2_CH2).
- **SBUS / serial RC protocols** (CRSF, ELRS, DSM, iBus, etc.): Connect to the RX1 pad and set `SERIAL4_PROTOCOL` to the appropriate value. Set `SERIAL4_OPTIONS = 0` normally; for SBUS set to `0` with half-duplex if required by your receiver.

For CRSF/ELRS (bi-directional):

- Connect TX to RX1(PA1)
- Set `SERIAL4_PROTOCOL = 23` (RCIN)
- Set `SERIAL4_BAUD = 115`

See [Radio Control Systems](https://ardupilot.org/copter/docs/common-rc-systems.html) for more detail.

## OSD Support

The SimpliFly H7 has a built-in AT7456E (MAX7456) OSD chip on SPI2. The OSD is enabled by default (`OSD_TYPE = 1`).

To use MSP DisplayPort OSD simultaneously on an external device, set `OSD_TYPE2 = 5` and configure a UART for MSP.

## 9V supply

The 9V supply is controlled by pin PB2 (PINIO1, GPIO 81). The EN pin is held HIGH by default via pull-up, keeping the 9V rail always active at boot.

To enable software control of 9V buck, assign a relay to GPIO 81 in Mission Planner:

- Set `RELAY1_PIN = 81` (or any available relay number)
- Use the relay channel to switch 9V buck on/off

## Camera Switch

Camera control is available on pin PB3 (CAMERA1, GPIO 82).

- GPIO 82 is assigned to `RELAY3` by default (`RELAY3_PIN_DEFAULT 82`)
- Set `CAM_RELAY_ON` and associated parameters to control the camera trigger

## GPIOs

| GPIO | Pin | Default       | Function                      |
|------|-----|---------------|-------------------------------|
| 80   | PD3 | HIGH (silent) | Buzzer (inverted, open-drain) |
| 81   | PB2 | HIGH          | 9V VTX regulator EN (PINIO1)  |
| 82   | PB3 | LOW           | Camera control (RELAY3)       |
| 90   | PD1 | LOW           | Status LED                    |

## RSSI / Analog Pins

An analog RSSI input is available on pin PC0 (ADC1, GPIO analog pin 10).

Set `RSSI_ANA_PIN = 10` in Mission Planner to enable analog RSSI reading.

## Battery Monitor

The SimpliFly H7 has onboard voltage and current sensing. Default parameters:

| Parameter        | Value |
|------------------|-------|
| `BATT_MONITOR`   | 4 (Analog voltage and current) |
| `BATT_VOLT_PIN`  | 12 (PC2) |
| `BATT_CURR_PIN`  | 11 (PC1) |
| `BATT_VOLT_MULT` | 11.0 (voltage divider ratio: scales raw ADC voltage to actual battery voltage) |
| `BATT_AMP_PERVLT`| 35.4 (adjust for your current sensor) |

The voltage sensor supports up to 2S-6S LiPo batteries. The current scale value may need adjustment depending on the ESC or current sensor used.

## Compass

The SimpliFly H7 does not have a built-in compass. An external compass can be connected via I2C1 (SCL: PB8, SDA: PB9). ArduPilot will automatically probe for all supported I2C compass types.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
\*.apj firmware files.
