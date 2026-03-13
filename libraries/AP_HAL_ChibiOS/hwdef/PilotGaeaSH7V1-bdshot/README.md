# PilotGaeaSH7V1-bdshot Flight Controller

The PilotGaeaSH7V1-bdshot is a flight controller designed and produced by PilotGaea.

## Features

- STM32H743 microcontroller
- Dual ICM42688P IMUs (on SPI1 and SPI4)
- DPS310 barometer
- AT7456E OSD
- MicroSD card slot
- Dual camera input with switching support
- 1x USB (Type-C)
- 5.5 UARTs (UART7 with CTS/RTS flow control)
- 11 PWM / Dshot outputs (8 with Bi-directional DShot support)
- 2x I2C (I2C2 internal, I2C1 external)
- 1x CAN bus
- 5 ADC inputs (Battery Voltage, Current, BATT2 Voltage, BATT2 Current, RSSI)
- 5V/1.5A BEC for main power supply
- 8V/1.5A BEC for powering Video Transmitter

## Mechanical

- Dimensions: 36 x 36 x 17 mm
- Weight: 10.5g

## Physical and pinout

![PilotGaeaSH7V1-bdshot front view](./PilotGaea_front_view_Pin.jpg)

![PilotGaeaSH7V1-bdshot rear view](./PilotGaea_rear_view_Pin.jpg)

## Power supply

The PilotGaeaSH7V1-bdshot supports 3-8s Li battery input. It has 2 ways of BEC, which result in 3 ways of power supplys. Please see the table below.

| Power symbol | Power source | Max power (current) |
|--------------|---------------------------------------------------|---------------------|
| BAT | directly from battery | |
| 5V | from 5V BEC | 7.5W (1.5A) |
| 8V | from 8V BEC, controlled by MCU | 12W (1.5A) |
| 4V5 | from USB or 5V BEC, diodes isolate the two powers | 4.7W (1A) |

## UART Mapping

The UARTs are marked RXn and TXn in the above pinouts. The RXn pin is the receive pin for UARTn. The TXn pin is the transmit pin for UARTn.

| ArduPilot Serial | Hardware UART | Default Function | DMA Support | DMA Stream (Shared/Dedicated) |
|------------------|----------------|------------------|-------------|------------------------------------------------|
| SERIAL0 | OTG1 | USB | No | Handled by USB Controller |
| SERIAL1 | UART7(CTS/RTS) | Telem 1 | Yes | Shared TX (Stream 7) / Dedicated RX (Stream 0) |
| SERIAL2 | USART1 | GPS | Yes | Shared TX (Stream 7) / Dedicated RX (Stream 5) |
| SERIAL3 | USART2 | Telem 2 | Yes | Shared TX (Stream 7) / Dedicated RX (Stream 6) |
| SERIAL4 | USART3 | not available | Yes | Shared TX (Stream 7) / Dedicated RX (Stream 7) |
| SERIAL5 | UART8 | ESC Telem | No (NODMA) | (CPU-based PIO) |
| SERIAL6 | UART4 | User Port | No (NODMA) | (CPU-based PIO) |
| SERIAL7 | USART6 | RCIN | Yes | Shared TX (Stream 7) / Dedicated RX (Stream 1) |

Any UART can be re-tasked by changing its protocol parameter.

## RC Input

The default RC input is configured on the USART6 and supports all RC protocols except PPM. The SBUS pin is inverted and connected to RX6. RC can be attached to any UART port as long as the serial port protocol is set to `SERIALn_PROTOCOL=23` and SERIAL6_Protocol is changed to something other than '23'.

## OSD Support

The PilotGaeaSH7V1-bdshot Supports onboard analog OSD using the AT7456 chip. The composited image is output via the VTX pin.

## PWM Output and DShot

The PilotGaeaSH7V1-bdshot supports up to **11 physical PWM outputs**, organized into **6 independent timer groups**. All groups support DShot, but **Bi-directional DShot (BDShot)** is fully optimized for **Groups 1-4 (Outputs 1-8)**.

### PWM Grouping Table

| Group | PWM Output | Timer | BDShot Support | Recommended Use |
| :--- | :--- | :--- | :--- | :--- |
| **1** | 1, 2 | TIM3 | **Full (DMA)** | Motors 1-2 (BDShot) |
| **2** | 3, 4 | TIM2 | **Full (DMA)** | Motors 3-4 (BDShot) |
| **3** | 5, 6 | TIM5 | **Full (DMA)** | Motors 5-6 (BDShot) |
| **4** | 7, 8 | TIM4 | **Full (DMA)** | Motors 7-8 (BDShot) |
| **5** | 11, 12 | TIM15 | No (NODMA) | Auxiliary / Servos |
| **6** | 13 | TIM1 | No (NODMA) | NeoPixel / LED |

### Bi-directional DShot Configuration

To use BDShot for RPM filtering, you must flash the `PilotGaeaSH7V1-bdshot` firmware.

- **Outputs 1-8:** Fully optimized with dedicated DMA resources to ensure stable DShot600 performance and telemetry feedback.
- **Outputs 11-13:** Configured with `NODMA`. While they support standard PWM/DShot, they do not support RPM telemetry.

> **Note:** PWM 9 and 10 are defined in firmware but not physically broken out.
>
> **Important:** Every output within a timer group must use the same protocol (e.g., Output 3 & 4 must both be DShot).

## Battery Monitoring

The PilotGaeaSH7V1-bdshot features high-voltage monitoring capabilities, supporting up to 8S LiPo on both sensors. Notably, **BATT2** is optimized with a higher divider ratio for enhanced voltage range support.

### Primary Battery (BATT)

Enable Battery monitor with these parameter settings:

- BATT_MONITOR 4 (Then reboot)
- BATT_VOLT_PIN 10
- BATT_CURR_PIN 11
- BATT_VOLT_MULT 11.0
- BATT_AMP_PERVLT 40.0

### Secondary Battery (BATT2)

Enable Battery monitor with these parameter settings:

- BATT2_MONITOR 4 (Then reboot)
- BATT2_VOLT_PIN 18
- BATT2_CURR_PIN 7
- BATT2_VOLT_MULT 21.0
- BATT2_AMP_PERVLT 40.0

### Battery Monitoring (BATT2) Setup

The second battery monitor is **disabled** by default. To use it:

1. Set `BATT2_MONITOR` = **4** (Analog Voltage and Current) and reboot.
2. **Verification:** Confirm that `BATT2_VOLT_MULT` is set to **21.0**.

## Compass

The PilotGaeaSH7V1-bdshot has no built-in compass, so if needed, you should use an external compass.

## Analog cameras

The PilotGaeaSH7V1-bdshot supports up to 2 cameras, connected to pin C1 and C2. You can select the video signal to VTX from camera by an RC channel. Set the parameters below:

- RELAY2_FUNCTION 1
- RELAY_PIN2 82
- RC8_OPTION 34

## 8V switch

The 8V power supply can be controlled by an RC channel. Set the parameters below:

- RELAY1_FUNCTION 1
- RELAY_PIN 81
- RC7_OPTION 28

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "*with_bl.hex" firmware, using your favourite DFU loading tool.
Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the "*.apj" firmware files.
