# HCR-565 Fidelis Flight Controller

!\[HCR-565B.jpg](HCR-565B.jpg)

The HCR-565 Fidelis flight controller produced by [HC Robotics](https://www.hcrobo.com/), manufactured in India.

The HCR-565 (PCB ID: HCR-565B) is a compact 35mm x 35mm H7-class autopilot built around the STM32H753VIH6 microcontroller, with a fully redundant inertial, magnetic, and barometric sensor stack and dual FDCAN buses. It is designed for high-vibration small-frame multirotors and small fixed-wing platforms where size, weight, and sensor redundancy all matter.

## Pinout

**Top**

!\[HCR-565B-Top.jpg](HCR-565B-Top.jpg)

**Bottom**

!\[HCR-565B-Bottom.jpg](HCR-565B-Bottom.jpg)

## Features

**Compute \& Storage**

* STM32H753VIH6 microcontroller (firmware target STM32H743xx)
* 2 Gbit onboard NAND flash (W25N01) for LittleFS dataflash logging
* TC2030 JTAG/SWD debug connector

**Sensors**

* 2x ICM42688 6DOF IMUs, oriented +45° apart for vibration decorrelation
* 2x BMM350 magnetometers (internal, on independent I2C buses)
* 2x ICP20100 barometers (internal, on independent I2C buses)

**Connectivity**

* USB-C port (OTG1) plus secondary USB OTG2 (default SLCAN)
* 100 Mb RMII Ethernet (LAN8742A PHY)
* 5 serial ports plus 2 USB virtual serial ports
* 2 independent FDCAN buses (8 Mbit) with independently-switched 5V-CAN rail
* External I2C bus and external SPI bus broken out on a dedicated I2C/SPI connector
* Dedicated sBUS output port
* Bottom-edge connector for carrier-PCB mounting

**Outputs**

* 8 PWM outputs (4 with bi-directional DShot)
* Onboard tri-color RGB LED and buzzer

**Power \& Sensing**

* 2 power inputs with analog voltage sensing and CAN power-monitor support
* Analog inputs for HV battery sensing, 5V rail sensing, and a configurable ADC/GPIO

**Physical**

* 35mm x 35mm board, 31mm x 31mm mounting (2mm fasteners)
* 9.41 g with headers
* Bottom-edge connector designed for direct mounting onto a carrier PCB

## UART Mapping

|ArduPilot|Hardware|Default Use|Default Protocol|Notes|
|-|-|-|-|-|
|SERIAL0|OTG1 USB|Console|MAVLink2|USB-C connector|
|SERIAL1|USART2|TELEM1|MAVLink2 (2)|RTS/CTS flow control|
|SERIAL2|USART3|TELEM2|MAVLink2 (2)|RTS/CTS flow control|
|SERIAL3|UART4|GPS|—|General purpose|
|SERIAL4|UART5|—|—|General purpose|
|SERIAL5|UART7|sBUS output|SBUS Out (15)|TX only|
|SERIAL6|USART1|RC input|RCIN (23)|RX only; requires `BRD\_ALT\_CONFIG = 1`|
|SERIAL7|OTG2 USB|SLCAN|SLCAN (22)|Routed to CAN1 via `CAN\_SLCAN\_CPORT = 1`|

Only TELEM1 (USART2) and TELEM2 (USART3) provide hardware RTS/CTS flow control. The other UARTs do not.

USART1 RX is shared with the RC input pin (see [RC Input](#rc-input)) and is only available as SERIAL6 when `BRD\_ALT\_CONFIG = 1`.

## RC Input

RC input is on the dedicated RC IN pin (PB15), which is wired in two modes selected by the `BRD\_ALT\_CONFIG` parameter:

* `BRD\_ALT\_CONFIG = 0` (default): PB15 is connected to TIM12 in capture mode, supporting all unidirectional RC protocols (PPM, SBUS, DSM, ST24, SUMD, etc.).
* `BRD\_ALT\_CONFIG = 1`: PB15 is connected to USART1\_RX and exposed as SERIAL6, for UART-based RC protocols (CRSF, FPort, GHST, SRXL2, etc.).

For bi-directional or half-duplex protocols that require a TX line as well, a full UART must be used. See [RC systems](https://ardupilot.org/copter/docs/common-rc-systems.html) for details on each protocol.

## sBUS Output

The HCR-565 provides a dedicated sBUS output on UART7 TX (SERIAL5), preconfigured with `SERIAL5\_PROTOCOL = 15`. This is intended for driving sBUS-compatible servos or external receivers/PWM expanders.

## PWM Output

The HCR-565 has 8 PWM outputs, all driven directly by the STM32H753 (there is no separate IO co-processor). Outputs are arranged in two timer groups:

* PWM 1, 2, 3 and 4 in group 1 (TIM1)
* PWM 5, 6, 7 and 8 in group 2 (TIM3)

Channels within the same group must use the same output rate. If any channel in a group uses DShot then all channels in that group must use DShot. Bi-Directional DShot is supported on PWM 1, 3, 5 and 7.

The PWM outputs are broken out on two physical connectors:

* **Bottom edge connector (PWM 1–8):** The full 8-channel PWM output along with the rest of the board's I/O is brought out on the bottom-edge connector. This connector is designed to allow the HCR-565 to be mounted directly onto a carrier PCB, where the carrier provides whatever signal conditioning, ESC headers, and additional I/O the target airframe requires.
* **Side PWM header (PWM 1–4 + VBAT1):** PWM channels 1–4 are also brought out on a separate side-mounted pin header along with two VBAT1 pass-through pins, providing a simple 4-channel servo/ESC interface for builds (such as quadcopters) that do not require a carrier PCB. The signals on this header are the same physical PWM 1–4 lines as on the bottom connector — they are not independent outputs.

## Battery Monitoring

The board has two power inputs, each of which supports both analog voltage sensing and CAN-based power monitor inputs. The dedicated analog channels are scaled for high-voltage battery sensing (3.3 V × 11 ≈ 36.3 V max, covering up to 8S LiPo):

* Power 1 voltage -> analog input 1 (`BATT\_VOLT\_PIN = 1`, `BATT\_VOLT\_MULT = 11`)
* Power 2 voltage -> analog input 2 (`BATT2\_VOLT\_PIN = 2`, `BATT2\_VOLT\_MULT = 11`)

For digital battery monitoring, DroneCAN/UAVCAN power monitors can be attached to either CAN bus. Both CAN drivers are enabled by default (`CAN\_P1\_DRIVER = 1`, `CAN\_P2\_DRIVER = 2`).

The correct battery setting parameters depend on the type of power brick or smart battery connected.

## Compass

The HCR-565 has two builtin BMM350 magnetometers, each on its own internal I2C bus for redundancy. External compasses may also be attached via the external I2C4 port.

## GPIOs

The PWM outputs and several other pins can be used as GPIOs (relays, buttons, RPM input, etc.). The GPIO numbering scheme is:

|Range|Use|
|-|-|
|1–10|Internal analog inputs|
|11–20|External analog inputs|
|31–50|PWM-capable GPIOs|
|51–70|Internal GPIOs|
|71–90|External GPIOs|

Specific GPIO assignments:

|Signal|GPIO|Notes|
|-|-|-|
|PWM1|31||
|PWM2|32||
|PWM3|33||
|PWM4|34||
|PWM5|35||
|PWM6|36||
|PWM7|37||
|PWM8|38||
|Buzzer|51||
|CAN1 silent|52||
|CAN2 silent|53||
|External GPIO 1|71||
|External GPIO 2|72||
|External GPIO 3|73|Only when PB0 is configured as a digital output rather than ADC — see [Analog Inputs](#analog-inputs)|
|LED Red|90||
|LED Green|91||
|LED Blue|92||

## Analog Inputs

The HCR-565 has the following analog inputs:

|Analog|Pin|Scale|Function|
|-|-|-|-|
|1|PA4|11|HV Battery 1 voltage sense|
|2|PA5|11|HV Battery 2 voltage sense|
|3|PB0|1|External 0–3.3 V ADC input (shared with external GPIO 3)|
|21|PC0|2|5V rail voltage sense|

ANALOG 3 (pin PB0) ships configured as an ADC input. It can alternatively be used as a digital external GPIO (GPIO 73) by changing the hwdef configuration. Only one role can be active at a time.

## CAN

The HCR-565 has two independent FDCAN buses, each rated up to 8 Mbit. Both drivers are enabled by default (`CAN\_P1\_DRIVER = 1`, `CAN\_P2\_DRIVER = 2`), and CAN1 is preconfigured as the SLCAN port (`CAN\_SLCAN\_CPORT = 1`) so that the OTG2 USB virtual serial port can be used as a SLCAN bridge.

Each CAN transceiver has an independent silent-mode control line, exposed as GPIO 52 (CAN1) and GPIO 53 (CAN2).

The CAN connector exposes a dedicated **5V-CAN** rail, switched and fused independently from the 5V-PERIPHERALS rail used by the serial/I2C/SPI connectors. This isolates CAN-bus peripherals from short circuits or in-rush events on the rest of the I/O bus.

## External I2C / SPI

The HCR-565 brings I2C4 (external) and SPI4 (external) out to a single dedicated I2C/SPI connector on the bottom edge, alongside 5V-PERIPHERALS power. This is the recommended attachment point for external compasses, airspeed sensors, optical flow sensors, rangefinders, or other expansion peripherals that need a fast SPI link.

## Ethernet

The HCR-565 provides a 100 Mb Ethernet interface using an RMII PHY (LAN8742A). It can be used for MAVLink over UDP/TCP, DDS/ROS2 integration, and other IP-based telemetry and command links.

## Logging

Dataflash logging is provided by the onboard 2 Gbit W25N01 NAND flash, accessed through LittleFS. The default backend is set with `LOG\_BACKEND\_TYPE = 4` (Block/Dataflash) for ArduPilot 4.6.x firmware. Beginning with ArduPilot 4.7, the equivalent default will be `LOG\_BACKEND\_TYPE = 1`.

## IMU Orientation and Position

The two ICM42688 IMUs are mounted with different rotations to decorrelate vibration:

* IMU1 (SPI3, ICM42688\_A): `ROTATION\_ROLL\_180\_YAW\_90`
* IMU2 (SPI2, ICM42688\_B): `ROTATION\_ROLL\_180\_YAW\_45`

The physical offsets of each IMU from the board origin are preset in firmware:

* IMU1: `INS\_POS1\_X = 0.00375`, `INS\_POS1\_Y = 0.0075`
* IMU2: `INS\_POS2\_X = 0.00775`, `INS\_POS2\_Y = 0.0075`

Fast sampling is enabled on both IMUs by default (`HAL\_DEFAULT\_INS\_FAST\_SAMPLE = 3`).

## Debug

A TC2030 JTAG/SWD footprint is provided. SWD pins are PA13 (SWDIO) and PA14 (SWCLK), compatible with ST-Link v2 and Black Magic Probe.

A **Boot0** button is provided on the top side of the board. Holding it during reset/power-up forces the STM32H753 into its built-in DFU bootloader, allowing recovery via USB if the ArduPilot bootloader has been overwritten or corrupted.

## Loading Firmware

Firmware for this board can be found at the [ArduPilot firmware server](https://firmware.ardupilot.org) in sub-folders labeled "HCR-565".

The board comes pre-installed with an ArduPilot-compatible bootloader, allowing the loading of \*.apj firmware files with any ArduPilot-compatible ground station. The APJ board ID is 1961.

## Further info

* [HC Robotics](https://www.hcrobo.com/)

