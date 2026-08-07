# Agam MegH7 v1.1 Flight Controller

![Agam MegH7](agam-robotics_megh7-top.png)

## Introduction

The **Agam MegH7 v1.1** is a high-performance flight controller built around the
**STM32H743** processor, designed for modern FPV, autonomous UAV, and robotics
platforms. It features a fully solderless, plug-and-play design for clean and
reliable builds while still providing exposed pads for advanced custom
configurations.

An onboard **10 V BEC** powers HD digital video systems such as the DJI Air
Unit, Caddx Vista, and similar transmitters while also supporting analog FPV
systems. The integrated Betaflight OSD provides real-time flight information
including battery voltage, flight time, RSSI, SmartAudio controls, warnings,
and other telemetry.

The MegH7 includes seven dedicated UARTs, a full-size MicroSD card slot for
Blackbox logging, CAN connectivity for advanced peripherals, onboard IMU and
barometer sensors, and multiple plug-and-play connectors for GPS, telemetry,
receivers, digital VTX, LEDs, buzzers, SPI, and I²C devices.

Supporting **PX4**, **ArduPilot**, **Betaflight**, and **INAV**, the Agam MegH7
is suitable for FPV aircraft, autonomous UAVs, robotics, and research
applications.

### Package Contents

- Agam MegH7 v1.1 Flight Controller
- 16 GB MicroSD Card
- Cable Set

### Included Accessories

- 4 × M3 silicone vibration grommets
- 1 × JST-SH 8-pin ESC cable (150 mm)
- 2 × JST-SH 6-pin DJI Air Unit cables (80 mm & 150 mm)

For OEM, B2B, and bulk orders, contact **<info@agamrobotics.com>**.

## Hardware Specifications

### Processor

- STM32H743
- ARM Cortex-M7
- 480 MHz

### Sensors

- IMU: ICM45686
- Barometer: BMP390

### Onboard Components

- AT7456E OSD
- Full-size MicroSD slot
- Optional SPI Flash (W25Q128JVP)
- USB Type-C

### Interfaces

- 7 × UART
- 1 × CAN
- SPI
- I²C
- 9 PWM Outputs

### Blackbox Logging

- Full-size MicroSD Card
- Optional SPI Flash

## Supported Devices

The Agam MegH7 is compatible with numerous ArduPilot-supported peripherals.

### GPS

- u-blox GPS
- M10 GPS
- RTK GPS

### Radio Receivers

- ELRS
- Crossfire
- SBUS
- PPM

### CAN Devices

- Agam CAN-GNSS
- Agam CAN-FloRange
- DroneCAN peripherals

### Video Systems

- DJI Air Unit
- Caddx Vista
- Analog FPV VTX

### More Sensors

- Optical Flow
- Rangefinders
- External Magnetometers
- External Barometers
- SPI Sensors
- I²C Sensors
- UART Sensors

### Other Devices

- Telemetry Radios
- LED Strips
- Buzzers
- ESCs
- 4-in-1 ESC

## Electrical Specifications

| Parameter     | Specification     |
| ------------- | ----------------- |
| MCU           | STM32H743         |
| Input Voltage | 2S–8S LiPo        |
| USB           | Type-C            |
| 5 V BEC       | 5 V @ 3 A         |
| 10 V BEC      | 10 V @ 3 A        |
| UART          | 7                 |
| CAN           | 1                 |
| PWM Outputs   | 9                 |
| Blackbox      | Full-size MicroSD |
| SPI Flash     | Optional          |

### Status Indicators

- 2 × User LEDs
- 3.3 V Power LED
- 10 V VTX Indicator

## Hardware Setup

Typical hardware setup:

1. Install vibration isolation grommets.
2. Mount the flight controller.
3. Connect the 4-in-1 ESC.
4. Connect the radio receiver.
5. Connect GPS.
6. Connect Digital or Analog VTX.
7. Connect telemetry if required.
8. Connect CAN peripherals.
9. Insert the MicroSD card.
10. Connect USB Type-C for firmware installation.

## Connectors

### Main Connectors

| Connector   | Description                 |
| ----------- | --------------------------- |
| USB Type-C  | Firmware & Configuration    |
| MicroSD     | Blackbox Logging            |
| ESC         | JST-SH 8-pin                |
| GPS         | JST-GH 6-pin                |
| Telemetry   | JST-GH 6-pin (Flow Control) |
| Digital VTX | JST-SH 6-pin                |
| Receiver    | JST-GH 4-pin                |
| CAN         | JST-GH 4-pin                |
| SPI         | JST-SH 8-pin                |
| I²C         | JST-SH 4-pin                |
| LED/Buzzer  | JST-SH 4-pin                |

## Exposed Pads

![Exposed Pads for custom applications](agam-robotics_megh7-bot.png)

- 7x UART Ports
- 2x I²C Ports
- 8x PWM Outputs
- Analog Camera
- Analog VTX
- RSSI
- A1
- A2
- LED
- Buzzer
- PINIO P2–P4
- GPIO C13, C14, C15

## Features

- STM32H743 @ 480 MHz
- ICM45686 IMU
- BMP390 Barometer
- AT7456E OSD
- USB Type-C
- 7 UARTs
- CAN Interface
- Full-size MicroSD Blackbox
- Optional SPI Flash
- 9 PWM Outputs
- 5 V & 10 V BEC
- Digital FPV Support
- Analog FPV Support
- Plug-and-play connectors
- ELRS Compatible
- Crossfire Compatible
- GPS Ready
- PX4 Compatible
- ArduPilot Compatible
- Betaflight Compatible
- INAV Compatible

## Applications

The Agam MegH7 is designed for:

- FPV Racing
- Freestyle Drones
- Long Range FPV
- Autonomous UAVs
- VTOL Development
- Robotics
- Research Platforms
- Drone Education
- Mapping
- Surveying
- Inspection
- Indoor Navigation
- CAN-based Robotics

## Where to Buy

The Agam MegH7 v1.1 Flight Controller is available directly from
**Agam Robotics**.
<https://www.agamrobotics.com/shop>

## For Further Updates

<https://www.agamrobotics.com/product-page/agam-megh7>
