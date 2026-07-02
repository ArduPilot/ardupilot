# Lectron Pi5 Autopilot

![Lectron Pi5 Autopilot](lectron_pi5_autopilot.png)

The Lectron Pi5 Autopilot is an integrated flight control and companion computing platform that combines a Pixhawk FMUv6X compatible flight controller with a Raspberry Pi Compute Module 5 on a single unified board. Designed by [Lectron](https://lectrontech.com/), it maintains a clear separation between safety-critical real-time control and high-level computing workloads.

The FMU follows the Pixhawk FMUv6X open standard with triple-redundant IMUs and dual barometers on separate buses, enabling seamless sensor failover. An independent STM32F103 IO processor handles R/C and PWM outputs in isolation from the main FMU core.

## Features

- STM32H753 FMU processor (Arm Cortex-M7, 480 MHz, 2 MB flash, 1 MB RAM)
- STM32F103 IO processor for R/C and PWM output handling
- Triple redundancy: 3x IMU sensors and 2x barometers on separate buses
- Isolated sensor domains with independent power control per sensor set
- Integrated Raspberry Pi Compute Module 5 (CM5)
- Dedicated 100-Mbps FMU Ethernet (LAN8742AI PHY) and 1-Gbps CM5 Ethernet
- M.2 Key-M (2230/2242) slot for NVMe SSD or AI accelerator
- 8-position DIP switch for boot mode and peripheral configuration
- Reverse polarity protection and shared power regulation bus
- INA238 onboard voltage monitoring

## Processors and Sensors

- **FMU Processor:** STM32H753IIK6TR (32-bit Arm Cortex-M7, 480 MHz, 2 MB flash, 1 MB RAM)
- **IO Processor:** STM32F103 (32-bit Arm Cortex-M3, 72 MHz, 64 KB SRAM)
- **Onboard sensors:**
  - Accel/Gyro: ICM-42670-P (SPI)
  - Barometer: BMP390 (I2C)
  - FRAM (SPI)
  - EEPROM (I2C)
- **Sensor board (IMU-01, included with Lectron Pi5 Autopilot bundle):**
  - Accel/Gyro: ICM-42670-P (SPI)
  - Accel/Gyro: BMI270 (SPI)
  - Barometer: BMP390 (I2C)
  - Magnetometer: BMM350 (I2C)
  - EEPROM (I2C)

### Companion Computer

- **Supported module:** Raspberry Pi Compute Module 5 (CM5 / CM5 Lite)
- **Connection:** CM5 board-to-board connector

## Electrical Specifications

- **Power input:** 12-26 VDC (3S-6S LiPo), XT30 connector
- **Overcurrent protection:** 5 A maximum
- **FMU and CM5 shared rail:** 5 V regulated bus
- **Per-port output limit:** 1.5 A total across FMU and CM5 peripheral 5 V rails
- **Protection:** reverse polarity, overcurrent
- **Voltage monitoring:** INA238 (I2C)
- **Servo rail sensing limit:** 16 VDC

## Interfaces

### FMU

- 8x FMU (AUX) PWM outputs - DShot capable
- 8x IO (MAIN) PWM outputs
- RC input: PPM, S.BUS (DSM not supported)
- RSSI analog input
- S.BUS output
- 2x GPS ports
  - GPS-1 (full): UART + I2C + safety switch + LED + buzzer (10-pin JST-GH)
  - GPS-2 (basic): UART + I2C (6-pin JST-GH)
- 2x TELEM serial ports with hardware flow control (UART7, UART5)
  - TELEM3 (USART2) bridged by default to CM5 UART3
- 1x UART4 + I2C3 (combined external port)
- 1x I2C2 external
- 1x SPI6 with 2x chip-select and 2x data-ready lines (11-pin JST-GH)
- 1x CAN bus
- 1x USB 2.0 Type-C
- Micro SD card slot
- 1x 4-pin JST-GH Ethernet port - 100 Mbps (LAN8742AI PHY)
- FMU debug port + IO debug port (10-pin SM10B-SRSS, SWD + UART)

### CM5

- 2x CSI camera interfaces (22-pin FFC, 0.5 mm pitch, 4-lane MIPI)
- UART2 (combined on GPIO/UART connector)
- I2C-1, I2C-3 (combined on I2C connector)
- SPI1 with 3x chip-select
- GPIO22-GPIO27 (10-pin SM10B-GHS, 3.3 V logic)
- 4x PWM (combined on SPI/PWM connector)
- 1x CAN (MCP2515 via SPI1-CS0)
- 2x USB 3.0 Type-C
- 1x USB 2.0 Micro (debug / CM5 flashing)
- 1x Micro HDMI
- 1x 8-pin 1-Gbps Ethernet port
- M.2 Key-M 2230/2242 (PCIe Gen2 x1) - NVMe SSD or Hailo-8 AI accelerator
- 8-position DIP switch (WiFi/BT disable, boot mode, power control)
- 4-pin JST fan connector (with tacho)

## Where to Buy

Order from [Lectron Technology](https://lectrontech.com/pi5autopilot/).

## Assembly

The Lectron Pi5 Autopilot ships as a kit. Assembly requires the following components:

- CM5-FMU baseboard
- Raspberry Pi Compute Module 5 (CM5 or CM5 Lite)
- MicroSD card(s)
- M.2 Key-M module (2230/2242, optional: Hailo or SSD)
- Bottom case
- Top case (heatsink + fan)
- IMU sensor board (IMU-01)
- Fasteners: 4x M2x4 mm (IMU board), 5x M2x10 mm (case)

For full photo-illustrated steps, see the [Lectron Pi5 Assembly Guide](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/assembly/).

## UART Mapping

| UART   | Port                                        |
| ------ | ------------------------------------------- |
| USART1 | GPS1 - GPS, Mag, Buzzer, Safety Switch, LED |
| USART2 | TELEM3 - bridged to CM5 UART3 by default    |
| UART3  | FMU Debug Console                           |
| UART4  | UART4 and I2C3 (external)                   |
| UART5  | TELEM2 - hardware flow control              |
| USART6 | IO Chip (STM32F103 communication)           |
| UART7  | TELEM1 - hardware flow control              |
| UART8  | GPS2 - GPS, Mag                             |

## PWM Outputs

The board has 16 PWM outputs: 8 IO (MAIN) outputs driven by the STM32F103C8T6 IO co-processor and 8 FMU (AUX) outputs driven by the STM32H753IIK6 FMU.

### IO - STM32F103C8T6

IO outputs support PWM only. DShot is not supported on any IO output. All outputs within the same timer group must use the same protocol and update rate.

| IO Channel | MCU Pin | Timer / Channel | DShot |
| ---------- | ------- | --------------- | ----- |
| IO_CH 1    | PA0     | TIM2_CH1        | No    |
| IO_CH 2    | PA1     | TIM2_CH2        | No    |
| IO_CH 3    | PB8     | TIM4_CH3        | No    |
| IO_CH 4    | PB9     | TIM4_CH4        | No    |
| IO_CH 5    | PA6     | TIM3_CH1        | No    |
| IO_CH 6    | PA7     | TIM3_CH2        | No    |
| IO_CH 7    | PB0     | TIM3_CH3        | No    |
| IO_CH 8    | PB1     | TIM3_CH4        | No    |

### FMU - STM32H753IIK6

FMU_CH 1-6 support DShot and Bidirectional DShot. FMU_CH 7-8 do not support DShot (TIM12 has no DMA). All outputs within the same timer group must use the same protocol and update rate.

| FMU Channel | MCU Pin | Timer / Channel | DShot |
| ----------- | ------- | --------------- | ----- |
| FMU_CH 1    | PI0     | TIM5_CH4        | Yes   |
| FMU_CH 2    | PH12    | TIM5_CH3        | Yes   |
| FMU_CH 3    | PH11    | TIM5_CH2        | Yes   |
| FMU_CH 4    | PH10    | TIM5_CH1        | Yes   |
| FMU_CH 5    | PD13    | TIM4_CH2        | Yes   |
| FMU_CH 6    | PD14    | TIM4_CH3        | Yes   |
| FMU_CH 7    | PH6     | TIM12_CH1       | No    |
| FMU_CH 8    | PH9     | TIM12_CH2       | No    |

## Building Firmware

```sh
python3 waf configure --board Lectron-Pi5-H7 --bootloader
python3 waf bootloader

python3 waf configure --board Lectron-Pi5-H7
python3 waf copter
```

## Loading Firmware

### FMU Firmware

See the [Lectron Pi5 Autopilot FMU Installation Guide](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/setup/#fmu-firmware-installation) for installation steps.

### CM5 Operating System

See the [Lectron Pi5 Autopilot CM5 Installation Guide](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/setup/#cm5-installation) for installation steps.

## Debug Port

The FMU debug port runs on UART3 (`/dev/ttyS2`). The pinout complies with the Pixhawk Debug Full interface defined in the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) (JST SM10B-SRSS connector).

| Pin      | Signal            | Volt  |
| -------- | ----------------- | ----- |
| 1 (red)  | FMU VDD           | +3.3V |
| 2 (blk)  | Console TX (OUT)  | +3.3V |
| 3 (blk)  | Console RX (IN)   | +3.3V |
| 4 (blk)  | SWDIO             | +3.3V |
| 5 (blk)  | SWDCLK            | +3.3V |
| 6 (blk)  | SPI6_SCK_EXTERNAL | +3.3V |
| 7 (blk)  | NFC GPIO          | +3.3V |
| 8 (blk)  | PH11              | +3.3V |
| 9 (blk)  | nRST              | +3.3V |
| 10 (blk) | GND               | GND   |

## Further Information

- [Lectron Pi5 Autopilot Documentation](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/)
- [Pinout and Connector Details](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/pinout/)
- [Assembly Guide](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/assembly/)
- [Firmware Installation Guide](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/setup/#fmu-firmware-installation)
- [FMU and CM5 Communication](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/fmu-cm5-comm/)
- [CM5 GPIO Guide](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/cm5-gpio/)
- [Camera Setup](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/cam1-setup/)
- [Hailo-8 AI Accelerator Integration](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/hailo-setup/)
- [RealSense Integration](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/realsense-setup/)
- [Pixhawk Autopilot FMUv6X Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-012%20Pixhawk%20Autopilot%20v6X%20Standard.pdf)
- [Pixhawk Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf)
- [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)
