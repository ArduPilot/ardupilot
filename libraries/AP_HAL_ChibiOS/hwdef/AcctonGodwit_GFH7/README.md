# Accton FPV G-FH7

The FPV G-FH7 is a compact flight controller designed for fixed-wing, UAV,
and VTOL applications. It adopts an STM32H753 flight-management processor,
dual IMUs, an integrated barometer and compass, and solder-free connectors
for flexible system integration. The board supports ArduPilot, PX4,
Betaflight, and iNAV flight-control software.

Visit [Accton-IoT FPV G-FH7](https://www.accton-iot.com/godwit/g-fh7.html)
for more information.

![Accton FPV G-FH7](outlook.png "Accton FPV G-FH7")

![Accton FPV G-FH7 Front View](orientation_front.png "Accton FPV G-FH7 Front View")

![Accton FPV G-FH7 Back View](orientation_back.png "Accton FPV G-FH7 Back View")

## Specifications

### **Processor**

- STM32H753VIH6 (Arm Cortex-M7, 480MHz)

### **Sensors**

- TDK InvenSense ICM-42688-P
- STMicroelectronics LSM6DSK320X
- DPS368 Barometric Pressure Sensor
- IST8310 Geomagnetic Sensor

### **Power**

- Input voltage: up to 12S LiPo
- BEC output: 5 V/3 A and 12 V/3 A (supports power-saving mode)
- Static power consumption: 110 mA at 5 V
- microSD card for blackbox data logging

### **External ports**

- 1 CAN port
- 7 UARTs (6x available by connectors, 1x available by soldering pads)
- 9 PWM outputs: 8 motor outputs and 1 user-defined output
- 1 I2C port
- 3 ADC inputs (battery voltage, battery current, and analog RSSI)
- 2 LED indicators
- Software-based analog OSD
- Buzzer, video input, and video output interfaces
- USB Type-C with DFU button
- Ethernet RMII and PPS by connector (reserved for future use)

### **Size and Dimensions**

- 36 x 42 x 8.63 mm
- M4 mounting holes
- Weight: 10.6 g

## Where to Buy

- [Accton-IoT FPV G-FH7](https://www.accton-iot.com/godwit/g-fh7.html)
- [sales@accton-iot.com](mailto:sales@accton-iot.com)

## Pinout

![G-FH7 Pin Definition](pin_definition.png "Accton FPV G-FH7 Pin Definition")

## Interface Summary

| Interface | Function |
| --------- | -------- |
| `ESC` / `Ext ESC` | Connect the ESC and motor system. |
| `GPS` | Connect a GPS module. |
| `SBUS` / `ELRS` | Connect an RC receiver. |
| `TELEM` | Connect a telemetry radio or MAVLink device. |
| `CAN` | Connect CAN peripherals. |
| `I2C` | Connect external I2C sensors. |
| `VIDEO-IN` | Connect the analog video input. |
| `A-VTX` / `D-VTX` | Connect the supported video transmitter interface. |

## Wiring Diagram

![G-FH7 Wiring](wiring.png "Accton FPV G-FH7 Wiring")

## UART Mapping

| Serial# | Default protocol | Port | TX DMA | RX DMA | Connector role and notes |
| ------- | ---------------- | ---- | ------ | ------ | ------------------------ |
| `SERIAL0` | MAVLink2 | USB |  |  | USB connection |
| `SERIAL1` | MAVLink2 | `UART7` | No | No | TELEM connector |
| `SERIAL2` | MAVLink2 | `USART2` | No | No | External pad |
| `SERIAL3` | GPS | `USART3` | Yes | Yes | GPS connector |
| `SERIAL4` | MAVLink2 | `UART4` | Yes | Yes | ELRS receiver; alternate RC input after disabling `SERIAL5` RCIN. |
| `SERIAL5` | RCIN | `UART5` | No | No | SBUS connector; `UART5_RX` receives SBUS and is also wired to D-VTX pin 6. `UART5_TX` is shared by SBUS pin 4 and the A-VTX `UART5_TX` pin. |
| `SERIAL6` | MSP DisplayPort | `UART8` | No | No | D-VTX connector. |
| `SERIAL7` | ESC Telemetry | `USART1` | No | No | ESC connector `UART1_RX` pin only; TX is not available on a connector. |
| `SERIAL8` | MAVLink2 | USB (second interface) |  |  | Second USB virtual port |

## PWM Output

![G-FH7 Motor/ESC Wiring](motor_esc_wiring.png)

`PWM1`-`PWM8` are motor outputs and support bidirectional DShot. `PWM9` is on Ext ESC connector pin `9` and defaults to NeoPixel serial LED output.

PWM outputs are arranged in three timer groups: PWM1-PWM4 (TIM1), PWM5-PWM8 (TIM8), and PWM9 (TIM2). All outputs in a timer group must use the same output protocol. If a group uses DShot, every output in that group must use DShot.

## RC Input

The default RC input is SBUS on UART5 (`SERIAL5`). Connect the receiver to the SBUS connector.

To use an ELRS receiver instead, connect it to the ELRS receiver connector on UART4 (`SERIAL4`). Using Mission Planner or QGroundControl over USB or another telemetry link, set `SERIAL4_PROTOCOL` to `23` (RC Input) and `SERIAL5_PROTOCOL` to `-1` (Disabled), then reboot the flight controller. Only one serial port can be configured for RC input at a time.

To switch back to SBUS, set `SERIAL5_PROTOCOL` to `23`, restore `SERIAL4_PROTOCOL` to its intended setting, then reboot.

For setup details on common RC systems, including CRSF and ELRS, see the [common-rc-systems documentation](https://ardupilot.org/plane/docs/common-rc-systems.html).

![G-FH7 Radio](radio.png "Accton FPV G-FH7 Radio")

## OSD Support

The on-board SPI OSD uses an STM32G431 running AT7456E-compatible firmware and defaults to `OSD_TYPE=1` for analog OSD. UART8 (`SERIAL6`) defaults to MSP DisplayPort, and `OSD_TYPE2` defaults to `5` (MSP DisplayPort), so the analog OSD and a DisplayPort OSD run simultaneously.

Note: if `SERIAL6_PROTOCOL` is ever changed from MSP DisplayPort, `OSD_TYPE2` must be set to `0`, or a pre-arm failure will result.

## Analog VTX Support

The A-VTX connector `UART5_TX` pin is shared with the SBUS connector. SmartAudio/Tramp and SBUS cannot both run on UART5, so VTX control is not configured by default. To use VTX control:

1. Move RC input to the ELRS connector: set `SERIAL4_PROTOCOL` to `23` (RC Input).
2. Set `SERIAL5_PROTOCOL` to `37` (SmartAudio) or `44` (Tramp).
3. Reboot the flight controller.

## Digital VTX Support

The D-VTX connector uses UART8 (`SERIAL6`; `UART8_TX` and `UART8_RX` pins) for its digital link and is intended for digital VTX OSD. The firmware defaults to `SERIAL6_PROTOCOL=42` (MSP DisplayPort) and `OSD_TYPE2=5`, which enables DisplayPort OSD on the second OSD instance.

If `SERIAL6_PROTOCOL` is changed from MSP DisplayPort, set `OSD_TYPE2` to `0` to avoid a pre-arm failure.

## VTX Power Control

The VTX 12V rail is controlled by Relay 1 on GPIO 81. It is off while the bootloader is running and is enabled by default once the flight firmware starts. Use Relay 1 to turn VTX power off or on.

The `12V` pin of the VIDEO-IN connector is on the same Relay 1 switched rail as the A-VTX and D-VTX connectors, so turning Relay 1 off also removes power from a camera connected to VIDEO-IN.

WARNING: the `12V` pin on the VIDEO-IN connector is 12V, not 5V. Do not connect a 5V-only camera to it.

## GPIOs and Analog Inputs

### GPIO Numbers

The GPIO numbers of the PWM outputs and the VTX power switch are:

| Pin | GPIO Number |
| --- | ----------- |
| PWM1 | 50 |
| PWM2 | 51 |
| PWM3 | 52 |
| PWM4 | 53 |
| PWM5 | 54 |
| PWM6 | 55 |
| PWM7 | 56 |
| PWM8 | 57 |
| PWM9 | 58 |
| VTX PWR | 81 |

### RSS Analog Input

The `RSS` pad on the back face is mapped to `RSSI_ANA_PIN=5`. To use an analog receiver RSSI voltage, set `RSSI_TYPE` to `1` (AnalogPin); RSSI is disabled by default.

### UART2 Solder Pads

The `T2` and `R2` solder pads are `SERIAL2`: `T2` is USART2 TX and `R2` is USART2 RX. `SERIAL2` defaults to MAVLink2 and has no TX or RX DMA.

### External Five-Pad Test Points

The top face has two unpopulated five-pad test/debug groups for manufacturing, debugging, or hardware verification:

- `R V G C D`: STM32H753 (H7) test/debug pads.
- `R C D G V`: OSD co-processor test/debug pads.

They are not general user interfaces. Do not connect wiring or apply power, and do not use them as GPIO, UART, or I2C interfaces. Individual pad nets and functions are not publicly defined.

## Power Connection and Battery Monitoring

The board has an internal voltage sensor and a current sensor input on the ESC connector (`C` pin) for the ESC's current sensor. The voltage sensor can handle up to 12S LiPo batteries.

The default battery parameters are:

- BATT_MONITOR = 4
- BATT_VOLT_PIN = 10
- BATT_CURR_PIN = 9
- BATT_VOLT_MULT = 17.0
- BATT_AMP_PERVLT = 40.0 (will need to be adjusted for whichever current sensor is attached)

## Compass

The G-FH7 has a built-in IST8310 compass. Due to potential interference, the autopilot is usually used with an external I2C compass as part of a GPS/Compass combination, connected to the GPS or I2C connector.

![G-FH7 GPS](gps.png "Accton FPV G-FH7 GPS")

## SD Card

The board supports a microSD card for blackbox and flight-log storage.

![FPV G-FH7 SD Card](sdcard.png "FPV G-FH7 SD Card")

## Loading Firmware

The G-FH7 ships with an ArduPilot-compatible bootloader. To load ArduPilot firmware, connect the board to a ground control station (Mission Planner or QGroundControl or MAVProxy, plus others can be used) and use it to load the "\<vehicle\>.apj" file from the `AcctonGodwit_GFH7` folder on the [ArduPilot firmware server](https://firmware.ardupilot.org).

For recovery, hold the BOOT button (next to the USB-C connector) while connecting USB, then use STM32CubeProgrammer to load the "***_with_bl.hex" file.

## More Information and Support

- [Accton-IoT FPV G-FH7 product page](https://www.accton-iot.com/godwit/g-fh7.html)
- [FPV G-FH7 datasheet](https://www.accton-iot.com/godwit/assets/doc/DS-Godwit%20FPV%20G-FH7.pdf)
- [sales@accton-iot.com](mailto:sales@accton-iot.com)
- [support@accton-iot.com](mailto:support@accton-iot.com)
