# Atlas Control Flight Controller

The Atlas Control flight controller
[AtlasFlight documentation](https://altasflight.github.io/atlas-docs/)

## Features

- STM32H743 microcontroller
- Three IMUs, ICM-42688-P, ICM-20689 and ICM-20689
- internal heater for IMU temperature control
- two MS5611 SPI barometers
- internal RM3100 compass
- microSD card slot
- 4 UARTs plus USB
- ADSB receiver connected to SERIAL5
- 14 PWM outputs
- I2C and CAN ports
- External Buzzer
- builtin RGB LED
- voltage monitoring for servo rail and Vcc
- two dedicated power input ports for external power bricks
- external USB connectors (micro USB and JST GH)

## Pinout

![AtlasControl Board](img.png "Atlas Control")

- 14 PWM servo outputs (12 support DShot)

- Analog/ PWM RSSI input

- 2 GPS ports(GPS and UART4 ports)

- 4 I2C buses(Two external I2C ports)

- 2 CAN bus ports

- 2 Power ports(Power A is an analog PMU interface, Power C is a DroneCAN PMU interface)

- 2 ADC inputs

- 1 USB-C port

## UART Mapping

| Port | UART | Protocol | TX DMA | RX DMA |
| :--- | :--- | :--- | :---: | :---: |
| 0 | USB | MAVLink2 | ✘ | ✘ |
| 1 | USART2 | MAVLink2 | ✔ | ✔ |
| 2 | USART6 | MAVLink2 | ✔ | ✔ |
| 3 | USART1 | GPS | ✘ | ✘ |
| 4 | UART4 | GPS | ✘ | ✘ |
| 6 | UART7 | None | ✘ | ✘ |
| 7 | USB | SLCAN | ✘ | ✘ |

> **Note:** SERIAL 1 and 2 have RTS/CTS flow control capability.

## Connectors

Unless noted otherwise all connectors are JST GH 1.25mm pitch

### TELEM1, TELEM2 ports

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 (red) | VCC | +5V |
| 2 (blk) | TX (OUT) | +3.3V |
| 3 (blk) | RX (IN) | +3.3V |
| 4 (blk) | CTS | +3.3V |
| 5 (blk) | RTS | +3.3V |
| 6 (blk) | GND | GND |

### ADC

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | ADC IN | up to +3.3V |
| 2 | GND | GND |
| 3 | 3.3V | +3.3V |

### FMU and IO SWD

When the case is removed there are two SWD connectors, one for FMU and
the other for IOMCU. The IO SWD connector is the one closer to the
servo rail. The GND pin of both connectors is the one furthest from
the servo rail.

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX | +3.3 |
| 3 | RX | +3.3 |
| 4 | SWDIO | +3.3 |
| 5 | SWCLK | +3.3 |
| 6 | GND | GND |

### CAN1&2

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 (red) | VCC | +5V |
| 2 (blk) | CAN_H | +12V |
| 3 (blk) | CAN_L | +12V |
| 4 (blk) | GND | GND |

### POWER_A

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 (red) | VCC | +5V |
| 2 (red) | VCC | +5V |
| 3 (blk) | CURRENT | up to +3.3V |
| 4 (blk) | VOLTAGE | up to +3.3V |
| 5 (blk) | GND | GND |
| 6 (blk) | GND | GND |

### POWER_C

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 (red) | VCC | +5V |
| 2 (red) | VCC | +5V |
| 3 (blk) | CAN_H | up to +3.3V |
| 4 (blk) | CAN_L | up to +3.3V |
| 5 (blk) | GND | GND |
| 6 (blk) | GND | GND |

### USB

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 (red) | VCC | +5V |
| 2 (blk) | D_plus | +3.3V |
| 3 (blk) | D_minus | +3.3V |
| 4 (blk) | GND | GND |
| 5 (blk) | BUZZER | battery voltage |
| 6 (blk) | Boot/Error LED |  |

## RC Input

RC input is configured on the RCIN pin, at one end of the servo rail,
marked RCIN in the above diagram. All ArduPilot supported unidirectional
RC protocols can be input here including PPM. For bi-directional or half-duplex
protocols, such as CRSF/ELRS a full UART will have to be used.
See [ArduPilot RC documentation](https://ardupilot.org/plane/docs/common-rc-systems.html)

## PWM Output

The 14 PWM outputs are in 4 groups: Each group must be the same protocol (ie PWM or DShot or Serial LED, etc.):

- Outputs 1, 2, 3 and 4 in group1 (supports Bi-directional DShot)
- Outputs 5, 6, 7 and 8 in group2 (supports Bi-directional DShot)
- Outputs 9, 10, 11 and 12 in group3 (supports DShot)
- Outputs 13 and 14 in group4 (PWM only, no DMA)

## Battery Monitoring

The board has two dedicated power monitor ports on 6 pin connectors.

The board is supplied with an analog power module. Analog monitoring is set as the default configuration.
The module is rated for up to 60V (14S) and 60A continuous current.

- **BATT_MONITOR**: 4 (Analog Voltage and Current)
- **BATT_VOLT_PIN**: 16
- **BATT_CURR_PIN**: 17
- **BATT_VOLT_MULT**: 18.000
- **BATT_AMP_PERVLT**: 24.000
- **BATT_VLT_OFFSET**: -0.1

## Compass

The Atlas Control has an RM3100 builtin compass, and you can attach an external compass using I2C on the SDA and SCL connector.

## GPIOs

The 14 outputs can be used as GPIOs (relays, buttons, RPM etc). To use them you need to set the output’s `SERVOx_FUNCTION` to -1. See [GPIOs](https://ardupilot.org/copter/docs/common-gpios.html) page for more information.
The numbering of the GPIOs for use in the PIN parameters in ArduPilot is:

| Output | GPIO Number |
| :--- | :--- |
| PWM1 (M1) | 50 |
| PWM2 (M2) | 51 |
| PWM3 (M3) | 52 |
| PWM4 (M4) | 53 |
| PWM5 (M5) | 54 |
| PWM6 (M6) | 55 |
| PWM7 (M7) | 56 |
| PWM8 (M8) | 57 |
| PWM9 (M9) | 58 |
| PWM10 (M10) | 59 |
| PWM11 (M11) | 60 |
| PWM12 (M12) | 61 |
| PWM13 (M13) | 62 |
| PWM14 (M14) | 63 |
| **Analog RSSI (PF12)** | **92** |

## IMU Heater

The IMU heater in the Atlas Control can be controlled with the
BRD_HEAT_TARG parameter, which is in degrees C.

## Loading Firmware

Firmware for these boards can be found at the [ArduPilot firmware server](https://firmware.ardupilot.org) in sub-folders labeled “Atlas-Control”.

Initial firmware load can be done with DFU by plugging in USB with the
boot button pressed. Then you should load the "xxx_bl.hex"
firmware, using your favorite DFU loading tool.

Subsequently, you can update the firmware with Mission Planner.
