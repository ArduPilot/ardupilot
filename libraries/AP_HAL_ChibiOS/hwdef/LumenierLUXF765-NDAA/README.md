# Lumenier LUX F765 Flight Controller - NDAA

The Lumenier LUX F765 NDAA flight controller is sold by
[GetFPV](https://www.getfpv.com/lumenier-lux-f765-flight-controller-ndaa.html).

![LUX F765 NDAA Slim](LUXF765_NDAA_Slim_Graphic.png "LUX F765 NDAA Slim")

## Features

- Processor: MCU STM32F765, 216MHz, 512KB RAM, 2MB Flash
- ICM42688 IMU
- BMP280 Barometer
- microSD Card Slot
- 12 PWM Outputs
- I2C Ports for External Sensors
- CANbus Support
- 8 UART Ports
- Buzzer Control
- LED Strip Control
- Analog Current Sensor Input
- Analog Battery Sensor Input
- USB Type-C (2.0)
- Blackbox Storage - SD Card and Flash (128Mbit/16Mbyte)
- Camera Control Output
- AT7456E OSD
- 10V Regulator for VTX Power
- 5V Regulator for Accessories
- Supported Firmware - Betaflight, Ardupilot, and PX4
- NDAA compliant
- Power Supply: 3S to 6S Battery Voltage

## Pinout

![LUX F765 NDAA Top](LUXF765_NDAA_TOP.png "LUX F765 NDAA Top")
![LUX F765 NDAA Bottom](LUXF765_NDAA_BOTTOM.png "LUX F765 NDAA Bottom")

## UART Mapping

- SERIAL0 -> USB
- SERIAL1 -> USART1 (Telem1, DMA enabled)
- SERIAL2 -> UART5  (Telem2, DMA enabled)
- SERIAL3 -> USART3 (GPS1, TX DMA enabled)
- SERIAL4 -> UART8  (GPS2)
- SERIAL5 -> USART2 (ESC Telemetry)
- SERIAL6 -> UART4  (DisplayPort)
- SERIAL7 -> UART7  (RCinput, DMA enabled)
- SERIAL8 -> USART6 (Spare)

## SPI Mapping

   | SPI | Device |
| --- | --- |
| 1 | ICM42688 |
| 2 | W25Q128JV Flash |
| 3 | AT7456E |
| 4 | SD Card |

## Connectors

All connectors are JST SH 1.0mm pitch EXCEPT for the CANbus port, which is JST GH 1.25mm pitch.

### ESC #1 Port

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | VBAT |
| 2 | GND | GND |
| 3 | CURRENT | +3.3V |
| 4 | TELEMETRY | +3.3V |
| 5 | MOTOR 1 (TIM2_CH1) | +3.3V |
| 6 | MOTOR 2 (TIM2_CH2) | +3.3V |
| 7 | MOTOR 3 (TIM2_CH3) | +3.3V |
| 8 | MOTOR 4 (TIM2_CH4) | +3.3V |

### ESC #2 Port

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | NOT CONNECTED | NOT CONNECTED |
| 2 | GND | GND |
| 3 | NOT CONNECTED | NOT CONNECTED |
| 4 | TELEMETRY | +3.3V |
| 5 | MOTOR 5 (TIM4_CH1) | +3.3V |
| 6 | MOTOR 6 (TIM4_CH2) | +3.3V |
| 7 | MOTOR 7 (TIM4_CH3) | +3.3V |
| 8 | MOTOR 8 (TIM4_CH4) | +3.3V |

### GPS port

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | +5V | +5V |
| 2 | TX3 | +3.3V |
| 3 | RX3 | +3.3V |
| 4 | I2C3 SCL | +3.3V |
| 5 | I2C3 SDA | +3.3V |
| 6 | GND | GND |

### HD VTX port

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | +10V | +10V |
| 2 | GND | GND |
| 3 | TX4 | +3.3V |
| 4 | RX4 | +3.3V |
| 5 | GND | GND |
| 6 | RX7 | +3V3 |

### Receiver Port

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | +5V | +5V |
| 2 | GND | GND |
| 3 | RX7 | +3.3 |
| 4 | TX7 | +3.3V |

### CANbus Port

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | +5V | +5V |
| 2 | GND | GND |
| 3 | CAN_H | +5V |
| 4 | CAN_L | +5V |

## PWM Output

The Lumenier LUX F765 NDAA supports 12 PWM outputs and a serial LED PWM output.
All outputs are DShot capable. Outputs 1-4 are bi-directional DShot capable.
The 8 main PWM outputs are labeled M1 through M8.
The 4 auxiliary outputs are labeled S1 through S4.

The 8 main PWM outputs are in 2 groups:

- PWM 1 through 4 (M1 - M4) are in TIM2
- PWM 5 through 8 (M5 - M8) are in TIM4

The auxiliary PWM outputs are grouped as follows:

- PWM 9/10 (S1/S2) are in a group
- PWM 11/12 (S3/S4) are in a group
- PWM 13 (LED) is in a group

## RC Input

Bi-directional RC inputs like CRSF/ELRS are supported on UART7 (Serial7).
UART7 (Serial7) will also support all unidirectional RC protocols.

## OSD Support

The LUX F765 - NDAA is equipped with an onboard AT7456E OSD.
The AT7456E communicates with the flight controller on SPI3.

## Camera Control

The LUX F765 - NDAA has a camera control output on PE10, which corresponds to GPIO 82. Additionally, RELAY3 is pre-configured to control GPIO 82.

## Analog inputs

The LUX F765 NDAA has 2 analog inputs:

- PC2 -> Battery Current
- PC3 -> Battery Voltage

## Battery Monitor

The LUX F765 - NDAA has an internal voltage sensor and connections on the ESC connector
for an external current sensor input. The voltage sensor can handle up to an 8S battery.

The default parameters are as follows:

- :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
- :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 12
- :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 13
- :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 10.1
- :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 17.0 (will need to be adjusted for whichever current sensor is attached)

## Compass

The LUX F765 - NDAA does not have a built-in compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Firmware

Firmware for the LUX F765 - NDAA can be found [here](https://firmware.ardupilot.org) in sub-folders labeled “LumenierLUXF765-NDAA".

## Loading Firmware

The LUX F765 - NDAA does not come with ArduPilot firmware pre-installed. Use instructions here to load ArduPilot the first time :ref:`common-loading-firmware-onto-chibios-only-boards`.

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favorite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
