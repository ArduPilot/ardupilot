# VUAV-V7pro Flight Controller

The VUAV-V7pro  flight controller is manufactured and sold by [V-UAV](http://www.v-uav.com/).

## Features

- STM32H743 microcontroller
- Three IMUs: ADIS16470,ICM42688,ICM42688
- Internal vibration isolation for IMUs
- Internal RM3100 SPI magnetometer
- Internal two MS5611 SPI barometer
- Internal RGB LED
- MicroSD card slot port
- 1 Analog power port
- 1 CAN power port
- 5 UARTs and 1 USB ports
- 1 RS232 port
- 14 PWM output ports
- 4 I2C and 2 CAN ports
- Safety switch port
- Buzzer port
- RC IN port

## Pinout

![VUAV-V7pro-interface.png](VUAV-V7pro-interface.png)

![VUAV-V7pro-pinout.png](VUAV-V7pro-pinout.png)

## UART Mapping

- SERIAL0 -> USB
- SERIAL1 -> UART2 (Telem1) (DMA enabled)
- SERIAL2 -> UART6 (Telem2) (DMA enabled)
- SERIAL3 -> UART1 (GPS1)
- SERIAL4 -> UART3 (GPS2)
- SERIAL5 -> UART8 (USER) TX only on pin, RX is tied to RCIN
- SERIAL6 -> UART4 (RS232)
- SERIAL7 -> USB2 (virtual port on same connector)
- SERIAL8 -> UART7 (DEBUG)

The Telem1,Telem2 port has RTS/CTS pins, the other UARTs do not have RTS/CTS.

## Connectors

### TELEM1 ,TELEM2 port

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX (OUT) | +3.3V |
| 3 | RX (IN) | +3.3V |
| 4 | CTS | +3.3V |
| 5 | RTS | +3.3V |
| 6 | GND | GND |

### GPS1/I2C4 port

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX (OUT) | +3.3V |
| 3 | RX (IN) | +3.3V |
| 4 | SCL I2C4 | +3.3V (pullups) |
| 5 | SDA I2C4 | +3.3V (pullups) |
| 6 | SafetyButton | +3.3V |
| 7 | SafetyLED | +3.3V |
| 8 | - | - |
| 9 | Buzzer | +3.3V |
| 10 | GND | GND |

### GPS2/I2C3 port

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX (OUT) | +3.3V |
| 3 | RX (IN) | +3.3V |
| 4 | SCL I2C3 | +3.3V (pullups) |
| 5 | SDA I2C3 | +3.3V (pullups) |
| 6 | GND | GND |

### CAN1,CAN2 port

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | CAN_H | +24V |
| 3 | CAN_L | +24V |
| 4 | GND | GND |

### I2C1,I2C2 port

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | SCL | +3.3 (pullups) |
| 3 | SDA | +3.3 (pullups) |
| 4 | GND | GND |

### USB Ex

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC IN | +5V |
| 2 | D_minus | +3.3V |
| 3 | D_plus | +3.3V |
| 4 | GND | GND |

### RSSI

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | RSSI | up to +3.3V |
| 3 | UART8_TX (OUT) | +3.3 |
| 4 | GND | GND |

### RS232

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX (OUT) | +3.3V |
| 3 | RX (IN) | +3.3V |
| 4 | GND | GND |

### DEBUG

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | TX (OUT) | +3.3V |
| 3 | RX (IN) | +3.3V |
| 4 | SWDIO | +3.3V |
| 5 | SWCLK | +3.3V |
| 6 | GND | GND |

### ADC

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC | +5V |
| 2 | ADC_3V3 | up to +3.3V |
| 3 | ADC_6V6 | up to +6.6V |
| 4 | GND | GND |

### POWER1

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC IN | +5V |
| 2 | VCC IN | +5V |
| 3 | CURRENT | up to +3.3V |
| 4 | VOLTAGE | up to +3.3V |
| 5 | GND | GND |
| 6 | GND | GND |

### POWER2(CAN1)

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 | VCC IN | +5V |
| 2 | VCC IN | +5V |
| 3 | CAN1_H | +24V |
| 4 | CAN1_L | +24V |
| 5 | GND | GND |
| 6 | GND | GND |

## RC Input

RC input is configured on the RCIN pin, at one end of the servo rail. This pin supports all unidirectional RC protocols. For bi-directional protocols, such as CRSF/ELRS, UART8 can be set to protocol "23" and the receiver tied to RCIN (shared with UART8 RX) and UART8 TX. In this case, the RC_PROTOCOLS parameter should be set to the expected protocol type to avoid accidental erroneous detection by the RCIN path.

## PWM Output

The VUAV-V7pro supports up to 14 PWM outputs,support all PWM protocols. Outputs 1-12 support  DShot. Outputs 1-8 support bi-directional Dshot. All 14 PWM outputs have GND on the top row, 5V on the middle row and signal on the bottom row.

The 14 PWM outputs are in 4 groups:

- PWM 1, 2, 3 and 4 in group1
- PWM 5, 6, 7 and 8 in group2
- PWM 9, 10, 11 and 12 in group3
- PWM 13, 14 in group4

Channels within the same group need to use the same output rate. If any channel in a group uses DShot, then all channels in that group need to use DShot.

## GPIOs

All 14 PWM channels can be used for GPIO functions (relays, buttons, RPM etc).
The pin numbers for these PWM channels in ArduPilot are shown below:

   | PWM Channels | Pin | PWM Channels | Pin |
| --- | --- | --- | --- |
| PWM1 | 50 | PWM8 | 57 |
| PWM2 | 51 | PWM9 | 58 |
| PWM3 | 52 | PWM10 | 59 |
| PWM4 | 53 | PWM11 | 60 |
| PWM5 | 54 | PWM12 | 61 |
| PWM6 | 55 | PWM13 | 62 |
| PWM7 | 56 | PWM14 | 63 |

## Analog inputs

The VUAV-V7pro flight controller has 5 Analog inputs

- ADC Pin2-> Battery Current
- ADC Pin16 -> Battery Voltage
- ADC Pin19 -> ADC 3V3 Sense
- ADC Pin3 -> ADC 6V6 Sense
- ADC Pin9 -> RSSI voltage monitoring

## Battery Monitor Configuration

The board has voltage and current inputs sensor on the POWER1 ADC and POWER2 CAN connector.

The correct battery setting parameters are:

Enable Battery1 monitor:

- BATT_MONITOR   4
- BATT_VOLT_PIN 16
- BATT_CUR_PIN 2
- BATT_VOLT_MULT 15.7 (may need adjustment if supplied monitor is not used)
- BATT_AMP_PERVLT 60.61 (may need adjustment if supplied monitor is not used)

Enable Battery2 monitor (if used):

- BATT2_MONITOR  8

## Loading Firmware

The VUAV-V7pro flight controller comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of \*.apj firmware files with any ArduPilot compatible ground station.
