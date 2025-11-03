# Morakot Flight Controller Overview

The Morakot is a flight controller designed and produced by [Taiphoon](https://taiphoon.com.tw/)

![top](Morakot_top.png)
![bottom](Morakot_bottom.png)

Morakot Flight Controller — **NDAA-Compliant. Made in Taiwan.** Built for Performance.
Engineered, tested, and manufactured in Taiwan, the Morakot Flight Controller meets full NDAA compliance, ensuring trusted quality and security for professional applications. With an integrated Ethernet interface, it delivers high-speed, reliable connectivity for next-generation FPV and unmanned aerial systems.

## Pinout
![top_wired](Morakot_top_Wired.png)
![bottom_wired](Morakot_bottom_Wired.png)

## Features
#### Sensors
- [ICM-45686 High-Performance IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-45686/)
- [Bosch BMP390 Barometer](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp390/)
- [ST IIS2MDC Magnetometer](https://www.st.com/en/mems-and-sensors/iis2mdc.html)

#### Microprocessor
- [STM32H743VIT MCU](https://www.st.com/en/microcontrollers-microprocessors/stm32h743vi.html)
  - 480Mhz / 1MB RAM / 2MB Flash

#### Power
- 3S–8S DC Input power
- 5V 2A BEC peripherals power 
- 9V 2A BEC servos power 
- 12V 2A BEC video power

#### Other
- LED Indicators
- Battery voltage indicator LED
- MicroSD Slot
- USA Built
- 8x motor outputs
- 8x UART
- 1x I2C
- 1x CAN
- 1x Ethernet

### UART Mapping
UART Mapping
The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
- SERIAL0 -> USB (MAVLink2)
- SERIAL1 -> USART1 (DisplayPort, DMA-enabled)
- SERIAL2 -> USART2 (MAVLink1-Telem1, DMA-enabled)
- SERIAL3 -> USART3 (MAVLink2-Telem2(default)/S.BUS, DMA-enabled)
- SERIAL4 -> UART5  (GPS, DMA-enabled)
- SERIAL5->  USART6 (ESCTelemetry2, DMA-enabled)
- SERIAL6 -> UART7  (MAVLink-Telem3 (default)/ ESCTelemetry1, DMA-enabled)
- SERIAL7 -> UART8  (RC Input,  DMA-enabled)

### VTX Support
The JST-GH 7p connector supports a DJI Air Unit / HD VTX connection. Protocol defaults to DisplayPort. Pin 1 of the connector is 12v so be careful not to connect this to a peripheral that can not tolerate this voltage.

## Additional Information
- Dimensions: 38.5 x 30.5 mm
- Height: up to 8.5 mm
- Mounting: 30.5 x 30.5 mm (M3/M4 with dampers)
- Weight: 7 g

## More Information
- [Morakot Flight Controller](https://taiphoon.com.tw/morakot-flight-controllor)
- [Morakot documentation](https://taiphoon-com.gitbook.io/taiphoon.com-docs/flight-stack/morakot-flight-controller)

## Connetions
#### PWM1 UART7 - 8 Pin JST-GH
| Pin | Signal Name     | Voltage      |
|-----|-----------------|--------------|
| 1   | VBAT IN         | 12V-33.6V    |
| 2   | UART7_RX_EXT    | 3.3V         |
| 3   | GND             | 3.3V         |
| 4   | CURR_IN_EXT     | 3.3V         |
| 5   | FMU_TIM1_CH4_EXT| 3.3V         |
| 6   | FMU_TIM1_CH3_EXT| 3.3V         |
| 7   | FMU_TIM1_CH2_EXT| 3.3V         |
| 8   | FMU_TIM1_CH1_EXT| 3.3V         |

#### PWM2 UART6 - 8 Pin JST-GH
| Pin | Signal Name     | Voltage      |
|-----|-----------------|--------------|
| 1   | VBAT IN         | 12V-33.6V    |
| 2   | UART6_RX_EXT    | 3.3V         |
| 3   | GND             | 3.3V         |
| 4   | CURR_IN_EXT     | 3.3V         |
| 5   | FMU_TIM2_CH1_EXT| 3.3V         |
| 6   | FMU_TIM2_CH2_EXT| 3.3V         |
| 7   | FMU_TIM2_CH3_EXT| 3.3V         |
| 8   | FMU_TIM2_CH4_EXT| 3.3V         |


#### CAN - 4 Pin JST-GH
| Pin | Signal Name | Voltage |
|-----|-------------|---------|
| 1   | 5.0V        | 5.0V    |
| 2   | CAN1_H      | 5.0V    |
| 3   | CAN1_L      | 5.0V    |
| 4   | GND         | GND     |

#### GPS - 6 Pin JST-GH
| Pin | Signal Name         | Voltage |
|-----|---------------------|---------|
| 1   | 5.0V                | 5.0V    |
| 2   | UART5_TX_GPS1_EXT   | 3.3V    |
| 3   | UART5_RX_GPS1_EXT   | 3.3V    |
| 4   | I2C1_SCL_GPS1_EXT   | 3.3V    |
| 5   | I2C1_SDA_GPS1_EXT   | 3.3V    |
| 6   | GND                 | GND     |

#### UART(TELEM) - 6 Pin JST-GH
| Pin | Signal Name           | Voltage |
|-----|-----------------------|---------|
| 1   | 5.0V                  | 5.0V    |
| 2   | UART7_TX_TELEM1_EXT   | 3.3V    |
| 3   | UART7_RX_TELEM1_EXT   | 3.3V    |
| 4   | UART7_CTS_TELEM1_EXT  | 3.3V    |
| 5   | UART7_RTS_TELEM1_EXT  | 3.3V    |
| 6   | GND                   | GND     |

#### VTX - 7 Pin JST-GH
Note: connector pinout not in same order as standard HD VTX cabling
| Pin | Signal Name           | Voltage |
|-----|-----------------------|---------|
| 1   | VIDEO                 |         |
| 2   | 12.0V                 | 12.0V   |
| 3   | GND                   | GND     |
| 4   | USART1_RX_TELEM1_EXT  | 3.3V    |
| 5   | USART1_TX_TELEM1_EXT  | 3.3V    |
| 6   | GND                   | 3.3V    |
| 7   | USART3_RX_TELEM3_EXT  | GND     |

#### SPI (OSD or IMU) - 6 Pin JST-SH
| Pin | Signal Name         | Voltage |
|-----|---------------------|---------|
| 1   | 5.0V                | 5.0V    |
| 2   | SPI4_MOSI_EXT       | 3.3V    |
| 3   | SPI4_MISO_EXT       | 3.3V    |
| 4   | SPI4_SCK_EXT        | 3.3V    |
| 5   | SPI4_CS_EXT         | 3.3V    |
| 6   | GND                 | GND     |


#### RC - 4 Pin JST-GH
| Pin | Signal Name         | Voltage |
|-----|---------------------|---------|
| 1   | 5.0V                | 5.0V    |
| 2   | UART8_RX_TELEM8_EXT | 3.3V    |
| 3   | UART8_TX_TELEM8_EXT | 3.3V    |
| 4   | GND                 | GND     |

#### ETH - 4 Pin JST-GH
| Pin | Signal Name         | Voltage |
|-----|---------------------|---------|
| 1   | RXN                 | 3.3V    |
| 2   | RXP                 | 3.3V    |
| 3   | TXN                 | 3.3V    |
| 4   | TXP                 | 3.3V    |

## RC Input

RC input is via SERIAL7 on the RC connector. Unidirectional protocols can be connected to R8. Bi-Directional Protocols will use the T8 pin also.

- PPM is supported.  
- SBUS/DSM/SRXL connects to the RX3 pin.  
- FPort requires connection to TX3. Set :ref:`SERIAL7_OPTIONS<SERIAL3_OPTIONS>` = 7  
- CRSF/ELRS also requires both TX3 and RX3 connections and provides telemetry automatically.
In order to use the SBUS pin on the HD VTX connector, you must change SERIAL7_PROTOCOL to something other than "23" and set SERILA3_PROTOCOL to "23".

- Use VTX RX interface as RC input
```
SERIAL7_OPTIONS,0
SERIAL7_PROTOCOL,23

# To disable RC input on Serial7:

SERIAL7_BAUD,115
SERIAL7_OPTIONS,0
SERIAL7_PROTOCOL,-1

# To enable VTX RC input on Serial3 (CRSF / ELRS):

SERIAL3_BAUD,115
SERIAL3_OPTIONS,0
SERIAL3_PROTOCOL,-1

# To enable VTX RC input on Serial3 (S.BUS):

SERIAL3_BAUD,115
SERIAL3_OPTIONS,1
SERIAL3_PROTOCOL,23
```

- Use RC input interface as RC input
```
# disable VTX RC input on Serial3
SERIAL3_BAUD,115
SERIAL3_OPTIONS,0
SERIAL3_PROTOCOL,-1

# Enable RC input on Serial7(CSRF、ELRS)
SERIAL7_BAUD,115
SERIAL7_OPTIONS,0
SERIAL7_PROTOCOL,23
```

## Compass
The Morakot has a built-in compass. Due to potential interference, the autopilot is usually used with an external I2C compass as part of a GPS/Compass combination and the internal compass disabled.

## Firmware
Firmware for this board can be found here in sub-folders labeled “Morakot”

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the “Morakotxx.hex” firmware, using your favorite DFU loading tool.
Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the “*.apj” firmware files.

