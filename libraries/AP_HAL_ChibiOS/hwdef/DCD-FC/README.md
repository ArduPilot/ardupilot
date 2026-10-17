# DCD FC

## Introduction

DCD FC is a flight controller with integrated power distribution from [Corp.DCLASS](https://www.d-class.co.kr). It provides battery input and power pads for four external ESCs.

![DCD FC with enclosure](images/product_photo.webp)

## Features and specifications

- STM32H743IIT6 main MCU and STM32F103C8T6 IO MCU.
- BMI088 and ICM-42688-P IMUs; MS5611 barometer; IST8310 compass.
- FRAM parameter storage and microSD logging.
- 16 PWM outputs, two GPS ports, three telemetry ports, I2C, SBUS and USB.
- 3–6S LiPo input, maximum 25.2 V.
- Enclosure approximately 126 × 84 × 13 mm; weight 103 g with enclosure.
- Front points toward the BAT/ESC pads. Secure the FC to the aircraft with double-sided tape. The 72 × 114 mm hole pattern is for enclosure assembly.
- No external CAN port or PPM input.

## Where to buy

Purchase enquiries and technical support: [Corp.DCLASS](https://www.d-class.co.kr).

## Pinout

Top view, with Front at the top. Circled numbers are pin numbers. The cable mating-face view is mirrored. TX/RX are named from the FC perspective.

![Dimensions and Front](images/dimensions_front.png)

![Battery, ESC and PWM connections](images/power_pwm.png)

Battery positive and ESC positive pads distribute the same battery voltage; negative pads are GND. F1–F8 are FMU outputs (SERVO9–16), and I1–I8 are IO outputs (SERVO1–8). PWM PWR supplies shared 5 V; do not parallel another BEC.

### GPS1

![GPS1 pin numbers](images/GPS1.png)

| Pin | Signal |
| --- | --- |
| 1 | GND |
| 2 | BUZZER |
| 3 | +3.3 V |
| 4 | SAFETY LED |
| 5 | SAFETY SW |
| 6 | SDA |
| 7 | SCL |
| 8 | RX |
| 9 | TX |
| 10 | +5 V |

### GPS2

![GPS2 pin numbers](images/GPS2.png)

| Pin | Signal |
| --- | --- |
| 1 | +5 V |
| 2 | TX |
| 3 | RX |
| 4 | SCL |
| 5 | SDA |
| 6 | GND |

### TLM1

![TLM1 pin numbers](images/TLM1.png)

| Pin | Signal |
| --- | --- |
| 1 | +5 V |
| 2 | TX |
| 3 | RX |
| 4 | CTS |
| 5 | RTS |
| 6 | GND |

### TLM2

![TLM2 pin numbers](images/TLM2.png)

| Pin | Signal |
| --- | --- |
| 1 | +5 V |
| 2 | TX |
| 3 | RX |
| 4 | CTS |
| 5 | RTS |
| 6 | GND |

### TLM3

![TLM3 pin numbers](images/TLM3.png)

| Pin | Signal |
| --- | --- |
| 1 | +5 V |
| 2 | TX |
| 3 | RX |
| 4 | SCL |
| 5 | SDA |
| 6 | GND |

### I2C

![I2C pin numbers](images/I2C.png)

| Pin | Signal |
| --- | --- |
| 1 | GND |
| 2 | SDA |
| 3 | SCL |
| 4 | +5 V |

### S.B

![S.B pin numbers](images/S_B.png)

| Pin | Signal |
| --- | --- |
| 1 | GND |
| 2 | NC |
| 3 | RSSI |
| 4 | SBUS IN |
| 5 | +5 V |

### S.OUT

![S.OUT pin numbers](images/S_OUT.png)

| Pin | Signal |
| --- | --- |
| 1 | NC |
| 2 | SIGNAL OUT |
| 3 | GND |

### FUUP

![FUUP pin numbers](images/FUUP.png)

| Pin | Signal |
| --- | --- |
| 1 | 3.3 V REF |
| 2 | SWCLK |
| 3 | SWDIO |
| 4 | GND |

### IOUP

![IOUP pin numbers](images/IOUP.png)

| Pin | Signal |
| --- | --- |
| 1 | 3.3 V REF |
| 2 | SWCLK |
| 3 | SWDIO |
| 4 | GND |

S.OUT carries IO USART3 TX and has no power output. FUUP and IOUP are SWD service ports; their 3.3 V pins are debugger references. Leave NC pins unconnected.

### Port power

| Supply | Voltage | Shared current condition |
| --- | --- | --- |
| GPS1/2, TLM1/2/3, I2C, S.B | 5 V | Up to 1.5 A total including internal FC consumption |
| F1–F8 and I1–I8 PWR | 5 V | Up to 1.5 A total across both banks, separate from FC/peripherals |
| GPS1 pin 3 | 3.3 V | 1 A supply rating shared with microSD |

Allow for internal consumption and thermal limits. USB is also limited by the host and cable.

## UART mapping

| Serial port | Peripheral | Default protocol | Connection |
| --- | --- | --- | --- |
| SERIAL0 | OTG1 | USB MAVLink | USB |
| SERIAL1 | UART7 | MAVLink2 | CN3 Telem1, CTS/RTS |
| SERIAL2 | UART5 | MAVLink2 | CN4 Telem2, CTS/RTS |
| SERIAL3 | USART1 | GPS | CN1 GPS1 |
| SERIAL4 | UART8 | GPS | CN2 GPS2 |
| SERIAL5 | USART2 | None | CN5 Telem3 |
| SERIAL6 | USART3 | None | FMU debug signals; no external connector claimed |
| SERIAL7 | OTG2 | Build-defined USB secondary interface | No second physical USB socket claimed |

USART6 connects the FMU to the IO processor and is not an external serial port.

## PWM outputs

| ArduPilot outputs | Board outputs | Timer group |
| --- | --- | --- |
| 1-2 | IO 1-2 | IO TIM2 |
| 3-4 | IO 3-4 | IO TIM4 |
| 5-8 | IO 5-8 | IO TIM3 |
| 9-12 | FMU 1-4 | FMU TIM1 |
| 13-14 | FMU 5-6 | FMU TIM4 |
| 15-16 | FMU 7-8 | FMU TIM5 |

Outputs in a timer group must use compatible output protocols. Standard and DShot IO firmware are included. Use BRD_IO_ENABLE=1 when IO firmware updates are required; value 2 skips those updates.

## RC input

Connect SBUS to S.B pin 4 with 5 V and GND. PPM is not supported. For CRSF/ELRS UART receivers, use a free full TX/RX UART, set SERIALx_PROTOCOL=23, and follow the [ArduPilot radio control guide](https://ardupilot.org/copter/docs/common-rc-systems.html).

## GPIOs

FMU output GPIO numbers 50–57 correspond to F1–F8 when configured as GPIO. They are signal pins, not power outputs.

## RSSI and analog inputs

S.B pin 3 carries the IO RSSI input. Onboard voltage/current sensing uses analog pins 8/4. These ADC connections are not battery input pads. There is no dedicated external analog airspeed input; external I2C peripherals can use the I2C port.

## Battery monitoring

| Parameter | Setting |
| --- | ---: |
| BATT_MONITOR | 4 |
| BATT_VOLT_PIN | 8 |
| BATT_CURR_PIN | 4 |
| BATT_VOLT_MULT | 8.987575 |
| BATT_AMP_PERVLT | 0.25 |

Use BATT_AMP_OFFSET=0 as the initial offset and calibrate voltage/current against a reference meter. The onboard sensor measures the FC power branch, excluding ESC propulsion current. Use a separate sensor for total battery current and consumption. Set capacity to the installed pack; existing saved parameters take precedence over defaults.

## Compass

An IST8310 compass is fitted. External compasses can use I2C on the GPS or I2C connectors. Select and calibrate compasses for the installed aircraft.

## Firmware

The firmware target is `DCD-FC`. Development firmware and corresponding source are available from the [DCD FC project](https://github.com/LeeWonseok-kr/DCD-FC/releases).

## Loading firmware

Use Mission Planner's Load custom firmware function with a DCD-FC APJ matching the installed bootloader. The FMU application and bootloader use reserved board ID **1413**. Do not bypass board-ID checks.

The FMU bootloader starts at 0x08000000; the application starts at 0x08020000. IO application images are embedded in the FMU firmware and are not IO bootloaders. Retain the compatible stock IO bootloader.

## Other links

- [Manufacturer and support](https://www.d-class.co.kr)
- [ArduPilot porting guide](https://ardupilot.org/dev/docs/porting.html)
