# AET-405-Mini Flight Controller

## Specifications
- MCU: STM32F405
- IMU: ICM42688 / BMI270 (auto-detected)
- OSD: AT7456E
- Baro: SPL06
- Remote ID: DroneCAN (CAN1)
- UARTs: 1 (console), 2 (RCIN), 3 (camera), 4 (GPS), 6 (Bluetooth/MAVLink2)
- PWM outputs: 9 channels
- LED: 2x status LEDs

## UART Mapping
| UART | Function |
|------|----------|
| USART1 | Console |
| USART2 | RCIN |
| USART3 | Camera |
| UART4 | GPS |
| UART6 | Bluetooth (MAVLink2) |

## PWM Output
| PWM | Channel |
|-----|---------|
| 1-9 | Motor outputs |

