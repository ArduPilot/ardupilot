## ZeroOneX6 Flight Controller
The ZeroOne X6 is a flight controller manufactured by ZeroOne, which is based on the open-source FMU v6X architecture and Pixhawk Autopilot Bus open source specifications.

## Features:
- Separate flight control core design.
- MCU 
   STM32H753IIK6 32-bit processor running at 480MHz
   2MB Flash
   1MB RAM
- Sensors
- IMU: 
   Internal Vibration Isolation for IMUs
   IMU constant temperature heating(1 W heating power).
   With Triple Synced IMUs, BalancedGyro technology, low noise and more shock-resistant:
   IMU1-ICM45686(With vibration isolation) 
   IMU2-BMI088(With vibration isolation) 
   IMU3- ICM45686(No vibration isolation)
- Baro:
   Two barometers:2 x ICP20100
  Magnetometer:   Builtin RM3100 magnetometer

## Pinout
![ZeroOneX6 Pinout](https://github.com/ZeroOne-Aero/ardupilot/blob/zeroOneBootLoader/libraries/AP_HAL_ChibiOS/hwdef/ZeroOneX6/ZeroOneX6Pinout.jpg "ZeroOneX6")


## UART Mapping
The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
| Name    | Function |
| ------- | -------- |
| SERIAL0 | OTG1     |
| SERIAL1 | UART7    |
| SERIAL2 | UART5    |
| SERIAL3 | USART1   |
| SERIAL4 | UART8    |
| SERIAL5 | USART2   |
| SERIAL6 | UART4    |
| SERIAL7 | USART3   |
| SERIAL8 | OTG2     |

## RC Input
The remote control signal should be connected to the SBUS RC IN port or DSM/PPM RC Port.Support three types of remote control signal inputs, SBUS/DSM and PPM signals.

## PWM Output
The X6 flight controller supports up to 16 PWM outputs. All 16 outputs support normal PWM output formats. All FMU outputs, except 7 and 8, also support DShot.

The 8 FMU PWM outputs are in 4 groups:
- Outputs 1, 2, 3 and 4 in group1
- Outputs 5 and 6 in group2
- Outputs 7 and 8 in group3

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

## Battery Monitoring
The X6 flight controller has two six-pin power connectors, supporting CAN interface power supply.
These are set by default in the firmware and shouldn't need to be adjusted.

## Compass
The X6 flight controller built-in industrial-grade electronic compass chip RM3100.

## Analog inputs
The X6 flight controller has 2 analog inputs.
- ADC Pin12 -> ADC 6.6V Sense
- ADC Pin13 -> ADC 3.3V Sense
- RSSI input pin = 103
