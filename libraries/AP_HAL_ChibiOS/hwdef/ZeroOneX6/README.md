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
| Name    | Function |DMA|
| ------- | -------- |---|
| SERIAL0 | OTG1     ||
| SERIAL1 | UART7    |DMA Enabled |
| SERIAL2 | UART5    |DMA Enabled |
| SERIAL3 | USART1   |DMA Enabled |
| SERIAL4 | UART8    |DMA Enabled |
| SERIAL5 | USART2   |DMA Enabled |
| SERIAL6 | UART4    |DMA Enabled |
| SERIAL7 | USART3   |DMA Enabled |
| SERIAL8 | OTG2     ||

## RC Input
The remote control signal should be connected to the SBUS RC IN port or DSM/PPM RC Port.It will support ALL unidirectional RC protocols.

## PWM Output
The X6 flight controller supports up to 16 PWM outputs. 
First first 8 outputs (labelled 1 to 8) are controlled by a dedicated STM32F103 IO controller. These 8 outputs support all PWM output formats, but not DShot.
The remaining 8 outputs (labelled 9 to 16) are the "auxiliary" outputs. These are directly attached to the STM32H753 FMU controller .
All 16 outputs support normal PWM output formats. All FMU outputs, except 15 and 16, also support DShot.

The 8 IO PWM outputs are in 4 groups:
- Outputs 1 and 2 in group1
- Outputs 3 and 4 in group2
- Outputs 5, 6, 7 and 8 in group3
  
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
