# CubeNode-vehicle

This build can be used to flash a vehicle code on a CubeNode. This will have to be loaded using dfu as the CubeNode ships with a Periph bootloader which cannot load a vehicle binary.

This can be used as a starting point for other configurations, pin functions can be add, removed, or changed as needed.

## Serial

Three serial physical ports are available along with two USB endpoints. The ordering is:

#### SERIAL0

First USB endpoint

#### SERIAL1

| Pin name | Pin number | function |
| -------- | ---------- | -------- |
| PE1      | 3          | UART8_TX |
| PE0      | 4          | UART8_RX |

#### SERIAL2

| Pin name | Pin number | function  |
| -------- | ---------- | --------- |
| PC6      | 44         | USART6_TX |
| PC7      | 43         | USART6_RX |

#### SERIAL3

UART 3 has flow control.

| Pin name | Pin number | function  |
| -------- | ---------- | --------- |
| PE7      | 23         | UART7_RX  |
| PE8      | 22         | UART7_TX  |
| PE10     | 21         | UART7_CTS |
| PE9      | 72         | UART7_RTS |

##### SERIAL4

Second USB endpoint

# CAN

Two CAN busses are available, transceivers are within the Node itself but termination resistors are not.

#### CAN 1

| Pin name | Pin number | function  |
| -------- | ---------- | --------- |
| CAN1_H   | 2          | CAN1 high |
| CAN1_L   | 1          | CAN1 low  |

#### CAN 2

| Pin name | Pin number | function  |
| -------- | ---------- | --------- |
| CAN2_H   | 15         | CAN2 high |
| CAN2_L   | 14         | CAN2 low  |

# I2C

A single I2C bus is provided, software pullup resistors are enabled.

##### I2C

| Pin name | Pin number | function |
| -------- | ---------- | -------- |
| PB8      | 5          | I2C1_SCL |
| PB7      | 6          | I2C1_SDA |

# LEDs

Three board LEDs are defined for use with notify to provide status feedback, these are active low.

| Pin name | Pin number | function  |
| -------- | ---------- | --------- |
| PB1      | 45         | Blue LED  |
| PB0      | 46         | Green LED |
| PE6      | 47         | Red LED   |

# Buzzer

A Alarm output is provided for a buzzer, some external drive circuit is required.

| Pin name | Pin number | function |
| -------- | ---------- | -------- |
| PA15     | 60         | ALARM    |

# SD Card

A SD can be connected via SDMMC

| Pin name | Pin number | function   |
| -------- | ---------- | ---------- |
| PB14     | 102        | SDMMC2_D0  |
| PB15     | 101        | SDMMC2_D1  |
| PB3      | 106        | SDMMC2_D2  |
| PB4      | 107        | SDMMC2_D3  |
| PD6      | 110        | SDMMC2_CK  |
| PD7      | 111        | SDMMC2_CMD |

## PWM outputs

PWM output pins can also be used as GPIOs. They are in two groups.

| Pin name | Pin number | function   | SERVO num | GPIO num | Group |
| -------- | ---------- | ---------- | --------- | -------- | ------|
| PE14     | 69         | TIM1_CH4   | 1         | 50       | A     |
| PE13     | 70         | TIM1_CH3   | 2         | 51       | A     |
| PE11     | 71         | TIM1_CH2   | 3         | 52       | A     |
| PD13     | 34         | TIM4_CH2   | 4         | 53       | B     |
| PD14     | 33         | TIM4_CH3   | 5         | 54       | B     |
| PD15     | 32         | TIM4_CH4   | 6         | 55       | B     |
