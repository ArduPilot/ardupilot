# CubeNode-vehicle

This build can be used to flash a vehicle code on a CubeNode. This will have to be loaded using dfu as the CubeNode ships with a Periph bootloader which cannot load a vehicle binary.

This can be used as a starting point for other configurations, pin functions can be add, removed, or changed as needed.

## Serial

Four serial physical ports are available along with two USB endpoints. The ordering is:

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

| Pin name | Pin number | function  |
| -------- | ---------- | --------- |
| PE7      | 23         | UART7_RX  |
| PE8      | 22         | UART7_TX  |
| PE10     | 21         | UART7_CTS |
| PE9      | 72         | UART7_RTS |

#### SERIAL4

Note this is not available on V3.

| Pin name | Pin number | function  |
| -------- | ---------- | --------- |
| PB9      | 59         | UART4_TX  |
| PB8      | 5          | UART4_RX  |

#### SERIAL5

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

A two I2C buses are provided, software pull-up resistors can be enabled but hardware pull-up are recommended.

##### I2C 1

| Pin name | Pin number | function |
| -------- | ---------- | -------- |
| PA8      | 10         | I2C3_SCL |
| PC9      | 30         | I2C3_SDA |

Note that pin 10 is connected to both PA8 and PA10. PA10 must not be used.

##### I2C 2

| Pin name | Pin number | function |
| -------- | ---------- | -------- |
| PD12     | 35         | I2C4_SCL |
| PD13     | 34         | I2C4_SDA |

# LEDs

Three board LEDs are defined for use with notify to provide status feedback, these are active low.

| Pin name | Pin number | function  |
| -------- | ---------- | --------- |
| PB1      | 45         | Blue LED  |
| PA4      | 51         | Green LED |
| PE6      | 47         | Red LED   |

# Buzzer

A Alarm output is provided for a buzzer, some external drive circuit is required.

| Pin name | Pin number | function |
| -------- | ---------- | -------- |
| PE5      | 57         | ALARM    |

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

# PWM outputs

PWM output pins can also be used as GPIOs. They are in three groups. All outputs are capable of bi-directional dshot.

| Pin name | Pin number | function   | SERVO num | GPIO num | Group |
| -------- | ---------- | ---------- | --------- | -------- | ------|
| PE11     | 71         | TIM1_CH2   | 1         | 50       | A     |
| PE13     | 70         | TIM1_CH3   | 2         | 51       | A     |
| PE14     | 69         | TIM1_CH4   | 3         | 52       | A     |
| PA5      | 50         | TIM2_CH1   | 4         | 53       | B     |
| PB10     | 58         | TIM2_CH3   | 5         | 54       | B     |
| PB7      | 6          | TIM4_CH2   | 6         | 55       | C     |
| PD14     | 33         | TIM4_CH3   | 7         | 56       | C     |
| PD15     | 32         | TIM4_CH4   | 8         | 57       | C     |

# RC Input

Multi-protocol RC input pin.

| Pin name | Pin number | function |
| -------- | ---------- | -------- |
| PC8      | 31         | TIM3_CH3 |

# Analog input

ADC inputs, general purpose inputs can be used by battery monitor for example. There is no internal resistor divider inputs must be 0 to 3.3v.

| Pin name | Pin number | function   | Analog pin number |
| -------- | ---------- | ---------- | ----------------- |
| PA0      | 68         | ADC1_INP16 | 16                |
| PF11     | 65         | ADC1_INP2  | 2                 |
| PF12     | 62         | ADC1_INP6  | 6                 |
| PC0      | 53         | ADC1_INP10 | 10                |

Hard coded inputs for servo and board voltage are provided. The configuration assumes a resistor divider ratio.

| Pin name | Pin number | function   | source              |
| -------- | ---------- | ---------- | ------------------- |
| PF5      | 26         | ADC3_INP4  | board voltage * 1/2 |
| PF3      | 25         | ADC3_INP5  | servo rail * 1/3    |

# Status inputs

Some general digital inputs are used to populate the power flags.

| Pin name | Pin number | function | flag |
| -------- | ---------- | -------- | ---- |
| PA9      | 77         | VBUS     | High if USB voltage is detected |
| PA6      | 64         | I/O      | High if primary supply voltage is good |
| PF4      | 37         | I/O      | High if secondary supply voltage is good |

# SPI

A single SPI bus is available, this is used internally for the IMU. Two chip-select pins are also provided for use with external sensors. A hwdef change will be required to enable any external sensor.

| Pin name | Pin number | function  | SPI function           |
| -------- | ---------- | --------- | ---------------------- |
| PC10     | 7          | SPI3_SCK  | SCK                    |
| PC11     | 8          | SPI3_MISO | MISO                   |
| PC12     | 9          | SPI3_MOSI | MOSI                   |
| PF14     | 38         | I/O       | external chip select 1 |
| PF13     | 39         | I/O       | external chip select 2 |
