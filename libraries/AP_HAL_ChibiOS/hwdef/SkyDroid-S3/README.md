# Skydroid S3 Flight Controller

The Skydroid S3  flight controller is sold by a range of
resellers, linked from [SKYDROID](https://www.skydroid.xin/)

## Features

- STM32H743VIT6 microcontroller
- BMI088 IMU
- internal vibration isolation for IMUs
- MS5611 SPI barometer
- microSD card slot
- 4 UARTs plus USB
- 12 PWM outputs
- 1 I2C 
- 2 CAN ports
- RCIN port
- voltage monitoring for  power bricks
- power input port for external power bricks

## Pinout

## UART Mapping

- SERIAL0 -> USB
- SERIAL1 -> USART2 (Telem1)
- SERIAL2 -> USART1 (GPS)
- SERIAL3 -> UART7 (debug port))
- SERIAL4 -> UART5 (Telem2)
- SERIAL5 -> UART4 (resv)

## Connectors

Unless noted otherwise all connectors are JST GH1.25mm

### TELEM1  &RC port

| Pin     | Signal    | Volt  |
| ------- | --------- | ----- |
| 1 (red) | VCC       | +5V   |
| 2 (red) | VCC       | +5V   |
| 3 (blk) | GND       | GND   |
| 4 (blk) | GND       | GND   |
| 5 (blk) | RX2 (IN)  | +3.3V |
| 6 (blk) | TX2 (OUT) | +3.3V |
| 7 (blk) | SBUS      | +3.3V |

### GPS1 port

| Pin     | Signal    | Volt  |
| ------- | --------- | ----- |
| 1 (blk) | GND       | GND   |
| 2 (blk) | SDA I2C2  | +3.3V |
| 3 (blk) | SCL I2C2  | +3.3V |
| 4 (blk) | RX3 (IN)  | +3.3V |
| 5 (blk) | TX3 (OUT) | +3.3V |
| 6 (red) | VCC       | +5V   |

### Telem2 port

| Pin     | Signal   | Volt  |
| ------- | -------- | ----- |
| 1 (blk) | GND      | GND   |
| 2 (blk) | TX5(OUT) | +3.3V |
| 3 (blk) | RX5 (IN) | +3.3V |

### uart7 port(&pwm)

| Pin          | Signal    | Volt  |
| ------------ | --------- | ----- |
| up(blk)      | GND       | GND   |
| middle (blk) | RX7(IN)   | +3.3V |
| down (blk)   | TX7 (OUT) | +3.3V |

### CAN1

| Pin     | Signal | Volt  |
| ------- | ------ | ----- |
| 1 (blk) | CAN_L  | +3.3V |
| 2 (blk) | CAN_H  | +3.3V |
| 3 (red) | VCC    | +5V   |
| 4 (blk) | GND    | GND   |

### CAN1-REV

| Pin     | Signal | Volt  |
| ------- | ------ | ----- |
| 1 (blk) | CAN_L  | +3.3V |
| 2 (blk) | CAN_H  | +3.3V |
| 3 (red) | VCC    | +5V   |
| 4 (blk) | GND    | GND   |

### POWER1

| Pin     | Signal  | Volt      |
| ------- | ------- | --------- |
| 1 (blk) | GND     | GND       |
| 2 (blk) | GND     | GND       |
| 3 (red) | BAT+    | 5V to 60V |
| 4 (red) | BAT+    | 5V to 60V |
| 5 (blk) | NC      | ---   |

### USB

type-c 

## RC Input

RC input is configured on the RCIN pin, at one end of the servo rail,
marked RCIN in the above diagram. This pin supports all RC protocols.

## PWM Output

The Skydroid supports up to 12 PWM outputs . First first 8 outputs
(labelled "MAIN") are controlled by a dedicated STM32H743 controller. 
These 8 outputs support all PWM output formats,

The remaining 4 outputs (labelled AUX1 to AUX4) are the "auxiliary"
outputs. These are directly attached to the STM32H743 and support all
PWM protocols as well as DShot.

All 12 PWM outputs have GND on the top row, 5V on the middle row and
signal on the bottom row.

The 8 main PWM outputs are in 2 groups:

- PWM 1, 2, 3 and 4 in group1
- PWM 5, 6, 7 and 8 in group2

The 4 auxiliary PWM outputs are in 1 groups:

- PWM 1, 2, 3 and 4 in group3


## Battery Monitoring

The board has a dedicated power monitor ports on a 5 pin connector. The correct battery setting parameters are dependent on the type of power brick which is connected.

## Compass

There is no compass inside, an external compass is required

## GPIOs
The 4 AUX PWM ports can be used as GPIOs (relays, buttons, RPM etc).

The numbering of the GPIOs for PIN variables in ArduPilot is:

- PWM1 50
- PWM2 51
- PWM3 52
- PWM4 53
## Analog inputs

The Skydroid has 2 analog inputs

- ADC Pin10 -> Battery Voltage
- ADC Pin13 -> Battery Current Sensor

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of *.apj firmware files with any ArduPilot
compatible ground station.
