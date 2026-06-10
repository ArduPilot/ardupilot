# S-Vehicle E2-mini Flight Controller

The S-Vehicle E2-mini Flight Controller produced by [S-Vehicle](http://svehicle.cn).

![E2mini.png](E2mini.png)

## Features

- STM32H743IIK6 microcontroller
- 2 IMUs: Bosch BMI088, InvenSense ICM-42688P
- bmm150 magnetometer
- bmp388 barometers
- microSD card slot
- 1 ETH network interface
- 6 UARTs plus USB
- USB-TypeC port
- 14 PWM outputs
- PPM/SBus input, DSM/SBus input
- 3 I2C ports
- 2 CAN ports
- 100Mbps Ethernet port
- 1 power monitor inputs (XT30 port)

## Pinout

![E2mini-Top.png](E2mini-Top.png)
![E2mini-Top-pinouts.png](E2mini-TOP-pinouts.png)
![E2mini-pinouts-side.png](E2mini-pinouts-side.png)

## UART Mapping

- SERIAL0 -> USB (MAVLink2)
- SERIAL1 -> UART7 (TELEM1, MAVLink2)
- SERIAL2 -> UART5 (TELEM2, MAVLink2)
- SERIAL3 -> USART1 (GPS1/Compass/Safety Switch)
- SERIAL4 -> UART8 (GPS2/Compass)
- SERIAL5 -> USART2 (TELEM3)
- SERIAL6 -> UART4 (User)
- SERIAL7 -> USART3 (Debug)
- SERIAL8 -> USART6 (Generally used for SBUS)

All UARTs have full DMA capability.

## RC Input

RC input is configured on the RCIN pin, at one end of the servo rail, marked PPM in the above diagram. All ArduPilot supported unidirectional RC protocols can be input here including PPM. For bi-directional or half-duplex protocols, such as CRSF/ELRS a full UART will have to be used. See [RC systems](https://ardupilot.org/copter/docs/common-rc-systems.html) for details on USRT setup for other protocols.

## PWM Output

The E2mini flight controller supports up to 12 PWM outputs.

The 14 PWM outputs are in 4 groups:

- PWM 1-4 in group1 (TIM5)
- PWM 5-8 in group2 (TIM4)
- PWM 9, 10 in group3 (TIM12)
- PWM 11,12 in group4 (TIM3)

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

## GPIOs

The complete list of GPIOS is:

| Label | GPIO number |
|---|---|
| PWM(1) | 50 |
| PWM(2) | 51 |
| PWM(3) | 52 |
| PWM(4) | 53 |
| PWM(5) | 54 |
| PWM(6) | 55 |
| PWM(7) | 56 |
| PWM(8) | 57 |
| PWM(9) | 58 |
| PWM(10) | 59 |
| PWM(11) | 60 |
| PWM(12) | 61 |
| LED_RED | 90 |
| LED_GREEN | 91 |
| LED_BLUE | 92 |
| SP3_DRDY2 | 93 |

## Battery Monitoring

The board has a XT30 port,it can receive power input ranging from 2 to 14s, and it integrates a step-down module internally,in this way, an external voltage reduction module can be avoided, and the board can read the input voltage through the internal ADC.

## Compass

The E2-mini has an bmm150 builtin compass, but due to interference the board is usually used with an external I2C or CAN compass as part of a GPS/Compass combination.

## Analog Inputs

The E2-mini has 5 analog inputs.

- ADC Pin9 -> Battery Voltage
- ADC Pin8 -> Battery Current Sensor
- ADC Pin5 -> Vdd 5V supply sense
- ADC Pin13 -> ADC 3.3V Sense
- ADC Pin12 -> ADC 6.6V Sense

## Loading Firmware

Firmware for these boards can be found at the [ArduPilot firmware server](https://firmware.ardupilot.org) in sub-folders labeled "SVehicle-E2-mini".

The board comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of *.apj firmware files with any ArduPilot compatible ground station.
