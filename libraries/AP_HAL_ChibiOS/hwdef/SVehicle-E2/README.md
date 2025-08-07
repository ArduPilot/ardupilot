# S-Vehicle E2-Plus Flight Controller

The S-Vehicle E2-Plus Flight Controller produced by [S-Vehicle](http://svehicle.cn).

![E2Plus.png](E2Plus.png)

## Features

- STM32H753IIK6 microcontroller
- STM32F103 microcontroller
- 3 IMUs: Bosch BMI088, InvenSense ICM-20649, InvenSense ICM-42688P
- RM3100 magnetometer
- dual ICP-20100 barometers
- microSD card slot
- 1 ETH network interface
- 6 UARTs plus USB
- USB-TypeC port
- 14 PWM outputs
- PPM/SBus input, DSM/SBus input
- 3 I2C ports
- 2 CAN ports 
- Analog RSSI input
- 100Mbps Ethernet port
- 3 power monitor inputs

# IOMCU

This board has an IOMCU implemented on a STM32F103 microcontroller which, manages 

## Pinout

![E2plus_pinouts.jpg](E2plus-Top.png)
![E2plus_pinouts-left.png](E2plus-pinouts-left.png)
![E2plus_pinouts-right.png](E2plus-pinouts-right.png)
![E2plus_pinouts-front.png](E2plus-pinouts-front.png)

## UART Mapping

- SERIAL0 -> USB (MAVLink2)
- SERIAL1 -> UART7 (TELEM1, MAVLink2)
- SERIAL2 -> UART5 (TELEM2, MAVLink2)
- SERIAL3 -> USART1 (GPS1/Compass/Safety Switch)
- SERIAL4 -> UART8 (GPS2/Compass)
- SERIAL5 -> USART2 (TELEM3, RS422)
- SERIAL6 -> UART4 (User)
- SERIAL7 -> USART3 (Debug)
- SERIAL8 -> USB Virtual

All UARTs have full DMA capability. TELEM3 can be used as an RS422 interface, the driver is included.

## RC Input

RC input is configured on the RCIN pin, at one end of the servo rail, marked PPM in the above diagram. All ArduPilot supported unidirectional RC protocols can be input here including PPM. For bi-directional or half-duplex protocols, such as CRSF/ELRS a full UART will have to be used. See :ref:`common-rc-systems` for details on USRT setup for other protocols.

## PWM Output

The E2-Plus flight controller supports up to 14 PWM outputs.

The 14 PWM outputs are:

M1 - M8 are connected to the IOMCU
A1 - A6 are connected to the FMU

M1 - M8 support DShot but not BDShot and are all in one group (IS THIS CORRECT?)
A1 - A4 are in one group (TIM5). 
A5, A6 are in a 2nd group (TIM4)

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot. Outputs A1-A6 support BDShot.

-- TO BE DELETED
- PWM 1-4 in group1 (TIM5)
- PWM 5 and 6 in group2 (TIM4)
- PWM 7 and 8 in group3 (TIM1 - no DMA, PWM only)
- PWM 9, 10 and 11 in group4 (TIM8)
- PWM 12 in group5 (TIM15)
- PWM 13 and 14 in group6 (TIM12)
-- TO BE DELETED


## Battery Monitoring

The board has connectors for 3 power monitors. The board is configure by default for a DroneCAN power monitor, and also has analog power monitor defaults configured which is enabled. The default PDB included with the E2+ is DroneCAN and must be connected to Power 2. If using a analog power monitor, battery voltage is on pin 6 and current on pin 9 and this should be connected to Power 1.  

## Compass

The E2-Plus has an RM3100 builtin compass, but due to interference the board is usually used with an external I2C or CAN compass as part of a GPS/Compass combination.

## Analog inputs

The E2-Plus has 6 analog inputs.

- ADC Pin9 -> Battery Voltage
- ADC Pin8 -> Battery Current Sensor
- ADC Pin5 -> Vdd 5V supply sense
- ADC Pin13 -> ADC 3.3V Sense
- ADC Pin12 -> ADC 6.6V Sense
- ADC Pin10 -> RSSI voltage monitoring

## Loading Firmware

Firmware for these boards can be found at https://firmware.ardupilot.org in sub-folders labeled "SVehicle-E2".

The board comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of *.apj firmware files with any ArduPilot compatible ground station.
