# CyberX-v10 Flight Controller

This firmware is compatible with CyberX-v10.

![CyberX-v10 Board](CyberX-v10_right.png)

## Features

* Separate flight control core design.
* MCU
  STM32H743IIK6 32-bit processor running at 400MHz
  2MB Flash
  1MB RAM
* IO MCU
  STM32F103 at 72MHz 128KB Flash 20kB SRAM
* Sensors
* IMU:
  Internal Vibration Isolation for IMUs,
  IMU constant temperature heating(1 W heating power).
  With Triple Synced IMUs, BalancedGyro technology, low noise and more shock-resistant:
  IMU1-BMI088(With vibration isolation),
  IMU2-ICM42688-P(With vibration isolation),
  IMU3-ICM20689(No vibration isolation)
* Baro:
  Two barometers:Baro1-BMP581, Baro2-ICP20100
* Magnetometer:
  A builtin IST8310 magnetometer

## Pinout

![CyberX-v10 Board](CyberX-v10_left.png)

![CyberX-v10 Board]( CyberX-v10_pinout.jpg)

## UART Mapping

The UART are marked RX and TX in the above pinouts. The RX pin is the receive pin for UART. The TX pin is the transmit pin for UART.

|Name|Label/Protocol|UART|DMA|
|:-:|:-:|:-:|:-:|
|SERIAL0|USB, MAVLink2|OTG||
|SERIAL1|Telem1, MAVLink2|USART2|DMA Enabled|
|SERIAL2|Telem2, MAVLink2|USART3|DMA Enabled|
|SERIAL3|GPS1, GPS|USART1|DMA Enabled|
|SERIAL4|GPS2, GPS|UART4|DMA Enabled|
|SERIAL5|DEBUG, MAVLink2|UART5|DMA Enabled|
|SERIAL6|RCIN, RCIN|UART8|DMA Enabled|
|SERIAL7|RESERVE, None|UART7|DMA Enabled|
|SERIAL8|OTG-SLCAN, MAVLink2|USB|

The TEL1 and TEL2 ports have RTS/CTS pins, the other UART do not have RTS/CTS.

## RC Input

The RCIN pin, which by default is mapped to a timer input, can be used for all ArduPilot supported unidirectional receiver protocols. Bi-Directiona protocols, such as  CRSF/ELRS and SRXL2, require a true UART connection. However, FPort, connected to RCIN, can provide RC without telemetry.

To allow CRSF and embedded telemetry available in Fport, CRSF, and SRXL2 receivers, a full UART, such as SERIAL6 (UART8) would need to be used for receiver connections. Below are setups using Serial6, which is already set up by default for bi-directional RC input.

* CRSF would require `SERIAL6_OPTION`  set to "0".
* SRXL2 would require `SERIAL6_OPTIONS` set to "4". And only connect the TX pin.

* The SBUS_IN pin is internally tied to the RCIN pin of RCIN port.

Any UART can also be used for RC system connections in ArduPilot and is compatible with all protocols except PPM.
See [Radio Control Systems](https://ardupilot.org/copter/docs/common-rc-systems.html) for details.

## PWM Output

The CyberX flight controller supports up to 16 PWM outputs.
First 8 outputs (labelled 1 to 8) are controlled by a dedicated STM32F103 IO controller.
The remaining 8 outputs (labelled 9 to 16) are the "auxiliary" outputs. These are directly attached to the STM32H743 FMU controller.
All 16 outputs support normal PWM output formats. All 16 outputs support DShot, except 15 and 16.

The 8 IO PWM outputs are in 3 groups:

* Outputs 1 and 2 in group1
* Outputs 3 and 4 in group2
* Outputs 5, 6, 7 and 8 in group3

The 8 FMU PWM outputs are in 3 groups:

* Outputs 9, 10, 11 and 12 in group1
* Outputs 13 and 14 in group2
* Outputs 15 and 16 in group3

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

## GPIO

All PWM outputs can be used as GPIOs (relays, camera, RPM etc). To use them you need to set the output’s SERVOx_FUNCTION to -1. The numbering of the GPIOs for PIN variables in ArduPilot is:

* M1 101
* M2 102
* M3 103
* M4 104
* M5 105
* M6 106
* M7 107
* M8 108
* M9  50
* M10 51
* M11 52
* M12 53
* M13 54
* M14 55
* M15 56
* M16 57

## CAN

The CyberX has two independent CAN buses.

## Battery Monitoring

Two power monitor interfaces have been configured:  one analog (PWR1) and one I2C(INA2xx) , PWR2 port.

The PWR1 default battery parameters are:

* BATT_MONITOR_DEFAULT=4
* BATT_VOLT_PIN=16
* BATT_CURR_PIN=18
* BATT_VOLT_SCALE=15.5
* BATT_AMP_PERVLT=50

The PWR2 default battery parameters are:

* BATTMON_INA2XX_BUS=0
* BATTMON_INA2XX_ADDR=64
* BATT2_MONITOR_DEFAULT=21
* BATTMON_INA2XX_SHUNT=0.0005

## Compass

The CyberX flight controllers have an integrated IST8310 high-precision magnetometer.
Often, users will need to disable the internal compass and use an external compass, appropriated located, to avoid electrical interference.

## Analog inputs

The CyberX flight controller has 5 analog inputs.

* ADC Pin16 -> Battery Voltage monitoring
* ADC Pin18 -> Battery Current monitoring
* ADC Pin13 -> ADC 6.6V Sense
* ADC Pin12 -> ADC 3.3V Sense
* ADC Pin9 -> RSSI voltage monitoring

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of xxxxxx.apj firmware files with any ArduPilot compatible ground station.

Firmware for these boards can be found [here](https://firmware.ardupilot.org/) in sub-folders labeled “CyberX-v10”.
