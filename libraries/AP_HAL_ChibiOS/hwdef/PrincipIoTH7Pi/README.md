# PrincipIoT H7 Pi

The PrincipIoT H7 Pi autopilot is manufactured by
[PrincipIoT](https://principiot.com>). It's designed to be paired with a
Raspberry Pi Zero 2 for low cost onboard computing applications.

![PrincipIoT H7 Pi](H7-Pi-board-image.jpg)

## Where to Buy

[PrincipIoT Website](https://principiot.com>)

## Specifications

- Processor

  - STM32H743 32-bit processor, 480Mhz
  - 2MB Flash
  - 1MB RAM

- Sensors

  - Invensense ICM-42688P
  - Invensense IIM-42653
  - Infineon DPS368 Barometer
  - ST IIS2MDC Magnetometer

- Interfaces

  - Micro SD card
  - USB-C
  - 9 PWM (6 motors, 2 servos, and LED strip)
  - 6 UARTS, one shared with the Raspberry Pi
  - CAN
  - ESC Connector with current sense and telemetry inputs
  - VTX Connector with UART and analog video from the Raspberry Pi
  - Debug port

- Power

  - Integrated voltage/current power monitor 11V - 30V (3S - 6S) input
  - 10V GPIO controlled Video power BEC, 2A output
  - 5V, 2A output for board, Raspberry Pi, and peripherals

- Dimensions

  - Size: 65 x 30 x 9 mm (20mm high with Raspberry Pi installed)
  - Weight: 11.3g with MicroSD card

For more information, see the
[PrincipIoT Wiki](https://principiot.gitbook.io/principiot-docs/h7-pi/).

## Pinout

![Front side](H7-Pi-Rev-3-Pinout-Front.png)
![Back side](H7-Pi-Rev-3-Pinout-Back.png)

## UART Mapping

The UARTs are marked in the above pinouts. The RX pin is the receive pin
for the MCU (input). The TX pin is the transmit pin for the MCU (output).

|Port        | UART    |Protocol        |Function |
|------------|---------|----------------|---------|
|SERIAL0     |  USB    |  MAVLink2      |         |
|SERIAL1     |  USART2 |  Unused        |Debug    |
|SERIAL2     |  USART3 |  MAVLink2      |Pi UART  |
|SERIAL3     |  UART4  |  GPS           |GPS      |
|SERIAL4     |  USART6 |  DisplayPort   |VTX      |
|SERIAL5     |  UART7  |  ESC Telem     |ESC Telem|
|SERIAL6     |  UART8  |  RCin          |RX       |

All UARTs support DMA. Any UART can change function by changing the related
protocol parameter.
USART3 is wired to the UART on the 40 pin Raspberry Pi header and the external
UART connector.
UART7 is receive only.

## RC Input

The PrincipIoT H7 Pi defaults SERIAL 6 as RC input. It can be used with any
ArduPilot compatible serial protocol. It does not support PPM protocols.
SBUS/DSM would connect to UART8 RX input.

- FPort would require [SERIAL6_OPTIONS](https://ardupilot.org/copter/docs/parameters.html#serial6-options-serial6-options) be set to "15"
 and connect to the TX pin.
- CRSF would require [SERIAL6_OPTIONS](https://ardupilot.org/copter/docs/parameters.html#serial6-options-serial6-options) be set to "0"
 and use both RX and TX pins.
- SRXL2 would require [SERIAL6_OPTIONS](https://ardupilot.org/copter/docs/parameters.html#serial6-options-serial6-options) be set to "4"
 and connects only the TX pin.

Any other UART can be used for RC system connections in ArduPilot also, see
[RC systems](https://ardupilot.org/copter/docs/common-rc-systems.html) for details.

## PWM Output

The PrincipIoT H7 Pi controller supports up to 8 PWM outputs.

The PWM outputs are in 3 groups:

- PWM 1 - 4 in group 1 (Timer 5)
- PWM 5 & 6 in group 2 (Timer 3)
- PWM 7 & 8 in group 3 (Timer 15)

Channels within the same group need to use the same output rate. If any
channel in a group uses D-Shot then all channels in the group need to use
D-Shot.

All outputs are directly wired to the H743 MCU. All 8 outputs support normal
PWM output formats. All outputs support DShot, outputs 1-6 support
Bi-Directional DShot.

## Analog Video and OSD Support

The Raspberry Pi Zero 2 support analog video output through a pad on its PCB.
Installing the included POGO pin connects this pad to the H7 Pi and routes the
analog video from the Raspberry Pi to the analog video pin on the VTX
connector.

The SH1.0-5P connector supports a connection to a VTX. Protocol defaults to
DisplayPort. Pin 1 of the connector is +10V so be careful not to connect this
to a peripheral requiring 5v. DisplayPort OSD is enabled by default on SERIAL5.

## VTX Power Control

GPIO 81 controls the VTX BEC output to pins marked "10V" and is included on
the HD VTX connector. Setting this GPIO high turns off the voltage regulator.
By default this pin is set low (regulator on) at boot and is controlled by
RELAY2.

## Camera Switch

GPIO 82 controls the PINIO2 GPIO pin which can be used as a camera switch or
other user defined function. It defaults to low on boot.

## Compass

An on-board IIS2MDC compass is provided. This compass is on the external I2C bus.
Do not connect an external IIS2MDC compass or both may not be detected as they
only have one address. If you need to use an external compass, use a different
model or desolder the onboard one.

## GPIOs

|Pin           |GPIO Number |Function        |
|--------------|------------|----------------|
|PWM(1)        | 50         |Motor           |
|PWM(2)        | 51         |Motor           |
|PWM(3)        | 52         |Motor           |
|PWM(4)        | 53         |Motor           |
|PWM(5)        | 54         |Motor           |
|PWM(6)        | 55         |Motor           |
|PWM(7)        | 60         |Servo           |
|PWM(8)        | 61         |Servo           |
|PWM(9)        | 62         |LED Strip       |
|ALARM         | 32         |Buzzer          |
|VTX PWR       | 81         |10V BEC Enable  |
|USER SW       | 82         |GPIO Switch     |
|LED0          | 90         |Blue LED        |
|LED1          | 91         |Green LED       |

## Battery Monitoring

The board has a internal voltage sensor and connections on the ESC connector
for an external current sensor input.
The voltage sensor can handle up to 6S LiPo batteries.

The default battery parameters are:

- [BATT_MONITOR](https://ardupilot.org/copter/docs/parameters.html#batt-monitor-battery-monitoring) = 4
- [BATT_VOLT_PIN](https://ardupilot.org/copter/docs/parameters.html#batt-volt-pin-ap-battmonitor-analog-battery-voltage-sensing-pin) = 10
- [BATT_CURR_PIN](https://ardupilot.org/copter/docs/parameters.html#batt-curr-pin-ap-battmonitor-analog-battery-current-sensing-pin) = 11 (CURR pin)
- [BATT_VOLT_MULT](https://ardupilot.org/copter/docs/parameters.html#batt-volt-mult-ap-battmonitor-analog-voltage-multiplier) = 11.0
- [BATT_AMP_PERVLT](https://ardupilot.org/copter/docs/parameters.html#batt-amp-pervlt-ap-battmonitor-analog-amps-per-volt) = 40

## Firmware

Firmware for the PrincipIoT H7 Pi is available from
[ArduPilot Firmware Server](https://firmware.ardupilot.org) under
the `PrincipIoTH7Pi` target.

## Loading Firmware

To flash firmware initially, connect USB while holding the bootloader
button and use DFU to load the `with_bl.hex` file. Subsequent updates can
be applied using `\.apj` files through a ground station.
