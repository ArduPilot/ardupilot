# Atlas Control Flight Controller

The Atlas Control flight controller 
https://altasflight.github.io/atlas-docs/

## Features

 - STM32H743 microcontroller
 - Three IMUs, ICM-42688-P, ICM-20689 and ICM-20689
 - internal heater for IMU temperature control
 - two MS5611 SPI barometers
 - internal RM3100 compass
 - microSD card slot
 - 4 UARTs plus USB
 - ADSB receiver connected to SERIAL5
 - 14 PWM outputs
 - I2C and CAN ports
 - External Buzzer
 - builtin RGB LED
 - voltage monitoring for servo rail and Vcc
 - two dedicated power input ports for external power bricks
 - external USB connectors (micro USB and JST GH)



## Pinout

![AtlasControl Board](img.png "Atlas Control")

- 14 PWM servo outputs（12 support DShot）

- Analog/ PWM RSSI input

- 2 GPS ports(GPS and UART4 ports)

- 4 I2C buses(Two external I2C ports)

- 2 CAN bus ports

- 2 Power ports(Power A is an analog PMU interface, Power C is a DroneCAN PMU interface)

- 2 ADC inputs

- 1 USB-C port
## UART Mapping

SERIAL0 = console = USB (MAVLink2)

SERIAL1 = Telemetry1 (MAVlink2 default)= USART2 DMA-enabled

SERIAL2 = Telemetry2 (MAVlink2 default)= USART6 DMA-enabled

SERIAL3 = GPS1 = USART1

SERIAL4 = GPS2 = UART4

SERIAL6 = USER = UART7

## Connectors

Unless noted otherwise all connectors are JST GH 1.25mm pitch

### TELEM1, TELEM2 ports

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin </th>
   <th>Signal </th>
   <th>Volt </th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>TX (OUT)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>RX (IN)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>CTS</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>5 (blk)</td>
   <td>RTS</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>6 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

   <td>+3.3V</td>
   </tr>
   <tr>
   <td>2</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   <tr>
   <td>3</td>
   <td>3.3v</td>
   <td>+3.3V</td>
   </tr>
   </tbody>
   </table>


### FMU and IO SWD

When the case is removed there are two SWD connectors, one for FMU and
the other for IOMCU. The IO SWD connector is the one closer to the
servo rail. The GND pin of both connectors is the one furthest from
the servo rail.

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2</td>
   <td>TX</td>
   <td>+3.3</td>
   </tr>
   <tr>
   <td>3</td>
   <td>RX</td>
   <td>+3.3</td>
   </tr>
   <tr>
   <td>4</td>
   <td>SWDIO</td>
   <td>+3.3</td>
   </tr>
   <tr>
   <td>5</td>
   <td>SWCLK</td>
   <td>+3.3</td>
   </tr>
   <tr>
   <td>6</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>


### CAN1&2

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>CAN_H</td>
   <td>+12V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>CAN_L</td>
   <td>+12V</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>


### POWER_A

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>CURRENT</td>
   <td>up to +3.3V</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>VOLTAGE</td>
   <td>up to +3.3V</td>
   </tr>
   <td>5 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   <td>6 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

### POWER_C

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>CAN_H</td>
   <td>up to +3.3V</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>CAN_L</td>
   <td>up to +3.3V</td>
   </tr>
   <td>5 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   <td>6 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>


### USB

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin </th>
   <th>Signal </th>
   <th>Volt </th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>D_plus</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>D_minus</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   <tr>
   <td>5 (blk)</td>
   <td>BUZZER</td>
   <td>battery voltage</td>
   </tr>
   <tr>
   <td>6 (blk)</td>
   <td>Boot/Error LED</td>
   <td></td>
   </tr>
   </tbody>
   </table>

## RC Input
 
RC input is configured on the RCIN pin, at one end of the servo rail,
marked RCIN in the above diagram. This pin supports all RC
protocols. In addition there is a dedicated Spektrum satellite port
which supports software power control, allowing for binding of
Spektrum satellite receivers.

## PWM Output

The 14 PWM outputs are in 4 groups:

Outputs 1, 2, 3 and 4 in group1 (these also support Bi-dir DShot if the Bi-Dir firmware version is used)

Outputs 5, 6, 7 and 8 in group2

Outputs 9, 10, 11 and 12 in group3

Outputs 13 and 14 in group4（NO DMA)

## Battery Monitoring

The board has two dedicated power monitor ports on 6 pin
connectors. The correct battery setting parameters are dependent on
the type of power brick which is connected.

## Compass

The Atlas Control has an RM3100 builtin compass.

## GPIOs

The 14 outputs can be used as GPIOs (relays, buttons, RPM etc). To use them you need to set the output’s SERVOx_FUNCTION to -1. See GPIOs page for more information.
The numbering of the GPIOs for use in the PIN parametersin ArduPilot is:

pins:

- PWM1(M1) 50
- PWM2(M2) 51
- PWM3(M3) 52
- PWM4(M4) 53
- PWM5(M5) 54
- PWM6(M6) 55
- PWM7(M7) 56
- PWM8(M8) 57
- PWM9(M9) 58
- PWM10(M10) 59
- PWM11(M11) 60
- PWM12(M12) 61
- PWM13(M13) 62
- PWM14(M14) 63


## IMU Heater

The IMU heater in the Atlas Control can be controlled with the
BRD_HEAT_TARG parameter, which is in degrees C.

## Loading Firmware

The board comes pre-installed with an ArduPilot compatible bootloader,
allowing the loading of *.apj firmware files with any ArduPilot
compatible ground station.


