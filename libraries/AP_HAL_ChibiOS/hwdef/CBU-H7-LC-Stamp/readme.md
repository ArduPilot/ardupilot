# H743 Stamp Flight Controller (& Low Cost)

CBUnmanned H743 Stamp

The [CBUnmanned H743 Stamp](https://cbunmanned.com/store) is a flight controller loosely based on the FMUv6 standards & is designed for low volume OEMs as a drop in way to add ArduPilot to their custom hardware builds. It is a part of CBUnmanned's wider ["Stamp" Eco-System](https://cbunmanned.com/), which brings together all the typical avionics hardware into a neat custom carrier PCB. Mounting footprints and symbols are available along with examples of basic usage on the [Wiki](https://wiki.cbunmanned.com/).

A "Low Cost" version is available with a limited feature set.
| Full Featured Stamp | Low Cost Stamp  |
|--|--|
|![](https://wiki.cbunmanned.com/~gitbook/image?url=https%3A%2F%2F1886089318-files.gitbook.io%2F%7E%2Ffiles%2Fv0%2Fb%2Fgitbook-x-prod.appspot.com%2Fo%2Fspaces%252FolH18CdGEWKpuo9G8NHx%252Fuploads%252FqI2QAAVk2RxHplEJavxV%252FH743%2520Side.png%3Falt%3Dmedia%26token%3Db3444773-b155-46d4-98e5-9502bf50b538&width=768&dpr=4&quality=100&sign=d9afa711&sv=1)| ![](https://wiki.cbunmanned.com/~gitbook/image?url=https%3A%2F%2F1886089318-files.gitbook.io%2F%7E%2Ffiles%2Fv0%2Fb%2Fgitbook-x-prod.appspot.com%2Fo%2Fspaces%252FolH18CdGEWKpuo9G8NHx%252Fuploads%252Fy8KdecctQiPiTOfzhqXt%252FH7-LC-Side.png%3Falt%3Dmedia%26token%3D63434aee-4991-493e-88e7-144312333883&width=768&dpr=4&quality=100&sign=6bdaef1&sv=1)|

### 

Features

-   Class leading H7 SOC.
    
-   Direct solder mounting or optional 1.27mm header.
    
-   All complicated/supporting circuitry is on-board, just power with 5v.
    
-   From Just 22mm x 24.25mm

| Item | Full Featured Stamp | Low Cost Stamp |
|--|--|--|
| Dimensions | 22mm x 24mm | 23mm x 25mm |
| Weight | 3g | 3g |
| **Processor** | STM32H743IIK6 | STM32H743VIH6 |
|  | 480MHz | 480MHz |
|  | 2Mb Flash | 2Mb Flash |
|  | 1Mb RAM | 1Mb RAM |
| **Sensors** |  |  |
| IMU 1 | Ivensense ICM-42688 | Ivensense ICM-42670 |
| IMU 2 | Ivensense ICM-42688 | Not Fitted |
| IMU 3 | Ivensense ICM-42670 | Not Fitted |
| Barometer | BMP280 | BMP280 |
| Compass | BMM150  | Not Fitted |
| Micro SD Card | Yes | Yes |
| **IO** |  |  |
| PWM |10  | 10 |
| CAN | 2 | 2 |
| UART | 8 (3 with flow control) | 8 (None with flow control) |
| I2C | 2 | 2 |
| External SPI | 1 (With custom FW build) | 1 (With custom FW build) |
| Ethernet | Yes | No |
| Safety Button | Yes | Yes |
| Analog | 2 | 2 |
| Buzzer (PWM 11) |Yes |Yes |
| USB |Yes |Yes |
| **Power** |  |  |
| Input Voltage | 5v | 5v |
| Typical Current Consumption | 0.4 A | 0.4 A |
| Independent Power Domains | 6 Separate Power Domains | No |

#### 

**UART Mapping**

Ardupilot -> STM32

-   SERIAL0 -> USB
    
-   SERIAL1 -> USART1
    
-   SERIAL2 -> USART2 (With RTS/CTS)*
    
-   SERIAL3 -> USART3
    
-   SERIAL4 -> UART4
    
-   SERIAL5 -> UART5 (With RTS/CTS)*
    
-   SERIAL6 -> USART6 (Sbus In / IO coprocessor if fitted)
    
-   SERIAL7 -> UART7 (With RTS/CTS)*
    
-   SERIAL8 -> UART8
    

*Serial 2, 5 & 7 have RTS/CTS pins (not available on the low cost version), the other UARTs do not have RTS/CTS.

GPS 1 & 2 are on Serial 3 & 4 respectively.

#### 

**RC Input**

RC input is configured on the USART 6 Rx Pin. This pin allows all RC protocols compatible with direct connection to a H7 IC (SBus, CRSF etc), PPM is NOT supported.

USART 6 Tx is available for use with bi directional protocols.

An optional IOMCU can be connected to this serial port, a compatible custom build of the firmware required.

#### 

**CAN Ports**

2 CAN buses are available, each with a built in 120 ohm termination resistor.

#### 

**I2C**

I2C 1 - Internal for BMM150 Compass (not available on the low cost version)

I2C 2 - Internal for BMP280 Barometer

I2C 3 - External With internal 2.2k Pull Up

I2C 4 - External With internal 2.2k Pull Up

#### 

**SPI**

SPI 4 is available for use with external sensors alongside a Chip Select and Data Ready pin, compatible custom build of the firmware required.

#### 

**PWM Output**

The Stamp supports up to 10 PWM outputs with D-Shot.

The PWM outputs are in 3 groups:

-   PWM 1 - 4 in group 1
    
-   PWM 5 - 8 in group 2
    
-   PWM 9 & 10 in group 3
    

Channels within the same group need to use the same output rate. If any channel in a group uses D-Shot then all channels in the group need to use D-Shot.

BiDirectional DShot available on the first 8 outputs.

A buzzer alarm signal is available on PWM 11.

#### 

**Analog Inputs**

The board has two ADC input channels for Voltage (0-3.3v) and Current (0-3.3v) measurement. Settings are dependent on the external hardware used.

#### 

**Ethernet**

Ethernet is available on 4 output pads and has internal magnetics supporting direct connection to external equipment, without the need for a large RJ45 connector. (Not available on the low cost version).

#### 

**Compass**

The H743 Stamp has a built in compass, the BMM150. Due to potential interference the board is usually used with an external I2C or CAN compass as part of a GPS/Compass combination. (Not available on the low cost version).

#### 

**USB**

USB Signals D+ & D- are available to route to a suitable connector for your project.

#### 

**Safety Button**

Optional, if it is not fitted remove the check from arming mask. To activate short this pad to 3.3v with a momentary push button (Press & Hold)

#### 

**Power**

A regulated 3.3v output is available from the stamp for use with the safety button. WARNING! This is shared with the main IC - Do NOT use for accessories. Keep current draw under 0.1A!

The Stamp requires a stable 5v supply input of at least 1.5A. This directly powers the 5v components and supplies the 3.3v LDOs with power. Typical idle usage is 0.35A @ 5v.

### 

**Loading Firmware**

The board comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of *.apj firmware files with any ArduPilot compatible ground station.

A built in button can be used to activate DFU Mode by pressing during power up. The DFU Activate pin is broken out to allow remote mounting of this button if required.

For the full featured please use firmware "CBU-H7-Stamp"

For the low cost please use "CBU-H7-Stamp-LC"
