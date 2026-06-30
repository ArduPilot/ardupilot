# BlueBerry_H743 Flight Controller

The BlueBerry_H743 is a flight controller designed and produced by blueberry

## Features

- STM32H743 microcontroller
- Dual ICM426** IMUs
- 13 PWM / Dshot outputs
- 8 UARTs, one dedicated to the one board BT module
- 1 CAN
- Dedicated remote USB board with buzzer and boot switch
- DPS310 or SPL06 barometer
- 5V/6V/7.2V/8.4V 14A Servo rail BEC provided
- 9V 3A BEC for VTX
- 5V 3A BEC
- MicroSD Card Slot
- SGPIO switchable dual analog camera inputs
- AT7456E OSD
- 2 external I2Cs

## Physical

![BlueBerry_H743 overview](BlueBerry_H743_overview.jpg)
## Pinout

![BlueBerry_H743 pinout](BlueBerry_H743_pinout.png)

![BlueBerry_H743 wiring_diagran](BlueBerry_H743_wiring_diagran.png)

## Mechanical

- Dimensions: 34 x 51 x 15 mm
- Weight: 40g

## Power supplies

The BlueBerry_H743 supports 3-12s Li battery input. It provides 3 on-board BEC regulators. Please see the table below.

| Power symbol | Power source | Max power (current) |
|--------------|--------------|---------------------|
| 5V | from 5V BEC | 15W (3A) |
| 9V | from 9V BEC | 27W (3A) |
| VX | Servo rail VX BEC, default 5V, can be changed to 6V or 7.2V or 8.4V | 70W (14A) |

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "xxx_with_bl.hex" firmware, using your favorite DFU loading tool, such as the STM32CubeProgrammer.

Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the "\*.apj" firmware files.

Supports SWD for program downloading, can be used for debugging and secondary development, with program download speed significantly better than DFU.

## UART Mapping

All UARTs are DMA capable.

- SERIAL0 -> USB
- SERIAL1 -> UART1 (MAVLink2)
- SERIAL2 -> UART2 (GPS)
- SERIAL3 -> UART3 (MAVLink2)
- SERIAL4 -> UART4 (GPS2, RX4 is also available as ESC telem if protocol is changed for this UART)
- SERIAL5 -> USB (SLCAN)
- SERIAL6 -> UART6 (RCIN)
- SERIAL7 -> UART7 (MAVLink2, Integrated Bluetooth module)
- SERIAL8 -> UART8 (User)

## RC Input

RC input is provided on UART6 for all ArduPilot supported protocols except PPM. The SBUS pin on the HD VTX connector is tied to RX6. See [RC Systems](https://ardupilot.org/plane/docs/common-rc-systems.html) for details for each protocol type.

## OSD Support

The BlueBerry_H743 supports onboard analog SD OSD using a AT7456 chip. The analog VTX should connect to the VTX pin.

## PWM Output

The BlueBerry_H743 supports up to 13 PWM outputs.

All the channels support DShot.

Outputs are grouped and every output within a group must use the same output protocol:

1, 2 are Group 1;

3, 4, 5, 6 are Group 2;

7, 8, 9, 10 are Group 3;

11, 12 are Group 4;

13(LED) is Group 5;

Output 13 , marked LED, can be used as a  neopixel output

## Battery Monitoring

The board has two internal voltage sensors and one integrated current sensor, and a second external current sensor input.

The voltage sensors can handle up to 12S LiPo batteries.

The first voltage/current sensor is enabled by default and the pin inputs and voltage scale for the second, unenabled, sensor are also set by default:

- BATT_MONITOR 4
- BATT_VOLT_PIN 10
- BATT_CURR_PIN 11
- BATT_VOLT_MULT 21
- BATT_AMP_PERVLT 40
- BATT2_VOLT_PIN 18
- BATT2_CURR_PIN 7
- BATT2_VOLT_MULT 21

## Compass

The BlueBerry_H743 has no built-in compass. An external compass can be attached to either set of SDA/SCL pins or via DroneCAN.

## Camera Switch

The BlueBerry_H743 supports up to 2 cameras, connected to pin CAM1 and CAM2. You can select whcih camera is used by an RC channel. Set the parameters below (RC 6 used in the example:

- RELAY2_FUNCTION = 1
- RELAY2_PIN = 82
- RC6_OPTION = 34

## Bluetooth

The BlueBerry_H743 support both legacy bluetooth SPP and BLE serial. The bluetooth uses UART7 for its serial port. Search for `BLE` or `SPP` to connect. Be careful NOT to use UART7 for other uses.
