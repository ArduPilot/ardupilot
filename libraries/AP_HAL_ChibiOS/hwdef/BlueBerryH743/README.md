# BlueBerry_H743 Flight Controller

The BlueBerry_H743 is a flight controller designed and produced by blueberry

## Features

- STM32H743 microcontroller
- Dual ICM426** IMUs
- 13 PWM / Dshot outputs
- 8 UARTs, one dedicated to the one board BT module
- 1 CAN
- Dedicated USB board
- DPS310 or SPL06 barometer
- 5V/6V/7.2V/8.4V 14A Servo rail BEC
- 9V 3A BEC for VTX
- 5V 3A BEC
- MicroSD Card Slot
- 2-way camera input
- AT7456E OSD
- 2 I2Cs

## Physical

![BlueBerry_H743 overview](BlueBerry_H743_overview.jpg)

![BlueBerry_H743 pinout](BlueBerry_H743_pinout.png)

![BlueBerry_H743 wiring_diagran](BlueBerry_H743_wiring_diagran.png)

## Mechanical

- Dimensions: 34 x 51 x 15 mm
- Weight: 40g

## Power supply

The BlueBerry_H743 supports 3-12s Li battery input. It has 3 ways of BEC. Please see the table below.

| Power symbol | Power source | Max power (current) |
|--------------|--------------|---------------------|
| 5V | from 5V BEC | 15W (3A) |
| 9V | from 9V BEC | 27W (3A) |
| VX | from Servo rail VX BEC, default 5V, can be changed to 6V or 7.2V or 8.4V | 70W (14A) |S

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "with_bl.hex" firmware, using your favorite DFU loading tool, such as Mission Planner.

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

When the SERIALn_PROTOCOL parameter for RCin is 23, it accepts SBUS protocol input; when set to 0, it supports CRSF protocol input.

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

Output 13 can be used as LED neopixel output;

## Battery Monitoring

The board has two internal voltage sensors and one integrated current sensor, and a second external current sensor input.

The voltage sensors can handle up to 12S LiPo batteries.

The first voltage/current sensor is enabled by default and the pin inputs for the second, unenabled sensor are also set by default:

- BATT_MONITOR 4
- BATT_VOLT_PIN 10
- BATT_CURR_PIN 11
- BATT_VOLT_MULT 21
- BATT_AMP_PERVLT 40
- BATT2_VOLT_PIN 18
- BATT2_CURR_PIN 7
- BATT2_VOLT_MULT 21

## Compass

The BlueBerry_H743 has no built-in compass, so if needed, you should use an external compass.

## Analog cameras

The BlueBerry_H743 supports up to 2 cameras, connected to pin CAM1 and CAM2. You can select the video signal to VTX from camera by an RC channel. Set the parameters below:

- RELAY2_FUNCTION = 1
- RELAY_PIN2 = 82
- RC8_OPTION = 34

## Bluetooth

The BlueBerry_H743 support both legacy bluetooth SPP and BLE serial. The bluetooth uses UART7 for its serial port. Search for `BLE` or `SPP` to connect. Be careful NOT to use UART7 for other uses.
