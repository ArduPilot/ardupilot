# XRush4 H743 V1 Fight Controller

uk_UA: [україномовна документація та прошивки в репозиторії](https://github.com/CO-CF-TECHNO4/XRush4-ArduPilot)

[XRush4 H743 V1 official site](https://xrush4.tech/xrush4-h743-v1/)

## Features

 - STM32H743VIH6 microcontroller
 - x2 ICM42688 IMU
 - DPS310 Baro
 - AT7456E OSD

## Datasheet

![XRush4 H743 V1](XRush4H743V1-board-side1.jpg "XRush4 H743 V1")
![XRush4 H743 V1](XRush4H743V1-board-side2.jpg "XRush4 H743 V1")

## UART Mapping

|Name|Function|
|:-|:-|
|SERIAL0|USB/MAVLink|
|UART1||SerialProtocol_Tramp|
|UART2|SerialProtocol_None|
|UART3|SerialProtocol_ESCTelemetry|
|UART4|SerialProtocol_RCIN|
|UART5|SerialProtocol_None|

## OSD Support

## PWM Output

## Battery Monitoring

## Compass

The XRush4 F4 V3 does not have a builting compass.

## Alternate settings

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.
