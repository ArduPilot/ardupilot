# XRush4 F4 V3 Flight Controller

uk_UA: [україномовна документація та прошивки в репозиторії](https://github.com/CO-CF-TECHNO4/XRush4-ArduPilot)

[XRush4 F4 V3 official site](https://xrush4.tech/xrush4-f4-v3/)

## Features

 - STM32F405 microcontroller
 - ICM42688 IMU
 - BMP280 Baro
 - AT7456E OSD

## Datasheet

[Xrush4_F4V3_Manual_Rev.2.0](https://xrush4.tech/wp-content/uploads/2025/05/Xrush4_F4V3_Manual_Rev.2.0.pdf)

![XRush4 F4 V3](XRush4F4V3-board-side1.jpg "XRush4 F4 V3")

## UART Mapping

|Name|Function|
|:-|:-|
|SERIAL0|USB/MAVLink|
|UART1|SerialProtocol_Tramp|
|UART2|SerialProtocol_ESCTelemetry|
|UART3|SerialProtocol_GPS|
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
