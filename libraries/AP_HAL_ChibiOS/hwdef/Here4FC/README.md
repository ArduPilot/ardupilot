# Here4 GPS Module as Flight Controller

Here4 Module is sold as a GPS module, but it can be used as a flight controller.

## Features
- STM32H757 MCU
- ICM45686 IMU
- MS5611 Barometer
- RM3100 Compass
- 1x UART
- 2x CAN
- 8x PWM
- 1x RCIN
- 1x UBLOX NEO-F9P L1L5 GNSS

## How to use
- To load the FC firmware on production Here4. User needs to first update Here4 through DroneCAN tool or Mission Planners DroneCAN config using FC firmware, which is .bin format.
- After first update, users can connect through the serial port. Check https://docs.cubepilot.org/user-guides/here-4/here-4-manual#pinout for breakout board pinouts.
- For enabling future updates to FC firmware, use flash bootloader through mission planner, and then user should be able to update firmware through Serial port.
- Please note that once the firmware is updated to FC, to rollback to GPS firmware user will need to update using APJ file available here https://github.com/CubePilot/GNSSPeriph-release/releases.

## Pinout and Connectors

https://docs.cubepilot.org/user-guides/here-4/here-4-manual#pinout
