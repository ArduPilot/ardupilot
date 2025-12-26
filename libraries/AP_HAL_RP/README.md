# AP_HAL_RP
ArduPilot Hardware Abstraction Layer (HAL) for Raspberry Pi RP series MCUs.

## Overview
This HAL allows ArduPilot flight control software to run on RP2040-based flight controllers, utilizing the Raspberry Pi Pico C/C++ SDK and its built-in FreeRTOS support.

## Features
- Uses an approach similar to AP_HAL_ESP32, fully isolated from the ChibiOS infrastructure.
- Integration with the Pico SDK for low-level peripheral access.
- Multitasking support via FreeRTOS.
- Waf-based build system using custom rp_hwdef.py scripts.

## Supported Boards
- [List of boards will be added here as they are integrated]
- Default development board: <a href=https://github.com/bastian2001/Kolibri-FC-Hardware>Kolibri-FC</a>

## Building the Firmware

*Note*: Build was tested on Ubuntu 24.04 LTS

1. Run the following script from the cloned ardupilot directory to install required packages:
```
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
2. Reload the path (log-out and log-in to make it permanent):
```
. ~/.profile
```
3. Install Raspberry Pi Pico C/C++ SDK and FreeRTOS kernel:
```
Tools/scripts/get_pico_sdk.sh -d <path_to_install>
```
You can install Pico SDK and FreeRTOS kernel into ${HOME}/pico, just run the installation script without any parameters: 
```
Tools/scripts/get_pico_sdk.sh
```
4. Configure and build the firmware:
```
./waf configure --board Kolibri
./waf copter
```
*Note*: The first configure command should be called only once or when you want to change a
configuration option. One configuration often used is the `--board` option to
switch from one board to another one.

## Uploading the Firmware

#### Flashing the RP2350 firmware is a simple drag-and-drop process:

- Disconnect the flight controller from power.
- Press and hold the BOOT button and connect the USB cable.
- A new USB mass storage device named RPI-RP2 will appear on your computer.
- Copy the generated arducopter.uf2 file to this drive.
- The drive will automatically unmount, and the board will reboot with the new ArduPilot firmware running.

## Hardware Definition (hwdef)

The configuration for Input/Output (GPIO) pins, SPI/I2C/UART buses, and sensors is defined in the hwdef.dat files for each board, located in the libraries/AP_HAL_RP2040/hwdef/ subdirectory.

## Contributing

Bug fixes, improvements, and support for new RP2040-based flight controllers are welcome. Please refer to the ArduPilot Developer Guidelines before submitting pull requests.

