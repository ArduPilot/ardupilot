# AP_Periph DroneCAN Peripheral Firmware

This is an ArduPilot based DroneCAN peripheral firmware. This firmware
takes advantage of the wide range of sensor drivers in ArduPilot to
make building a DroneCAN peripheral firmware easy.

The AP_Periph firmware is based on the same ChibiOS hwdef.dat system
that is used to define pinouts for STM32 based flight controllers
supported by ArduPilot. That means you can add support for a new
DroneCAN peripheral based on the STM32 by just writing a simple
hwdef.dat that defines the pinout of your device.

We have over 60 build targets building for AP_Periph firmwares. All
ArduPilot supported MCUs can be used, including:

 - STM32F1xx
 - STM32F3xx
 - STM32F4xx
 - STM32F7xx
 - STM32H7xx
 - STM32L4xx
 - STM32G4xx

More can be added using the hwdef.dat system

# Features

The AP_Periph firmware can be configured to enable a wide range of
DroneCAN sensor types. Support is included for:

 - GPS modules (including RTK GPS)
 - Magnetometers (SPI or I2C)
 - Barometers (SPI or I2C)
 - Airspeed sensors (I2C)
 - Rangefinders (UART or I2C)
 - ADSB (Ping ADSB receiver on UART)
 - Battery Monitor (Analog, I2C/SMBus, UART)
 - LEDs (GPIO, I2C or WS2812 serial)
 - Safety LED and Safety Switch
 - Buzzer (tonealarm or simple GPIO)
 - RC Output (All standard RCOutput protocols)
 - RC input
 - battery balance monitor
 - EFI engines
 - Proximity sensors

An AP_Periph DroneCAN firmware supports these DroneCAN features:

 - dynamic or static CAN node allocation
 - firmware upload
 - automatically generated bootloader
 - parameter storage in flash
 - easy bootloader update
 - high resiliance features using watchdog, CRC and board checks
 - firmware and parameter update via MissionPlanner or DroneCAN GUI tool when attached to an autopilot
 - firmware update via USB if USB port is provided
 - parameter update using SLCAN and DroneCAN GUI on standalone peripheral via USB, if provided

# Building

Using f103-GPS as an example, build the main firmware like this:

 - ./waf configure --board f103-GPS
 - ./waf AP_Periph

that will build a file build/f103-GPS/bin/AP_Periph.bin. You can
now load that using the CAN bootloader and either dronecan_gui_tool or
MissionPlanner DroneCAN support.

# Flashing

To load directly with a stlink-v2, do this:

 - st-flash write build/f103-GPS/bin/AP_Periph.bin 0x8006400

for the CUAV_GPS which loads at offset 0x10000 use this:

 - st-flash write build/CUAV_GPS/bin/AP_Periph.bin 0x8010000

# Flashing bootloader

To flash the bootloader use this:

 - st-flash write Tools/bootloaders/f103-GPS_bl.bin 0x8000000

# Building bootloader

To build the bootloader use this:

 - Tools/scripts/build_bootloaders.py f103-GPS

the resulting bootloader will be in Tools/bootloaders

# Firmware Builds

Firmware targets are automatically built and distributed on the
ArduPilot firmware server on firmware.ardupilot.org. These firmwares
can be loaded using Mission Planner or the DroneCAN GUI Tool. Parameters
for peripherals can be changed using the Mission Planner DroneCAN support
or using DroneCAN GUI Tools.

# User Bootloader Update

The bootloader is automatically stored in ROMFS in the main
firmware. End users can update the bootloader by setting the DroneCAN
parameter "FLASH_BOOTLOADER" to 1. After setting it to 1 the node will
respond with a debug text message which can be seen in the DroneCAN GUI
tool to show the result of the flash.

# SITL Testing

A wide range of DroneCAN peripherals are supported in the SITL
simulation system. The simplest way of starting a DroneCAN enabled
simulated vehicle is to use sim_vehicle.py.

For a quadplane use: sim_vehicle.py with the option -f quadplane-can

For a quadcopter use: sim_vehicle.py with the option -f quad-can

# Discussion and Feedback

Please join the discussions at these locations:

 - https://discuss.ardupilot.org/
 - https://ardupilot.org/discord

