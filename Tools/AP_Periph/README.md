# AP_Periph UAVCAN Peripheral Firmware

This is an ArduPilot based UAVCAN peripheral firmware. This firmware
takes advantage of the wide range of sensor drivers in ArduPilot to
make building a UAVCAN peripheral firmware easy.

The AP_Periph firmware is based on the same ChibiOS hwdef.dat system
that is used to define pinouts for STM32 based flight controllers
supported by ArduPilot. That means you can add support for a new
UAVCAN peripheral based on the STM32 by just writing a simple
hwdef.dat that defines the pinout of your device.

Currently we have three targets building for AP_Periph firmwares:

 - A STM32F103 128k flash part made by mRobotics (target f103-GPS)
 - A STM32F412 512k flash part made by CUAV (target CUAV_GPS)
 - A STM32F105 256k flash part (used in ZubaxGNSSv2)
 - A STM32F303 256k flash part made by mRobotics (target f303-GPS)

More can be added using the hwdef.dat system

# Features

The AP_Periph firmware can be configured to enable a wide range of
UAVCAN sensor types. Support is included for:

 - GPS modules (including RTK GPS)
 - Magnetometers (SPI or I2C)
 - Barometers (SPI or I2C)
 - Airspeed sensors (I2C)
 - Rangefinders (UART or I2C)
 - ADSB (Ping ADSB receiver on UART)
 - LEDs (GPIO, I2C or WS2812 serial)
 - Safety LED and Safety Switch
 - Buzzer (tonealarm or simple GPIO)

An AP_Periph UAVCAN firmware supports these UAVCAN features:

 - dynamic or static CAN node allocation
 - firmware upload
 - automatically generated bootloader
 - parameter storage in flash
 - easy bootloader update
 - high resiliance features using watchdog, CRC and board checks
 - firmware update via MissionPlanner or uavcan-gui-tool

# Building

Using f103-GPS as an example, build the main firmware like this:

 - ./waf configure --board f103-GPS
 - ./waf AP_Periph

that will build a file build/f103-GPS/bin/AP_Periph.bin. You can
now load that using the CAN bootloader and either uavcan_gui_tool or
MissionPlanner SLCAN support.

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

 - Tools/scripts/build_bootloader.py f103-GPS

the resulting bootloader will be in Tools/bootloaders

