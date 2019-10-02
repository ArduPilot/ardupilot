# CAN Peripheral Firmware

This is an ArduPilot based CAN peripheral firmware. Currently two
targets are supported:

 - A STM32F103 128k flash part made by mRobotics (target f103-GPS)
 - A STM32F412 512k flash part made by CUAV (target CUAV_GPS)

# Building

Using f103-GPS as an example, build the main firmware like this:

 - ./waf configure --board f103-GPS
 - ./waf AP_Periph

that will build a file build/f103-GPS/bin/AP_Periph.bin. You can
now load that using the CAN bootloader and either uavcan_gui_tool or
MissionPlanner.

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

