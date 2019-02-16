# ArduPilot Bootloader

The main ArduPilot for STM32 boards is based on ChibiOS, and the
source code is in Tools/AP_Bootloader

The old (now unusued) bootloader was based on libopencm3, and a copy
of that is here:

  https://github.com/ArduPilot/Bootloader

## Bootloader images

Binaries for the bootloader for all supported STM32 boards are in this
directory. You can build one of these bootloaders using the
Tools/scripts/build_bootloaders.py script. That script takes a
wildcard pattern of which boards to build. For example, to build a
bootloader for a Pixhawk1, use:

 ./Tools/scripts/build_bootloaders.py Pixhawk1

The config files for the builds of these bootloaders are in
libraries/AP_HAL_ChibiOS/hwdef, in the files called hwdef-bl.dat

## Old Bootloader details

We also have copies of binaries for some of the older bootloaders in
this directory. They are:

px4fmu_bl.bin:
   for F405 based fmuv1
   boots at 0x08004000
   board ID 5

px4fmuv2_bl.bin:
   for F427 based fmuv2 boards
   boots at 0x08004000
   board ID 9

px4fmuv4_bl.bin:
   for F427 based fmuv4 boards
   boots at 0x08004000
   board ID 11

iomcu_bl.bin:
   for IOMCU on fmuv2

px4fmuv4pro_bl.bin:
   for F427 based p4pro board from drotek

skyviper_v2450_bl.bin:
   for F427 based skyviper-v2450 boards (based on fmuv3)
   reserves sectors 22 and 23 for ArduPilot storage
   board ID 9

skyviperf412_bl.bin:
   For F412 processors, setup to load on USART2 on pins PA2/PA3
   Setup for code start at 0x0800C000
   2 sectors reserved for storage, plus bootloader

# Bootloader in ROMFS

When building a ChibiOS based build, if there is a corresponding
bootloader in Tools/bootloaders for the board then that bootloader
will be included as a compressed file in ROMFS. You can then ask
ArduPilot to update the bootloader using the MAVLink command
MAV_CMD_FLASH_BOOTLOADER command using a param5 magic value of 290876

