These FMU and FMUv2 bootloader images are built from:

  https://github.com/ArduPilot/Bootloader

Bootloader details
------------------

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

px4io_bl.bin:
   for IOMCU on fmuv1

F4LightF405_bl.bin:
   for F405 based boards running F4Light HAL
   boots at 0x08010000
   reserve sector at 0x08004000 for OSD storage
   reserves sectors at 0x08008000 and 0x0800C000 for ArduPilot storage
   board ID 70

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

Bootloader update on px4 builds
-------------------------------

For ArduPilot builds using HAL_PX4 the bootloaders are stored in ROMFS
to make it easier for users to update their bootloaders using:

 bl_update /etc/bootloader/fmu_bl.bin

from a nsh prompt. Users can get a nsh prompt either via the CLI in
test -> shell, or by booting with no SD card installed

to use NSH to do this, please see the Wiki
http://dev.ardupilot.org/wiki/interfacing-with-pixhawk-using-the-nsh
