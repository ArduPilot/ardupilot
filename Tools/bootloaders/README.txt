These FMU and FMUv2 bootloader images are built from:

  https://github.com/ArduPilot/Bootloader

On PX4 builds they are in ROMFS to make it easier for users to update
their bootloaders using:

 bl_update /etc/bootloader/fmu_bl.bin

from a nsh prompt. Users can get a nsh prompt either via the CLI in
test -> shell, or by booting with no SD card installed

to use NSH to do this, please see the Wiki
http://dev.ardupilot.org/wiki/interfacing-with-pixhawk-using-the-nsh

