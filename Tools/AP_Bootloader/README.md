# ArduPilot Bootloader

This is the bootloader used for STM32 boards for ArduPilot. To build
the bootloader do this:

```bash
 ./waf configure --board BOARDNAME --bootloader
 ./waf bootloader
```

the bootloader will be in build/BOARDNAME/bin. If you have the
intelhex module installed it will build in both bin format and hex
format. Both are usually uploaded with DFU. The elf file will be in
build/BOARDNAME/AP_Bootloader for loading with gdb.

The --bootloader option tells waf to get the hardware config from
the hwdef-bl.dat file for the board. It will look in
libraries/AP_HAL_CHibiOS/hwdef/BOARDNAME/hwdef-bl.dat

The bootloader protocol is compatible with that used by the PX4
project for boards like the Pixhawk. For compatibility purposes we
maintain a list of board IDs in the board_types.txt file in this
directory.
  
the board IDs in that file match the APJ_BOARD_ID in the hwdef.dat and
hwdef-bl.dat files

The bootloader can load from USB or UARTs. The list of devices to load
from is given in the SERIAL_ORDER option in hwdef-bl.dat
