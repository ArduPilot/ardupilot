*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M3 STM32F103 on LeafLabs MapleMini       **
*****************************************************************************

** TARGET **

The demo runs on the MapleMini with the original MapleMini bootloader.

You can also use this firmware without the original bootloader. To do so,
pass USE_MAPLEMINI_BOOTLOADER=0 to make:

> make USE_MAPLEMINI_BOOTLOADER=0

** The Demo **

This demo flashes the board LED using a thread. Also, a simple command shell
is activated on virtual serial port via USB-CDC driver.

** Build Procedure **

The demo has been tested by using the free arm gcc embedded toolchain.
Just modify the TRGT line in the makefile in order to use different GCC ports.

** Flashing Target Board **

To flash the firmware to the maplemini, you can use dfu-util. While the MapleMini is connected via USB and
still in bootloader mode, just run

     > dfu-util -a1 -d 1eaf:0003 -D build/ch.bin -R

If you compiled without the bootloader support, you will have to flash via the
original serial flashing method or via SWD. For the serial method please see
http://static.leaflabs.com/pub/leaflabs/maple-docs/latest/bootloader.html#id7
for a howto. there you can also get the official bootloader files.
As a quick reference:

  1) connect TTL UART:
     TX to maple rx1 ("25")
     RX to tx1 ("26")
     GND to gnd
     3.3V to vcc  or  5V to vin
  
  2) hold RESET and BUT buttons, release RESET, then after a sec BUT
     (this sets the STM32 into serial upload mode)
  
  3) use stm32loader.py from the link above to upload the firmware
     > stm32loader.py -p <uart-device> -evw build/ch.bin
  
  4) reset device

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com
