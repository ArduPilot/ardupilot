*****************************************************************************
** ChibiOS/RT port for ATxmega128a4u.                                      **
*****************************************************************************

** TARGET **

The demo runs on the mattair Tech MT-DB-X4 board.

** The Demo **

The demo demonstrate the use of the SPI driver. Every time an Hello world
massage is printed to the serial console, four bytes are also send to the
spi bus.
To see the messages, you need to connect an FTDI converter to 
USARTC0 (PINC3 is MCU TX and PINC2 is MCU RX).

** Build Procedure **

The demo was built using the GCC AVR toolchain. It should build with WinAVR too!

** Programming procedure **

The board can be flash by using the Makefile and the DFU Programmer tools.
See the make file for more details.
