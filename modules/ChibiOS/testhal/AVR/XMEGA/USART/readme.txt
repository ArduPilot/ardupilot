*****************************************************************************
** ChibiOS/RT port for ATxmega128a4u.                                      **
*****************************************************************************

** TARGET **

The demo runs on the mattair Tech MT-DB-X4 board.

** The Demo **

The demo show how to configure and use the USARTC0 to transmit an Hello world!
message. To see this message in a console on a computer, you must connect an
FTDI adapter to PIN PC3(TX) and PC2(RX).

** Build Procedure **

The demo was built using the GCC AVR toolchain. It should build with WinAVR too!

** Programming procedure **

The board can be flash by using the Makefile and the DFU Programmer tools.
See the make file for more details.
