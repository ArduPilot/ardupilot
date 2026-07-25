*****************************************************************************
** ChibiOS/RT port for Atmel AVR AT90USB128.                               **
*****************************************************************************

** TARGET **

The demo runs on an PJRC Teensy 2++ board.

** The Demo **

The demo creates a standard serial-over-USB (CDC) slave device.  The simple
ChibiOS shell is run on the USB serial connection and can be interacted
with by a standard serial port program (e.g. putty or screen). 

** Build Procedure **

The demo was built using the GCC AVR toolchain. It should build with WinAVR too!

** Notes **

The USB driver itself is not that heavyweight but this demo requires a bit of
flash space and memory because of the nature of the ChibiOS shell code and
supporting functionality.
