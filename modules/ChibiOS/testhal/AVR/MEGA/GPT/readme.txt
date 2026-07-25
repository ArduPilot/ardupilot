*****************************************************************************
** ChibiOS/RT port for Atmel AVR ATmega2560.                               **
*****************************************************************************

** TARGET **

The demo runs on an Arduino Mega board.

** The Demo **

This demo creates a continuous timer which expires every 500ms, running a
callback function that toggles the LED on PB7. It also prints out the values
of the timer registers on the first serial port.

** Build Procedure **

The demo was built using the GCC AVR toolchain. It should build with WinAVR too!

** Notes **

This demo runs natively so the Arduino bootloader must be removed and the FUSEs
reprogrammed. The values used for fuses are LFUSE=0xe7 and HFUSE=0x99.
