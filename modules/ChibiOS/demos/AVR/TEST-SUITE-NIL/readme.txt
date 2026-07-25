*****************************************************************************
** ChibiOS/NIL port for Atmel AVR ATmega2560.                              **
*****************************************************************************

** TARGET **

The demo runs on an Arduino Mega board.

** The Demo **

One thread prints a message to the serial console, which is available on
the board USB connector (FT232 converter).

Another thread toggles the LED on PB7 (pin 13 on Arduino IDE) every 500ms.

** Build Procedure **

The demo was built using the GCC AVR toolchain. It should build with WinAVR too!

** Notes **

This demo runs natively so the Arduino bootloader must be removed and the FUSEs
reprogrammed. The values used for fuses are LFUSE=0xe7 and HFUSE=0x99.
