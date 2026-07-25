*****************************************************************************
** ChibiOS/RT port for Atmel AVR ATmega328p.                               **
*****************************************************************************

** TARGET **

The demo runs on an Arduino Mega board.

** The Demo **

The demo currently just prints the TestThread output on Serial0, which is
available on the board USB connector (FT232 converter), and toggles the LED
on PB7 (pin 13 on Arduino IDE) every second.

** Build Procedure **

The demo was built using the GCC AVR toolchain. It should build with WinAVR too!

** Notes **

This demo runs natively so the Arduino bootloader must be removed and the FUSEs
reprogrammed. The values used for fuses are LFUSE=0xe7 and HFUSE=0x99.
