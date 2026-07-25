*****************************************************************************
** ChibiOS/NIL port for Atmel AVR ATmega328p.                              **
*****************************************************************************

** TARGET **

The demo runs on an Arduino Nano board.

** The Demo **

One thread prints a message to the serial console, which is available on
the board USB connector (FT232 converter).

Another thread toggles the LED on PB5 (pin 13 on Arduino IDE) every 500ms.

** Build Procedure **

The demo was built using the GCC AVR toolchain.
