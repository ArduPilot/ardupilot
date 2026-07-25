*****************************************************************************
** ChibiOS/RT and NIL PWM demos for Atmel AVR ATmega1280.                  **
*****************************************************************************

** TARGET **

The demo runs on an Arduino Mega board.

** The Demo **

This demo creates three PWM channels on pins PB5, PB6 and PB7. Each channel uses
a different duty cycle with PB7 having a duty cycle of 50%, PB6 a duty cycle of
25% and PB5 a duty cycle of 75%. Since the LED is connected to PB7 on the Arduino
Mega, it can be seen flashing in high speed.

** Build Procedure **

The demo was built using the GCC AVR toolchain.
To build linking with ChibiOS:

$ make -f Makefile.ch

To build linking with NIL:

$ make -f Makefile.nil

** Notes **

This demo runs natively so the Arduino bootloader must be removed and the FUSEs
reprogrammed. The values used for fuses are LFUSE=0xe7 and HFUSE=0x99.
