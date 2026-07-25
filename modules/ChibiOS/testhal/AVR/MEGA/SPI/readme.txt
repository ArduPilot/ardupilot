*****************************************************************************
** ChibiOS/RT SPI test on Atmel AVR ATmega328p.                            **
*****************************************************************************

** TARGET **

The test demo runs on an Arduino Uno board.

** The Demo **

This test writes to the SPI pins (MOSI/PD11 on the ATmega328p).
There's no feedback provided but the SPI can easily be analyzed with any
logic analyzer or sniffing with a buspirate or similar device.

** Build Procedure **

The demo was built using the GCC AVR toolchain.
