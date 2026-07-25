*****************************************************************************
** ChibiOS/RT I2C test on Atmel AVR ATmega1280.                            **
*****************************************************************************

** TARGET **

The test demo runs on an Arduino Mega board.

** The Demo **

This test blinks the onboard LED and also writes/reads to a 24C64 connected
to the I2C pins (PD0/PD1 on the ATmega1280). There's no feedback provided but
the I2C can easily be analyzed with any logic analyzer or sniffing with a
buspirate or similar device.

** Build Procedure **

The demo was built using the GCC AVR toolchain.
