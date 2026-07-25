*****************************************************************************
** ChibiOS/RT port for Atmel AVR ATmega1280.                                **
*****************************************************************************

** TARGET **

The demo runs on an Arduino Mega board.

** The Demo **

This demo uses ICP3 (ICU3 input) which is located on pin PE7 to measure signal
width and period. The signal is output on pin PD4 using standard PAL calls and
thread sleep functions. PD4 must be wired to PE7. It outputs three different
waveforms with aproximately 1s duration using 50%, 25% and 75% respective duty
cycles. After that it just turns on ICU3 and waits for it to overflow. The values
read on each of these operations is output to SERIAL1.

** Build Procedure **

The demo was built using the GCC AVR toolchain. It should build with WinAVR too!

** Notes **

This demo runs natively so the Arduino bootloader must be removed and the FUSEs
reprogrammed. The values used for fuses are LFUSE=0xe7 and HFUSE=0x99.
