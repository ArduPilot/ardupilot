*****************************************************************************
** ChibiOS/RT ADC test on Atmel AVR ATmega2560.                            **
*****************************************************************************

** TARGET **

The test demo runs on an Arduino Mega board.

** The Demo **

This test blinks the onboard LED and also prints the result of the ADC
conversion to the Serial0. A voltmeter can be use to compare the printed
value to the value on pin A0 of the arduino board. To make this test, a
potentiometer is connected to the pin A0 of the board.

** Build Procedure **

The demo was built using the GCC AVR toolchain.
