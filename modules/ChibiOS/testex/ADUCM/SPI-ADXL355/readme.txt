*****************************************************************************
** ChibiOS/HAL + ChibiOS/EX - SPI + ADXL355 demo for ADUCM360.             **
*****************************************************************************

** TARGET **

The demo runs on an ADI ADICUP360 board. It has been tested 
connecting an external the EVAL-ADXL355.

** The Demo **

The application demonstrates the use of the ADUCM360 SPI driver in order
to acquire data from ADXL355 using ChibiOS/EX. The data is printed on the
User USB port.

** Board Setup **

With reference to the ADI UG-1030 (EVAL-ADXL354/EVAL-ADXL355 User Guide) and
to the schamatic of the ADICUP360 the following connection
are need:
 ---------------------------------------------
|     EVAL-ADXL355     |      ADICUP360       |
|---------------------------------------------|
|                      |                      |
|         P1.1         |      ARD_IOREF       |
|         P1.3         |       ARD_3V3        |
|         P1.5         |       ARD_GND        |
|                      |                      |
|         P2.2         |         CS           |
|         P2.4         |        SCLK1         |
|         P2.5         |        MISO1         |
|         P2.6         |        MOSI1         |
 ---------------------------------------------
 
To redirect P0.6 and P0.7 to the User COM port  and to connect the SPI pin to 
the PWMH connector it is required to setup the switch matrix as:
 - S1 -> 1
 - S2 -> 0
 - S3 -> 1
 - S4 -> 0
 
** Build Procedure **

The demo has been tested using the free Codesourcery GCC-based toolchain
and YAGARTO.
Just modify the TRGT line in the makefile in order to use different GCC ports.

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.analog.com