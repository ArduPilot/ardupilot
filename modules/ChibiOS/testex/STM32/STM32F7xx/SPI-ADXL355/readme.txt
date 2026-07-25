*****************************************************************************
** ChibiOS/HAL + ChibiOS/EX - SPI + ADXL355 demo for STM32F7xx.            **
*****************************************************************************

** TARGET **

The demo runs on an STM32 Discovery-F7. It has been tested 
connecting an external the EVAL-ADXL355.

** The Demo **

The application demonstrates the use of the STM32F7xx SPI driver in order
to acquire data from ADXL355 using ChibiOS/EX.

** Board Setup **

With reference to the ADI UG-1030 (EVAL-ADXL354/EVAL-ADXL355 User Guide) and
to the ST UM1907 (STM32 Discovery-F746G User Manual) the following connection
are need:
 ---------------------------------------------
|     EVAL-ADXL355     |    STM32 Discovery   |
|---------------------------------------------|
|                      |                      |
|         P1.1         |      ARD_IOREF       |
|         P1.3         |       ARD_3V3        |
|         P1.5         |       ARD_GND        |
|                      |                      |
|         P2.2         |       ARD_D9         |
|         P2.4         |       ARD_D13        |
|         P2.5         |       ARD_D11        |
|         P2.6         |       ARD_D12        |
 ---------------------------------------------
** Build Procedure **

The demo has been tested using the free Codesourcery GCC-based toolchain
and YAGARTO.
Just modify the TRGT line in the makefile in order to use different GCC ports.

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com