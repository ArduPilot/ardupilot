*****************************************************************************
** ChibiOS/HAL - SPI driver demo for ARM-Cortex-M3 ADUCM360.               **
*****************************************************************************

** TARGET **

The demo runs on an ADICUP360 board.

** The Demo **

The application demonstrates the use of the HAL SPI driver.

** Board setup **

To redirect P0.1 and P0.2 to the SPI connector it is required to setup
the switch matrix as:
 - S1 -> 0
You should than connect P0.0 and P0.2 togheter (MISO1 and MOSI1 from the 
P7 connector): this would guarantee the loopback connection.
 
** Build Procedure **

The command "make" builds the demo for the target. The demo can be compiled
using the "GNU ARM Embedded Toolchain" from:

                   https://launchpad.net/gcc-arm-embedded

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
Analog Devices and are licensed under a different license.
Also note that not all the files present within the Analog Devices 
Cross Core Embedded Studio are distributed with ChibiOS/RT, 
you can find the whole library on the ADI web site:

                             http://www.analog.com