*****************************************************************************
** ChibiOS/HAL - CAN driver demo for STM32G4xx.                            **
*****************************************************************************

** TARGET **

The demo runs on an ST STM32G474-Nucleo board.

** The Demo **

The application demonstrates the use of the STM32G4xx CAN driver.


** Board Setup **

External LED PA8 - On board CN9 - D7

Connect PA11 - CN10-14 (CAN1-RX) to pin RX of first interface protocol controller MCP2551.
Connect PA12 - CN10-12 (CAN1-TX) to pin TX of first interface protocol controller MCP2551.

Connect PB5 - CN10-29 (CAN2-RX) to pin RX of second interface protocol controller MCP2551.
Connect PB6 - CN10-17 (CAN2-TX) to pin TX of second interface protocol controller MCP2551.

Connect CAN-H and CAN-L of the two MCP2551 to create bus CAN.

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
