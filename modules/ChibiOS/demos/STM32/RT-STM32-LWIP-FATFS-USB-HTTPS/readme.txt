*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M7 STM32F746.                            **
*****************************************************************************

** TARGET **

The demo runs on a STM32F476-Discovery board.

** The Demo **

The demo flashes a LED to indicate that is running properly.
FatFs is integrated using SDIO.

The USB-FS port is used as USB-CDC and a command shell is ready to accepts
commands there.

An example HTTPS server is implemented to serve "GET /" requests at address 
192.168.1.10 on port 443.

SSL certificate and server key that are compiled in are the example keys 
taken from the wolfSSL repository. To use different keys, regenerate cert.c 
using "xxd -i" from your certificate and keys.


** Build Procedure **

This build has been tested using arm-none-eabi-gcc and make.
Just type 'make' from this directory to create the image.


** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com

WolfSSL is Copyright (c) by WolfSSL Inc.
