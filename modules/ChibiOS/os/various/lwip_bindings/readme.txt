This directory contains the ChibiOS "official" bindings with the lwIP
TCP/IP stack: http://savannah.nongnu.org/projects/lwip

In order to use lwIP within ChibiOS/RT project, unzip lwIP under
./ext/lwip then include $(CHIBIOS)/os/various/lwip_bindings/lwip.mk
in your makefile.
