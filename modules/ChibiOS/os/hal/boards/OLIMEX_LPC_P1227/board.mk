# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/OLIMEX_LPC-P1227/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/OLIMEX_LPC-P1227

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
