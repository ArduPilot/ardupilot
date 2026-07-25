# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/OLIMEX_LPC_P1343/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/OLIMEX_LPC_P1343

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
