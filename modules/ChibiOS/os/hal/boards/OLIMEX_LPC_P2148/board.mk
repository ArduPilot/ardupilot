# List of all the mandatory board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/OLIMEX_LPC_P2148/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/OLIMEX_LPC_P2148

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
