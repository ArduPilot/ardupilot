# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/OLIMEX_AVR_CAN/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/OLIMEX_AVR_CAN

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
