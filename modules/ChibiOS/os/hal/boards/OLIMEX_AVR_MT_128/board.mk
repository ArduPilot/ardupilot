# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/OLIMEX_AVR_MT_128/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/OLIMEX_AVR_MT_128

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
