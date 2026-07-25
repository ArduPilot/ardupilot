# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/ST_NUCLEO64_F103RB/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/ST_NUCLEO64_F103RB

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
