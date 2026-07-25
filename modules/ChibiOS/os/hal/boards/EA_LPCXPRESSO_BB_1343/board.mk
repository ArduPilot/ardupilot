# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/EA_LPCXPRESSO_BB_1343/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/EA_LPCXPRESSO_BB_1343

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
