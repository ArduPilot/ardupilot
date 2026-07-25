# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/ST_INEMO_M1_DISCOVERY/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/ST_INEMO_M1_DISCOVERY

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
