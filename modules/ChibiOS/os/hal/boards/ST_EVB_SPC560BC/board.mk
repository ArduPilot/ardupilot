# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/ST_EVB_SPC560BC/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/ST_EVB_SPC560BC

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
