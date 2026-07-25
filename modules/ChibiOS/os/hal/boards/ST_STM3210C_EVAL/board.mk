# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/ST_STM3210C_EVAL/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/ST_STM3210C_EVAL

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
