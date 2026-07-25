# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/SPARKFUN_PRO_MICRO/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/SPARKFUN_PRO_MICRO

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
