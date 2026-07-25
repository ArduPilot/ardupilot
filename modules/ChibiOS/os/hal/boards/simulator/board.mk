# List of all the simulator board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/simulator/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/simulator

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
