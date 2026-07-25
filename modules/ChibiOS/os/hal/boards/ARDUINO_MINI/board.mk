# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/ARDUINO_MINI/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/ARDUINO_MINI

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
