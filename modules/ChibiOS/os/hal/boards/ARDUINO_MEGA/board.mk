# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/ARDUINO_MEGA/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/ARDUINO_MEGA

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
