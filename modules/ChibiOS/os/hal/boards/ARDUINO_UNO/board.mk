# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/ARDUINO_UNO/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/ARDUINO_UNO

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
