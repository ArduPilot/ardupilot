# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/PJRC_TEENSY_2PLUSPLUS/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/PJRC_TEENSY_2PLUSPLUS

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
