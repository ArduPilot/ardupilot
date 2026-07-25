# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/DIGISPARK_ATTINY_167/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/DIGISPARK_ATTINY_167

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
