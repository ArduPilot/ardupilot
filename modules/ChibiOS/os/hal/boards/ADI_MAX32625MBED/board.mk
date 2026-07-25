# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ADI_MAX32625MBED/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ADI_MAX32625MBED

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
