# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/SB_GENERIC/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/SB_GENERIC

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
