# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ADICUP360/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ADICUP360

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
