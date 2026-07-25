# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_F302R8/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_F302R8

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
