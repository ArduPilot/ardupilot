# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO32_L011K4/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO32_L011K4

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
