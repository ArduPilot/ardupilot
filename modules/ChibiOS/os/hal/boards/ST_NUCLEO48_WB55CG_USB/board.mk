# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO48_WB55CG_USB/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO48_WB55CG_USB

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
