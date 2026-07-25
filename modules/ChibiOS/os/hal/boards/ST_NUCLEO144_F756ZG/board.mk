# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO144_F756ZG/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO144_F756ZG

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
