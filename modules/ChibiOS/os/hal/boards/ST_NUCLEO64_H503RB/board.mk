# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_H503RB/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_H503RB

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
