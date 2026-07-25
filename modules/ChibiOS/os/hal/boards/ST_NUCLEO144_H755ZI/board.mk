# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO144_H755ZI/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO144_H755ZI

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
