# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_L452RE_P/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_L452RE_P

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
