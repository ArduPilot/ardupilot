# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_U385RG_Q/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_U385RG_Q

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
