# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO144_F413ZH/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO144_F413ZH

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
