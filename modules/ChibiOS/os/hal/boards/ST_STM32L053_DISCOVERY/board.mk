# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_STM32L053_DISCOVERY/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_STM32L053_DISCOVERY

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
