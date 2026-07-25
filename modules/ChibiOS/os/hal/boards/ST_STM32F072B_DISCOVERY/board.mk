# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_STM32F072B_DISCOVERY/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_STM32F072B_DISCOVERY

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
