# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_STM32G474RE_DISCOVERY_DPOW1/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_STM32G474RE_DISCOVERY_DPOW1

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
