# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/MIKROE_CLICKER2_STM32/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/MIKROE_CLICKER2_STM32

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
