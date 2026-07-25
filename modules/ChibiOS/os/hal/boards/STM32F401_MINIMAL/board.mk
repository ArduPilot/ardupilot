# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/STM32F401_MINIMAL/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/STM32F401_MINIMAL

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
