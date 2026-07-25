# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/SEEED_ARCH_MAX/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/SEEED_ARCH_MAX

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
