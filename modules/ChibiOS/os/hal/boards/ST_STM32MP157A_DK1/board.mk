# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_STM32MP157A_DK1/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_STM32MP157A_DK1

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
