# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/MAPLEMINI_STM32_F103/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/MAPLEMINI_STM32_F103

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
