# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/GOLDBULL_STM32_F107VC/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/GOLDBULL_STM32_F107VC

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
