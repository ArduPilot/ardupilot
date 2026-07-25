# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/OLIMEX_STM32_H103/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/OLIMEX_STM32_H103

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
