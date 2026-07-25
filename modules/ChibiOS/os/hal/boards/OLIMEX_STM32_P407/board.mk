# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/OLIMEX_STM32_P407/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/OLIMEX_STM32_P407

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
