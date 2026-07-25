# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/OLIMEX_STM32_E407/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/OLIMEX_STM32_E407

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
