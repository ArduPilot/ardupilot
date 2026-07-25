# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/OLIMEX_STM32_E407_REV_D/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/OLIMEX_STM32_E407_REV_D

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
