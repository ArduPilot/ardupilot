# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/NGX_BB_LPC11U14/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/NGX_BB_LPC11U14

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
