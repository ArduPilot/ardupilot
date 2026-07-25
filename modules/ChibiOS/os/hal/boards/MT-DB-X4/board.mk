# List of all the board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/MT-DB-X4/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/MT-DB-X4

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
