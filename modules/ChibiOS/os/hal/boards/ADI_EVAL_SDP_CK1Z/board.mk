# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ADI_EVAL_SDP_CK1Z/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ADI_EVAL_SDP_CK1Z

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
