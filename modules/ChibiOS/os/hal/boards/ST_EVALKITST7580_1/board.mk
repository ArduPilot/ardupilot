# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_EVALKITST7580_1/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_EVALKITST7580_1

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
