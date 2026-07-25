# List of all the LPS25H device files.
LPS25HSRC := $(CHIBIOS)/os/ex/devices/ST/lps25h.c

# Required include directories
LPS25HINC := $(CHIBIOS)/os/ex/include \
             $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(LPS25HSRC)
ALLINC  += $(LPS25HINC)