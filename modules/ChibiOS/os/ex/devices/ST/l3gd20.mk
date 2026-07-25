# List of all the L3GD20 device files.
L3GD20SRC := $(CHIBIOS)/os/ex/devices/ST/l3gd20.c

# Required include directories
L3GD20INC := $(CHIBIOS)/os/ex/include \
             $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(L3GD20SRC)
ALLINC  += $(L3GD20INC)