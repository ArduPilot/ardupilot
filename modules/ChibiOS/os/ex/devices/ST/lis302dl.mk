# List of all the LIS302DL device files.
LIS302DLSRC := $(CHIBIOS)/os/ex/devices/ST/lis302dl.c

# Required include directories
LIS302DLINC := $(CHIBIOS)/os/ex/include \
             $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(LIS302DLSRC)
ALLINC  += $(LIS302DLINC)