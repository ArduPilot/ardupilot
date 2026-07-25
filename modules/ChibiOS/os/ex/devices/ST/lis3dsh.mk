# List of all the LIS3DSH device files.
LIS3DSHSRC := $(CHIBIOS)/os/ex/devices/ST/lis3dsh.c

# Required include directories
LIS3DSHINC := $(CHIBIOS)/os/ex/include \
             $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(LIS3DSHSRC)
ALLINC  += $(LIS3DSHINC)