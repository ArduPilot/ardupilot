# List of all the LIS3MDL device files.
LIS3MDLSRC := $(CHIBIOS)/os/ex/devices/ST/lis3mdl.c

# Required include directories
LIS3MDLINC := $(CHIBIOS)/os/ex/include \
              $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(LIS3MDLSRC)
ALLINC  += $(LIS3MDLINC)