# List of all the LSM303DLHC device files.
LSM303DLHCSRC := $(CHIBIOS)/os/ex/devices/ST/lsm303dlhc.c

# Required include directories
LSM303DLHCINC := $(CHIBIOS)/os/ex/include \
                 $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(LSM303DLHCSRC)
ALLINC  += $(LSM303DLHCINC)