# List of all the LSM303AGR device files.
LSM303AGRSRC := $(CHIBIOS)/os/ex/devices/ST/lsm303agr.c

# Required include directories
LSM303AGRINC := $(CHIBIOS)/os/ex/include \
                 $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(LSM303AGRSRC)
ALLINC  += $(LSM303AGRINC)