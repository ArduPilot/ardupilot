# List of all the LSM6DS0 device files.
LSM6DS0SRC := $(CHIBIOS)/os/ex/devices/ST/lsm6ds0.c

# Required include directories
LSM6DS0INC := $(CHIBIOS)/os/ex/include \
              $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(LSM6DS0SRC)
ALLINC  += $(LSM6DS0INC)