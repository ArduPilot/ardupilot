# List of all the LSM6DSL device files.
LSM6DSLSRC := $(CHIBIOS)/os/ex/devices/ST/lsm6dsl.c

# Required include directories
LSM6DSLINC := $(CHIBIOS)/os/ex/include \
              $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(LSM6DSLSRC)
ALLINC  += $(LSM6DSLINC)