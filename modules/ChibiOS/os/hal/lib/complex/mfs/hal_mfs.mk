# List of all the MFS subsystem files.
MFSSRC := $(CHIBIOS)/os/hal/lib/complex/mfs/hal_mfs.c

# Required include directories
MFSINC := $(CHIBIOS)/os/hal/lib/complex/mfs

# Shared variables
ALLCSRC += $(MFSSRC)
ALLINC  += $(MFSINC)
