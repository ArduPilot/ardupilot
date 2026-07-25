# List of all the MFS subsystem files.
BSIOSRC := $(CHIBIOS)/os/hal/lib/complex/buffered_sio/hal_buffered_sio.c

# Required include directories
BSIOINC := $(CHIBIOS)/os/hal/lib/complex/buffered_sio

# Shared variables
ALLCSRC += $(BSIOSRC)
ALLINC  += $(BSIOINC)
