# List of all the LPS22HB device files.
LPS22HBSRC := $(CHIBIOS)/os/ex/devices/ST/lps22hb.c

# Required include directories
LPS22HBINC := $(CHIBIOS)/os/ex/include \
             $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(LPS22HBSRC)
ALLINC  += $(LPS22HBINC)