# List of all the HTS221 device files.
HTS221SRC := $(CHIBIOS)/os/ex/devices/ST/hts221.c

# Required include directories
HTS221INC := $(CHIBIOS)/os/ex/include \
             $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(HTS221SRC)
ALLINC  += $(HTS221INC)