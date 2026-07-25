# List of all the ADX317 device files.
ADXL317SRC := $(CHIBIOS)/os/ex/devices/ADI/adxl317.c

# Required include directories
ADXL317INC := $(CHIBIOS)/os/ex/include \
              $(CHIBIOS)/os/ex/devices/ADI

# Shared variables
ALLCSRC += $(ADXL317SRC)
ALLINC  += $(ADXL317INC)