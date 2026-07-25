# List of all the ADX355 device files.
ADXL355SRC := $(CHIBIOS)/os/ex/devices/ADI/adxl355.c

# Required include directories
ADXL355INC := $(CHIBIOS)/os/ex/include \
              $(CHIBIOS)/os/ex/devices/ADI

# Shared variables
ALLCSRC += $(ADXL355SRC)
ALLINC  += $(ADXL355INC)