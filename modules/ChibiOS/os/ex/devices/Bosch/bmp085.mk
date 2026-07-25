# List of all the BMP085 device files.
BMP085SRC := $(CHIBIOS)/os/ex/devices/Bosch/bmp085.c

# Required include directories
BMP085INC := $(CHIBIOS)/os/ex/include \
             $(CHIBIOS)/os/ex/devices/Bosch

# Shared variables
ALLCSRC += $(BMP085SRC)
ALLINC  += $(BMP085INC)