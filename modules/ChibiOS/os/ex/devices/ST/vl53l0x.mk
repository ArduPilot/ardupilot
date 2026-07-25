# List of all the VL53L0X device files.
VL53L0XSRC := $(CHIBIOS)/os/ex/devices/ST/vl53l0x.c

# Required include directories
VL53L0XINC := $(CHIBIOS)/os/ex/include \
              $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(VL53L0XSRC)
ALLINC  += $(VL53L0XINC)