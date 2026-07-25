# List of all the Micron N25Q device files.
SNORSRC := $(CHIBIOS)/os/hal/lib/complex/serial_nor/hal_serial_nor.c \
           $(CHIBIOS)/os/hal/lib/complex/serial_nor/devices/micron_n25q/hal_flash_device.c

# Required include directories
SNORINC := $(CHIBIOS)/os/hal/lib/complex/serial_nor \
           $(CHIBIOS)/os/hal/lib/complex/serial_nor/devices/micron_n25q

# Shared variables
ALLCSRC += $(SNORSRC)
ALLINC  += $(SNORINC)
