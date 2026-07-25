# List of all the Micron N25Q device files.
SERNORSRC := $(CHIBIOS)/os/hal/lib/complex/serial_nor/hal_serial_nor.c \
             $(CHIBIOS)/os/hal/lib/complex/serial_nor/devices/macronix_mx25/hal_flash_device.c

# Required include directories
SERNORINC := $(CHIBIOS)/os/hal/lib/complex/serial_nor \
             $(CHIBIOS)/os/hal/lib/complex/serial_nor/devices/macronix_mx25

# Shared variables
ALLCSRC += $(SERNORSRC)
ALLINC  += $(SERNORINC)
