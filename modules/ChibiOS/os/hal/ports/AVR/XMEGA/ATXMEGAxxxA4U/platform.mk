# List of all the AVR platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/AVR/XMEGA/ATXMEGAxxxA4U/hal_lld.c \

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/AVR/XMEGA/ATXMEGAxxxA4U

# Drivers compatible with the platform.
include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/CRYPv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DACv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DMAv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/GPIOv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/SPIv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/SYSTICKv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/USARTv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/WDGv1/driver.mk

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
