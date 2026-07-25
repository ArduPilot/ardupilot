# List of all the AVR platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/AVR/TINY/ATTinyxxx/hal_lld.c

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/AVR/TINY/ATTinyxxx/

# Drivers compatible with the platform.
PLATFORMINC = ${CHIBIOS}/os/hal/ports/AVR/TINY/ATTinyxxx/
include ${CHIBIOS}/os/hal/ports/AVR/TINY/LLD/GPIOv1/driver.mk
include ${CHIBIOS}/os/hal/ports/AVR/TINY/LLD/SYSTICKv1/driver.mk
include ${CHIBIOS}/os/hal/ports/AVR/TINY/LLD/USARTv1/driver.mk

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
