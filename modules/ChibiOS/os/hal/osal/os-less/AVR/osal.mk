# OSAL files.
OSALSRC += ${CHIBIOS}/os/hal/osal/os-less/AVR/osal.c

# Required include directories
OSALINC += ${CHIBIOS}/os/hal/osal/os-less/AVR

# Shared variables
ALLCSRC += $(OSALSRC)
ALLINC  += $(OSALINC)
