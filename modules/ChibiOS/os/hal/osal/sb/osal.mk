# OSAL files.
OSALSRC += ${CHIBIOS}/os/hal/osal/sb/osal.c

# Required include directories
OSALINC += ${CHIBIOS}/os/hal/osal/sb

# Shared variables
ALLCSRC += $(OSALSRC)
ALLINC  += $(OSALINC)
