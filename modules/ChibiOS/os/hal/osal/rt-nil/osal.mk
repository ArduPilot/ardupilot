# OSAL files.
OSALSRC += ${CHIBIOS}/os/hal/osal/rt-nil/osal.c

# Required include directories
OSALINC += ${CHIBIOS}/os/hal/osal/rt-nil

# Shared variables
ALLCSRC += $(OSALSRC)
ALLINC  += $(OSALINC)
