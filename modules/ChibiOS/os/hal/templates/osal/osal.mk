# OSAL files.
OSALSRC += ${CHIBIOS}/os/hal/templates/osal/osal.c

# Required include directories
OSALINC += ${CHIBIOS}/os/hal/templates/osal

# Shared variables
ALLCSRC += $(OSALSRC)
ALLINC  += $(OSALINC)
