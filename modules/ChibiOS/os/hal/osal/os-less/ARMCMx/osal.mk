# OSAL files.
OSALSRC += ${CHIBIOS}/os/hal/osal/os-less/ARMCMx/osal.c \
           ${CHIBIOS}/os/hal/osal/lib/osal_vt.c

# Required include directories
OSALINC += ${CHIBIOS}/os/hal/osal/os-less/ARMCMx \
           ${CHIBIOS}/os/hal/osal/lib

# Shared variables
ALLCSRC += $(OSALSRC)
ALLINC  += $(OSALINC)
