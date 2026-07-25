# NASA CFE OSAL files.
CFEOSALSRC = $(CHIBIOS)/os/common/abstractions/nasa_cfe/osal/src/osapi.c

CFEOSALINC = $(CHIBIOS)/os/common/abstractions/nasa_cfe/osal/include

# Shared variables
ALLCSRC += $(CFEOSALSRC)
ALLINC  += $(CFEOSALINC)
