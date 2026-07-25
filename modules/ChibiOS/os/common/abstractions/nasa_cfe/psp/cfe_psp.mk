# NASA CFE PSP files.
CFEPSPSRC = $(CHIBIOS)/os/common/abstractions/nasa_cfe/psp/src/cfe_psp_support.c \
            $(CHIBIOS)/os/common/abstractions/nasa_cfe/psp/src/cfe_psp_timer.c \
            $(CHIBIOS)/os/common/abstractions/nasa_cfe/psp/src/cfe_psp_memory.c \
            $(CHIBIOS)/os/common/abstractions/nasa_cfe/psp/src/cfe_psp_exception.c

CFEPSPINC = $(CHIBIOS)/os/common/abstractions/nasa_cfe/psp/include

# Shared variables
ALLCSRC += $(CFEPSPSRC)
ALLINC  += $(CFEPSPINC)
