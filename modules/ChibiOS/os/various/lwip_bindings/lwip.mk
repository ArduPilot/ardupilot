# List of the required lwIP files.
LWIPDIR = $(CHIBIOS)/ext/lwip/src

# The various blocks of files are outlined in Filelists.mk.
include $(LWIPDIR)/Filelists.mk

LWBINDSRC = \
        $(CHIBIOS)/os/various/lwip_bindings/lwipthread.c \
        $(CHIBIOS)/os/various/lwip_bindings/arch/sys_arch.c \
        $(CHIBIOS)/os/various/evtimer.c


# Add blocks of files from Filelists.mk as required for enabled options
LWSRC_REQUIRED = $(COREFILES) $(CORE6FILES) $(CORE4FILES) $(APIFILES) $(LWBINDSRC) $(NETIFFILES) $(LWIPAPPFILES)
LWSRC_EXTRAS ?= $(HTTPFILES)

LWINC = \
        $(CHIBIOS)/os/various/lwip_bindings \
        $(LWIPDIR)/include

# Shared variables
ALLCSRC += $(LWSRC_REQUIRED) $(LWSRC_EXTRAS)
ALLINC  += $(LWINC) \
           $(CHIBIOS)/os/various
