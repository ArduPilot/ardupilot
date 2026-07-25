# Required platform files.
PLATFORMSRC := $(CHIBIOS)/os/hal/ports/sandbox/hal_lld.c \
               $(CHIBIOS)/os/hal/ports/sandbox/hal_st_lld.c \
               $(CHIBIOS)/os/hal/ports/sandbox/hal_pal_lld.c \
               $(CHIBIOS)/os/hal/ports/sandbox/hal_sio_lld.c

# Required include directories.
PLATFORMINC := $(CHIBIOS)/os/hal/ports/sandbox

# Optional platform files.
ifeq ($(USE_SMART_BUILD),yes)

# Configuration files directory
ifeq ($(HALCONFDIR),)
  ifeq ($(CONFDIR),)
    HALCONFDIR = .
  else
    HALCONFDIR := $(CONFDIR)
  endif
endif

HALCONF := $(strip $(shell cat $(HALCONFDIR)/halconf.h | egrep -e "\#define"))

else
endif

# Drivers compatible with the platform.

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
