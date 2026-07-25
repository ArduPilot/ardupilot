# Required platform files.
PLATFORMSRC := $(CHIBIOS)/os/hal/ports/common/ARMCMx/nvic.c \
               $(CHIBIOS)/os/hal/ports/MAX32/MAX32625/hal_lld.c \
               $(CHIBIOS)/os/hal/ports/MAX32/MAX32625/hal_pal_lld.c \
               $(CHIBIOS)/os/hal/ports/MAX32/MAX32625/hal_st_lld.c \
               $(CHIBIOS)/os/hal/ports/MAX32/MAX32625/max32_isr.c

# Required include directories.
PLATFORMINC := $(CHIBIOS)/os/hal/ports/common/ARMCMx \
               $(CHIBIOS)/os/hal/ports/MAX32/MAX32625

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
#include $(CHIBIOS)/os/hal/ports/ADUCM/LLD/UARTv1/driver.mk
#include $(CHIBIOS)/os/hal/ports/ADUCM/LLD/WUTv1/driver.mk

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
