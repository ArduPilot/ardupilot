# List of all the ChibiOS/RT kernel files, there is no need to remove the files
# from this list, you can disable parts of the kernel by editing chconf.h.
ifeq ($(USE_SMART_BUILD),yes)

# Configuration files directory
ifeq ($(CHCONFDIR),)
  ifeq ($(CONFDIR),)
    CHCONFDIR = .
  else
    CHCONFDIR := $(CONFDIR)
  endif
endif

CHCONF := $(strip $(shell cat $(CHCONFDIR)/chconf.h | egrep -e "\#define"))

KERNSRC := $(CHIBIOS)/os/rt/src/chsys.c \
           $(CHIBIOS)/os/rt/src/chrfcu.c \
           $(CHIBIOS)/os/rt/src/chdebug.c \
           $(CHIBIOS)/os/rt/src/chtrace.c \
           $(CHIBIOS)/os/rt/src/chvt.c \
           $(CHIBIOS)/os/rt/src/chschd.c \
           $(CHIBIOS)/os/rt/src/chinstances.c \
           $(CHIBIOS)/os/rt/src/chthreads.c
ifneq ($(findstring CH_CFG_USE_TM TRUE,$(CHCONF)),)
KERNSRC += $(CHIBIOS)/os/rt/src/chtm.c
endif
ifneq ($(findstring CH_DBG_STATISTICS TRUE,$(CHCONF)),)
KERNSRC += $(CHIBIOS)/os/rt/src/chstats.c
endif
ifneq ($(findstring CH_CFG_USE_REGISTRY TRUE,$(CHCONF)),)
KERNSRC += $(CHIBIOS)/os/rt/src/chregistry.c
endif
ifneq ($(findstring CH_CFG_USE_SEMAPHORES TRUE,$(CHCONF)),)
KERNSRC += $(CHIBIOS)/os/rt/src/chsem.c
endif
ifneq ($(findstring CH_CFG_USE_MUTEXES TRUE,$(CHCONF)),)
KERNSRC += $(CHIBIOS)/os/rt/src/chmtx.c
endif
ifneq ($(findstring CH_CFG_USE_CONDVARS TRUE,$(CHCONF)),)
KERNSRC += $(CHIBIOS)/os/rt/src/chcond.c
endif
ifneq ($(findstring CH_CFG_USE_EVENTS TRUE,$(CHCONF)),)
KERNSRC += $(CHIBIOS)/os/rt/src/chevents.c
endif
ifneq ($(findstring CH_CFG_USE_MESSAGES TRUE,$(CHCONF)),)
KERNSRC += $(CHIBIOS)/os/rt/src/chmsg.c
endif
ifneq ($(findstring CH_CFG_USE_DYNAMIC TRUE,$(CHCONF)),)
KERNSRC += $(CHIBIOS)/os/rt/src/chdynamic.c
endif
else
KERNSRC := $(CHIBIOS)/os/rt/src/chsys.c \
           $(CHIBIOS)/os/rt/src/chrfcu.c \
           $(CHIBIOS)/os/rt/src/chdebug.c \
           $(CHIBIOS)/os/rt/src/chtrace.c \
           $(CHIBIOS)/os/rt/src/chvt.c \
           $(CHIBIOS)/os/rt/src/chschd.c \
           $(CHIBIOS)/os/rt/src/chinstances.c \
           $(CHIBIOS)/os/rt/src/chthreads.c \
           $(CHIBIOS)/os/rt/src/chtm.c \
           $(CHIBIOS)/os/rt/src/chstats.c \
           $(CHIBIOS)/os/rt/src/chregistry.c \
           $(CHIBIOS)/os/rt/src/chsem.c \
           $(CHIBIOS)/os/rt/src/chmtx.c \
           $(CHIBIOS)/os/rt/src/chcond.c \
           $(CHIBIOS)/os/rt/src/chevents.c \
           $(CHIBIOS)/os/rt/src/chmsg.c \
           $(CHIBIOS)/os/rt/src/chdynamic.c
endif

# Required include directories
KERNINC := $(CHIBIOS)/os/rt/include

# Shared variables
ALLCSRC += $(KERNSRC)
ALLINC  += $(KERNINC)

# OS Library
include $(CHIBIOS)/os/oslib/oslib.mk
