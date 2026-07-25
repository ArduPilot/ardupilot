# List of all the ChibiOS/LIB files.
ifeq ($(USE_SMART_BUILD),yes)

# Configuration files directory
ifeq ($(CHCONFDIR),)
  ifeq ($(CONFDIR),)
    CHCONFDIR = .
  else
    CHCONFDIR := $(CONFDIR)
  endif
endif

CHLIBCONF := $(strip $(shell cat $(CHCONFDIR)/chconf.h | egrep -e "\#define"))

ifneq ($(findstring CH_CFG_USE_MAILBOXES TRUE,$(CHLIBCONF)),)
OSLIBSRC += $(CHIBIOS)/os/oslib/src/chmboxes.c
endif
ifneq ($(findstring CH_CFG_USE_MEMCORE TRUE,$(CHLIBCONF)),)
OSLIBSRC += $(CHIBIOS)/os/oslib/src/chmemcore.c
endif
ifneq ($(findstring CH_CFG_USE_HEAP TRUE,$(CHLIBCONF)),)
OSLIBSRC += $(CHIBIOS)/os/oslib/src/chmemheaps.c
endif
ifneq ($(findstring CH_CFG_USE_MEMPOOLS TRUE,$(CHLIBCONF)),)
OSLIBSRC += $(CHIBIOS)/os/oslib/src/chmempools.c
endif
ifneq ($(findstring CH_CFG_USE_PIPES TRUE,$(CHLIBCONF)),)
OSLIBSRC += $(CHIBIOS)/os/oslib/src/chpipes.c
endif
ifneq ($(findstring CH_CFG_USE_OBJ_CACHES TRUE,$(CHLIBCONF)),)
OSLIBSRC += $(CHIBIOS)/os/oslib/src/chobjcaches.c
endif
ifneq ($(findstring CH_CFG_USE_DELEGATES TRUE,$(CHLIBCONF)),)
OSLIBSRC += $(CHIBIOS)/os/oslib/src/chdelegates.c
endif
ifneq ($(findstring CH_CFG_USE_FACTORY TRUE,$(CHLIBCONF)),)
OSLIBSRC += $(CHIBIOS)/os/oslib/src/chfactory.c
endif
else
OSLIBSRC := $(CHIBIOS)/os/oslib/src/chmboxes.c \
            $(CHIBIOS)/os/oslib/src/chmemcore.c \
            $(CHIBIOS)/os/oslib/src/chmemheaps.c \
            $(CHIBIOS)/os/oslib/src/chmempools.c \
            $(CHIBIOS)/os/oslib/src/chpipes.c \
            $(CHIBIOS)/os/oslib/src/chobjcaches.c \
            $(CHIBIOS)/os/oslib/src/chdelegates.c \
            $(CHIBIOS)/os/oslib/src/chfactory.c
endif

# Required include directories
OSLIBINC := $(CHIBIOS)/os/oslib/include

# Shared variables
ALLCSRC += $(OSLIBSRC)
ALLINC  += $(OSLIBINC)
