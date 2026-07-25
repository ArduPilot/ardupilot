# XSNOR subsystem build.

# Dependencies.
include $(CHIBIOS)/os/common/oop/oop.mk

# Required files.
ifndef XSNORSRC
  XSNORSRC := $(CHIBIOS)/os/hal/lib/complex/xsnor/src/hal_xsnor_base.c
  ALLCSRC += $(XSNORSRC)
endif

# Required include directories
ifndef XSNORINC
  XSNORINC := $(CHIBIOS)/os/hal/lib/complex/xsnor/include
  ALLINC += $(XSNORINC)
endif
