# Source files located under $(AUTOBUILD_ROOT) are automatically added.
ifeq ($(AUTOBUILD_ROOT),)
  AUTOBUILD_ROOT := ./source/
endif

rwildcard    = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
AUTOCSRC    := $(call rwildcard,$(AUTOBUILD_ROOT),*.c)
AUTOCPPSRC  := $(call rwildcard,$(AUTOBUILD_ROOT),*.cpp)
AUTOASMSRC  := $(call rwildcard,$(AUTOBUILD_ROOT),*.s)
AUTOXASMSRC := $(call rwildcard,$(AUTOBUILD_ROOT),*.S)
AUTOINC     := $(sort $(dir $(call rwildcard,$(AUTOBUILD_ROOT),*)))

# Shared variables.
ALLCSRC     += $(AUTOCSRC)
ALLCPPSRC   += $(AUTOCPPSRC)
ALLASMSRC   += $(AUTOASMSRC)
ALLXASMSRC  += $(AUTOXASMSRC)
ALLINC      += $(AUTOINC)
