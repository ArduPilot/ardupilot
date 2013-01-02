# find the mk/ directory, which is where this makefile fragment
# lives
MK_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

ifeq ($(APPDIR),)

####################
# AVR and SITL build


include $(MK_DIR)/Arduino.mk
include $(MK_DIR)/targets.mk

else

####################
# PX4 build

include $(MK_DIR)/px4_core.mk

endif

# these targets need to be outside the APPDIR if above
include $(MK_DIR)/px4_targets.mk
