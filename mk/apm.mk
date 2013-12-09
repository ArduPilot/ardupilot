# find the mk/ directory, which is where this makefile fragment
# lives. (patsubst strips the trailing slash.)
SYSTYPE			:=	$(shell uname)

ifneq ($(findstring CYGWIN, $(SYSTYPE)),) 
  MK_DIR := $(shell cygpath -m ../mk)
else
  MK_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
endif

include $(MK_DIR)/environ.mk

# short-circuit build for the configure target
ifeq ($(MAKECMDGOALS),configure)
include $(MK_DIR)/configure.mk

else

# common makefile components
include $(MK_DIR)/targets.mk
include $(MK_DIR)/sketch_sources.mk

ifneq ($(MAKECMDGOALS),clean)
# board specific includes
ifeq ($(HAL_BOARD),HAL_BOARD_APM1)
include $(MK_DIR)/board_avr.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_APM2)
include $(MK_DIR)/board_avr.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_AVR_SITL)
include $(MK_DIR)/board_avr_sitl.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_LINUX)
include $(MK_DIR)/board_linux.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_PX4)
include $(MK_DIR)/board_px4.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_FLYMAPLE)
include $(MK_DIR)/board_flymaple.mk
endif

endif

endif
