#
# Makefile for the px4fmu-v1_APM configuration
#
include $(SKETCHBOOK)/mk/PX4/px4_common.mk

MODULES		+= drivers/boards/px4fmu-v1
MODULES		+= drivers/px4io
MODULES		+= drivers/px4flow
