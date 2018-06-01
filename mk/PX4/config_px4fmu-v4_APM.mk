#
# Makefile for the px4fmu-v4_APM configuration
#
include $(SKETCHBOOK)/mk/PX4/px4_common.mk

MODULES		+= drivers/boards/px4fmu-v4
MODULES		+= drivers/pwm_input
