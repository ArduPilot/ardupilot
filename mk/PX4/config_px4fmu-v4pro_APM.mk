#
# Makefile for the px4fmu-v2_APM configuration
#
include $(SKETCHBOOK)/mk/PX4/px4_common.mk

MODULES		+= drivers/boards/px4fmu-v4pro
MODULES		+= drivers/pwm_input
MODULES		+= drivers/px4io

