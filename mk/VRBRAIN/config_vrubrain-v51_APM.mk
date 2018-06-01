#
# Makefile for the VRUBRAIN 5.1 APM configuration
#
include $(SKETCHBOOK)/mk/VRBRAIN/vrbrain_common.mk

MODULES		+= drivers/boards/vrubrain-v51
MODULES		+= drivers/pwm_input

