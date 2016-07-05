#
# Makefile for the VRBRAIN 5.1 APM configuration
#
include $(SKETCHBOOK)/mk/VRBRAIN/vrbrain_common.mk

#MODULES		+= drivers/mpu9250
MODULES		+= drivers/boards/vrbrain-v51
MODULES		+= drivers/pwm_input
#MODULES         += modules/uavcan
MODULES         += lib/mathlib
