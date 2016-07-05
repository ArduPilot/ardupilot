#
# Makefile for the VRUBRAIN 5.2 APM configuration
#
include $(SKETCHBOOK)/mk/VRBRAIN/vrbrain_common.mk

#MODULES		+= drivers/mpu9250
MODULES		+= drivers/boards/vrubrain-v52
MODULES		+= drivers/pwm_input
#MODULES         += modules/uavcan
MODULES         += lib/mathlib
