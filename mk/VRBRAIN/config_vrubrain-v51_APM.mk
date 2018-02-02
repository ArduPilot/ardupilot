#
# Makefile for the VRUBRAIN 5.1 APM configuration
#
include $(SKETCHBOOK)/mk/VRBRAIN/vrbrain_common.mk

MODULES		+= drivers/mpu6000
#MODULES		+= drivers/mpu9250
MODULES		+= drivers/boards/vrubrain-v51
