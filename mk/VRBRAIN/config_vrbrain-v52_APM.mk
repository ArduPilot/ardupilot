#
# Makefile for the VRBRAIN 5.2 APM configuration
#
include $(SKETCHBOOK)/mk/VRBRAIN/vrbrain_common.mk

MODULES		+= drivers/mpu6000
#MODULES		+= drivers/mpu9250
MODULES		+= drivers/boards/vrbrain-v52
