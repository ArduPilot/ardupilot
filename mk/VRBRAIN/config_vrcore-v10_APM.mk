#
# Makefile for the VRCORE 1.0 APM configuration
#
include $(SKETCHBOOK)/mk/VRBRAIN/vrbrain_common.mk

#MODULES		+= drivers/mpu6000
MODULES		+= drivers/mpu9250
MODULES		+= drivers/boards/vrcore-v10
