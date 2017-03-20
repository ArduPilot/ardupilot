#
# common makefile elements for all PX4 boards
#

#
# Use the configuration's ROMFS.
#
ROMFS_ROOT	 = $(SKETCHBOOK)/mk/VRBRAIN/ROMFS
MODULES		+= $(APM_MODULE_DIR)

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= drivers/stm32
MODULES		+= drivers/stm32/adc
MODULES		+= drivers/stm32/tone_alarm
MODULES		+= drivers/led
MODULES		+= drivers/px4fmu
MODULES		+= drivers/hmc5883
MODULES		+= drivers/ms5611
MODULES		+= drivers/mb12xx
MODULES		+= drivers/ll40ls
MODULES		+= drivers/trone
#MODULES	+= drivers/gps
#MODULES	+= drivers/hil
#MODULES	+= drivers/hott_telemetry
#MODULES	+= drivers/blinkm
#MODULES	+= modules/sensors
MODULES		+= drivers/mkblctrl
MODULES		+= drivers/batt_smbus
MODULES		+= drivers/pwm_input
MODULES		+= drivers/px4flow

#
# System commands
#
MODULES		+= systemcmds/bl_update
MODULES		+= systemcmds/mixer
MODULES		+= systemcmds/perf
MODULES		+= systemcmds/pwm
MODULES		+= systemcmds/reboot
MODULES		+= systemcmds/top
#MODULES	+= systemcmds/tests
MODULES		+= systemcmds/nshterm
MODULES		+= systemcmds/mtd
MODULES		+= systemcmds/ver

ifneq ($(wildcard $(PX4_ROOT)/src/systemcmds/reflect),)  
MODULES		+= systemcmds/reflect
endif
ifneq ($(wildcard $(PX4_ROOT)/src/systemcmds/motor_test),)  
MODULES		+= systemcmds/motor_test
endif
ifneq ($(wildcard $(PX4_ROOT)/src/systemcmds/usb_connected),)  
MODULES		+= systemcmds/usb_connected
endif

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
MODULES		+= modules/uORB
MODULES		+= lib/mathlib/math/filter
MODULES		+= lib/mathlib
#MODULES		+= modules/uavcan

# Note: auth disabled to keep us under 1MB flash because of STM32 bug
#ifneq ($(wildcard $(PX4_ROOT)/src/systemcmds/auth),)  
#MODULES		+= systemcmds/auth
#endif
#ifneq ($(wildcard $(PX4_ROOT)/src/modules/libtomfastmath),)  
#MODULES	        += modules/libtomfastmath
#MODULES         += modules/libtomcrypt
#endif

MODULES		+= lib/conversion

#
# Transitional support - add commands from the NuttX export archive.
#
# In general, these should move to modules over time.
#
# Each entry here is <command>.<priority>.<stacksize>.<entrypoint> but we use a helper macro
# to make the table a bit more readable.
#
define _B
	$(strip $1).$(or $(strip $2),SCHED_PRIORITY_DEFAULT).$(or $(strip $3),CONFIG_PTHREAD_STACK_DEFAULT).$(strip $4)
endef

#                  command                 priority                   stack  entrypoint
BUILTIN_COMMANDS := \
	$(call _B, sercon,                 ,                          2048,  sercon_main                ) \
	$(call _B, serdis,                 ,                          2048,  serdis_main                )
