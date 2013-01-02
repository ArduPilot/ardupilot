
ifeq ($(APPDIR),)
####################
# AVR and SITL build
include ../mk/Arduino.mk
include ../mk/targets.mk

sitl-newcontrollers: EXTRAFLAGS += "-DAPM_CONTROL=ENABLED "
sitl-newcontrollers: sitl

else

####################
# PX4 build
APPNAME		 = ArduPlane
PRIORITY	 = SCHED_PRIORITY_DEFAULT
STACKSIZE	 = 4096

SKETCHBOOK=..
include $(APPDIR)/mk/apm.mk

endif
