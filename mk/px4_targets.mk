# PX4 build is via external build system

ifneq ($(PX4_ROOT),)

PX4_MAKE = make -C $(BUILDROOT) -f $(PX4_ROOT)/makefiles/firmware.mk CONFIG_FILE=$(PWD)/$(PX4_CONFIG_FILE) EXTRAFLAGS="$(EXTRAFLAGS)" APM_MODULE_DIR=$(BUILDROOT) SKETCHBOOK=$(SKETCHBOOK) PX4_ROOT=$(PX4_ROOT)


PX4_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu_APM.mk
PX4SRCS = $(SKETCHCPP) $(SKETCHLIBSRCS)
EXTRAFLAGS=$(SKETCHLIBINCLUDES) -I$(PWD) -DCONFIG_HAL_BOARD=HAL_BOARD_PX4 -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=ArduPilot_main

$(BUILDROOT)/module.mk:
	echo "# Auto-generated file - do not edit" > $@
	echo "MODULE_COMMAND = ArduPilot" >> $@
	echo "SRCS = $(SKETCH).cpp $(SKETCHLIBSRCS)" >> $@
	echo "MODULE_STACKSIZE = 4096" >> $@
	echo "MAXOPTIMIZATION  = -Os" >> $@

px4: $(SKETCHCPP) $(BUILDROOT)/module.mk
	$(PX4_MAKE) firmware

px4-clean: clean
	/bin/rm -rf $(PX4_ROOT)/makefiles/build

px4-upload: $(SKETCHCPP) $(BUILDROOT)/module.mk
	$(PX4_MAKE) firmware upload

px4-archives:
	$(PX4_MAKE) archives

else

px4:
	$(error ERROR: You need to add PX4_ROOT to your config.mk)

px4-clean: px4

px4-upload: px4

endif
