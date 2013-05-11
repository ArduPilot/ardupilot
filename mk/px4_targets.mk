# PX4 build is via external build system

ifneq ($(PX4_ROOT),)

PX4_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu_APM.mk
SKETCHFLAGS=$(SKETCHLIBINCLUDES) -I$(PWD) -DCONFIG_HAL_BOARD=HAL_BOARD_PX4 -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=ArduPilot_main
PX4_MAKE = make -C $(BUILDROOT) -f $(PX4_ROOT)/makefiles/firmware.mk CONFIG_FILE=$(PWD)/$(PX4_CONFIG_FILE) EXTRADEFINES="$(SKETCHFLAGS) "$(EXTRAFLAGS) APM_MODULE_DIR=$(BUILDROOT) SKETCHBOOK=$(SKETCHBOOK) PX4_ROOT=$(PX4_ROOT)

$(BUILDROOT)/module.mk:
	$(RULEHDR)
	$(v) echo "# Auto-generated file - do not edit" > $@
	$(v) echo "MODULE_COMMAND = ArduPilot" >> $@
	$(v) echo "SRCS = $(SKETCH).cpp $(SKETCHLIBSRCS)" >> $@
	$(v) echo "MODULE_STACKSIZE = 4096" >> $@
	$(v) echo "MAXOPTIMIZATION  = -Os" >> $@

px4: $(PX4_ROOT)/Archives/px4fmu.export $(SKETCHCPP) $(BUILDROOT)/module.mk
	$(RULEHDR)
	$(v) $(PX4_MAKE) firmware
	$(v) /bin/rm -f $(SKETCH).px4
	$(v) cp $(PX4_ROOT)/makefiles/build/firmware.px4 $(SKETCH).px4
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH).px4"

px4-clean: clean px4-archives-clean
	$(v) /bin/rm -rf $(PX4_ROOT)/makefiles/build $(PX4_ROOT)/Build

px4-upload: px4
	$(RULEHDR)
	$(PX4_MAKE) upload

px4-archives-clean:
	$(v) /bin/rm -rf $(PX4_ROOT)/Archives

px4-io: $(PX4_ROOT)/Archives/px4io.export
	$(v) make -C $(PX4_ROOT) px4io_default
	$(v) /bin/rm -f px4io.bin
	$(v) cp $(PX4_ROOT)/Build/px4io_default.build/firmware.bin px4io.bin
	$(v) echo "PX4IO Firmware is in px4io.bin"

$(PX4_ROOT)/Archives/px4fmu.export:
	make -C $(PX4_ROOT) archives

$(PX4_ROOT)/Archives/px4io.export:
	make -C $(PX4_ROOT) archives

px4-archives: $(PX4_ROOT)/Archives/px4fmu.export

else

px4:
	$(error ERROR: You need to add PX4_ROOT to your config.mk)

px4-clean: px4

px4-upload: px4

endif
