# PX4 build is via external build system

ifneq ($(PX4_ROOT),)

# try to cope with relative paths
ifeq ($(wildcard $(PX4_ROOT)/NuttX),)
PX4_ROOT := $(shell cd $(SKETCHBOOK)/$(PX4_ROOT) && pwd)
endif
ifeq ($(wildcard $(PX4_ROOT)/NuttX),)
$(error ERROR: PX4_ROOT not set correctly - no NuttX directory found)
endif

# we have different config files for V1 and V2
PX4_V1_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu-v1_APM.mk
PX4_V2_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu-v2_APM.mk

SKETCHFLAGS=$(SKETCHLIBINCLUDES) -I$(PWD) -DARDUPILOT_BUILD -DCONFIG_HAL_BOARD=HAL_BOARD_PX4 -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=ArduPilot_main

PX4_MAKE = make -C $(BUILDROOT) -f $(PX4_ROOT)/makefiles/firmware.mk EXTRADEFINES="$(SKETCHFLAGS) "$(EXTRAFLAGS) APM_MODULE_DIR=$(BUILDROOT) SKETCHBOOK=$(SKETCHBOOK) PX4_ROOT=$(PX4_ROOT)

$(BUILDROOT)/module.mk:
	$(RULEHDR)
	$(v) echo "# Auto-generated file - do not edit" > $@
	$(v) echo "MODULE_COMMAND = ArduPilot" >> $@
	$(v) echo "SRCS = $(SKETCH).cpp $(SKETCHLIBSRCS)" >> $@
	$(v) echo "MODULE_STACKSIZE = 4096" >> $@

px4-v1: $(PX4_ROOT)/Archives/px4fmu-v1.export $(SKETCHCPP) $(BUILDROOT)/module.mk
	$(RULEHDR)
	$(v) $(PX4_MAKE) CONFIG_FILE=$(PWD)/$(PX4_V1_CONFIG_FILE) firmware
	$(v) /bin/rm -f $(SKETCH)-v1.px4
	$(v) cp $(PX4_ROOT)/makefiles/build/firmware.px4 $(SKETCH)-v1.px4
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH)-v1.px4"

px4-v2: $(PX4_ROOT)/Archives/px4fmu-v2.export $(SKETCHCPP) $(BUILDROOT)/module.mk
	$(RULEHDR)
	$(v) $(PX4_MAKE) CONFIG_FILE=$(PWD)/$(PX4_V2_CONFIG_FILE) firmware
	$(v) /bin/rm -f $(SKETCH)-v2.px4
	$(v) cp $(PX4_ROOT)/makefiles/build/firmware.px4 $(SKETCH)-v2.px4
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH)-v2.px4"

px4: px4-v1 px4-v2

px4-clean: clean px4-archives-clean
	$(v) /bin/rm -rf $(PX4_ROOT)/makefiles/build $(PX4_ROOT)/Build

px4-upload-v1: px4
	$(RULEHDR)
	$(PX4_MAKE) CONFIG_FILE=$(PWD)/$(PX4_V1_CONFIG_FILE) upload

px4-upload-v2: px4
	$(RULEHDR)
	$(PX4_MAKE) CONFIG_FILE=$(PWD)/$(PX4_V2_CONFIG_FILE) upload

px4-upload: px4-upload-v1

px4-archives-clean:
	$(v) /bin/rm -rf $(PX4_ROOT)/Archives

px4-io-v1: $(PX4_ROOT)/Archives/px4io-v1.export
	$(v) make -C $(PX4_ROOT) px4io-v1_default
	$(v) /bin/rm -f px4io-v1.bin
	$(v) cp $(PX4_ROOT)/Build/px4io-v1_default.build/firmware.bin px4io-v1.bin
	$(v) echo "PX4IOv1 Firmware is in px4io-v1.bin"

px4-io-v2: $(PX4_ROOT)/Archives/px4io-v2.export
	$(v) make -C $(PX4_ROOT) px4io-v2_default
	$(v) /bin/rm -f px4io-v1.bin
	$(v) cp $(PX4_ROOT)/Build/px4io-v2_default.build/firmware.bin px4io-v2.bin
	$(v) echo "PX4IOv2 Firmware is in px4io-v2.bin"

px4-io: px4-io-v1 px4-io-v2

$(PX4_ROOT)/Archives/px4fmu-v1.export:
	make -C $(PX4_ROOT) archives

$(PX4_ROOT)/Archives/px4fmu-v2.export:
	make -C $(PX4_ROOT) archives

$(PX4_ROOT)/Archives/px4io-v1.export:
	make -C $(PX4_ROOT) archives

$(PX4_ROOT)/Archives/px4io-v2.export:
	make -C $(PX4_ROOT) archives

px4-archives: $(PX4_ROOT)/Archives/px4fmu-v1.export $(PX4_ROOT)/Archives/px4fmu-v2.export

else

px4:
	$(error ERROR: You need to add PX4_ROOT to your config.mk)

px4-clean: px4

px4-upload: px4

endif
