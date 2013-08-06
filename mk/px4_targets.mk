# PX4 build is via external build system

ifneq ($(PX4_ROOT),)

# try to cope with relative paths
ifeq ($(wildcard $(PX4_ROOT)/nuttx-configs),)
PX4_ROOT := $(shell cd $(SKETCHBOOK)/$(PX4_ROOT) && pwd)
endif
ifeq ($(wildcard $(PX4_ROOT)/nuttx-configs),)
$(error ERROR: PX4_ROOT not set correctly - no nuttx-configs directory found)
endif

# allow user to have NuttX git tree in directory above Firmware tree. This
# makes life simpler for git usage
ifeq ($(wildcard $(PX4_ROOT)/NuttX),)
 ifeq ($(wildcard $(PX4_ROOT)/../NuttX),)
 $(error ERROR: NuttX git tree not found)
 endif
$(shell cd $(PX4_ROOT) && ln -s ../NuttX)
endif

# we have different config files for V1 and V2
PX4_V1_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu-v1_APM.mk
PX4_V2_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu-v2_APM.mk

SKETCHFLAGS=$(SKETCHLIBINCLUDES) -I$(PWD) -DARDUPILOT_BUILD -DCONFIG_HAL_BOARD=HAL_BOARD_PX4 -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=ArduPilot_main

PX4_MAKE = make -C $(BUILDROOT) -f $(PX4_ROOT)/Makefile EXTRADEFINES="$(SKETCHFLAGS) "$(EXTRAFLAGS) APM_MODULE_DIR=$(BUILDROOT) SKETCHBOOK=$(SKETCHBOOK) PX4_ROOT=$(PX4_ROOT)

$(BUILDROOT)/module.mk:
	$(RULEHDR)
	$(v) echo "# Auto-generated file - do not edit" > $@
	$(v) echo "MODULE_COMMAND = ArduPilot" >> $@
	$(v) echo "SRCS = $(SKETCH).cpp $(SKETCHLIBSRCS)" >> $@
	$(v) echo "MODULE_STACKSIZE = 4096" >> $@

px4-v1: $(PX4_ROOT)/Archives/px4fmu-v1.export $(SKETCHCPP) $(BUILDROOT)/module.mk px4-io-v1
	$(RULEHDR)
	# we shouldn't need to remove these files ....
	$(v) find $(SKETCHBOOK)/libraries -type f -name '*.cpp.d' -exec rm -f {} \;
	$(v) find $(SKETCHBOOK)/libraries -type f -name '*.cpp.o' -exec rm -f {} \;
	$(v) ln -sf $(PWD)/$(PX4_V1_CONFIG_FILE) $(PX4_ROOT)/makefiles/
	$(v) $(PX4_MAKE) clean
	$(v) $(PX4_MAKE) px4fmu-v1_APM
	$(v) /bin/rm -f $(SKETCH)-v1.px4
	$(v) cp $(PX4_ROOT)/Images/px4fmu-v1_APM.px4 $(SKETCH)-v1.px4
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH)-v1.px4"

px4-v2: $(PX4_ROOT)/Archives/px4fmu-v2.export $(SKETCHCPP) $(BUILDROOT)/module.mk px4-io-v2
	$(RULEHDR)
	# we shouldn't need to remove these files ....
	$(v) find $(SKETCHBOOK)/libraries -type f -name '*.cpp.d' -exec rm -f {} \;
	$(v) find $(SKETCHBOOK)/libraries -type f -name '*.cpp.o' -exec rm -f {} \;
	$(v) ln -sf $(PWD)/$(PX4_V2_CONFIG_FILE) $(PX4_ROOT)/makefiles/
	$(v) $(PX4_MAKE) clean
	$(v) $(PX4_MAKE) px4fmu-v2_APM
	$(v) /bin/rm -f $(SKETCH)-v2.px4
	$(v) cp $(PX4_ROOT)/Images/px4fmu-v2_APM.px4 $(SKETCH)-v2.px4
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH)-v2.px4"

px4: px4-v1 px4-v2

px4-clean: clean px4-archives-clean
	$(v) /bin/rm -rf $(PX4_ROOT)/makefiles/build $(PX4_ROOT)/Build

px4-v1-upload: px4-v1
	$(RULEHDR)
	$(v) ln -sf $(PWD)/$(PX4_V1_CONFIG_FILE) $(PX4_ROOT)/makefiles/
	$(v) $(PX4_MAKE) px4fmu-v1_APM upload

px4-v2-upload: px4-v2
	$(RULEHDR)
	$(v) ln -sf $(PWD)/$(PX4_V2_CONFIG_FILE) $(PX4_ROOT)/makefiles/
	$(v) $(PX4_MAKE) px4fmu-v2_APM upload

px4-upload: px4-v1-upload

px4-archives-clean:
	$(v) /bin/rm -rf $(PX4_ROOT)/Archives

px4-io-v1: $(PX4_ROOT)/Archives/px4io-v1.export
	$(v) make -C $(PX4_ROOT) px4io-v1_default
	$(v) /bin/rm -f px4io-v1.bin
	$(v) cp $(PX4_ROOT)/Images/px4io-v1_default.bin px4io-v1.bin
	$(v) mkdir -p $(MK_DIR)/PX4/ROMFS/px4io/
	$(v) rm -f $(MK_DIR)/PX4/ROMFS/px4io/px4io.bin
	$(v) cp px4io-v1.bin $(MK_DIR)/PX4/ROMFS/px4io/px4io.bin
	$(v) echo "PX4IOv1 Firmware is in px4io-v1.bin"


px4-io-v2: $(PX4_ROOT)/Archives/px4io-v2.export
	$(v) make -C $(PX4_ROOT) px4io-v2_default
	$(v) /bin/rm -f px4io-v1.bin
	$(v) cp $(PX4_ROOT)/Build/px4io-v2_default.build/firmware.bin px4io-v2.bin
	$(v) cp $(PX4_ROOT)/Images/px4io-v2_default.bin px4io-v2.bin
	$(v) mkdir -p $(MK_DIR)/PX4/ROMFS/px4io/
	$(v) rm -f $(MK_DIR)/PX4/ROMFS/px4io/px4io.bin
	$(v) cp px4io-v2.bin $(MK_DIR)/PX4/ROMFS/px4io/px4io.bin
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

px4-archives:
	make -C $(PX4_ROOT) archives

else

px4:
	$(error ERROR: You need to add PX4_ROOT to your config.mk)

px4-clean: px4

px4-upload: px4

endif
