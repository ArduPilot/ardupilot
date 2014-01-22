# PX4 build is via external build system

ifneq ($(PX4_ROOT),)

# cope with relative paths
ifeq ($(wildcard $(PX4_ROOT)/nuttx-configs),)
PX4_ROOT := $(shell cd $(SKETCHBOOK)/$(PX4_ROOT) && pwd)
endif

# check it is a valid PX4Firmware tree
ifeq ($(wildcard $(PX4_ROOT)/nuttx-configs),)
$(error ERROR: PX4_ROOT not set correctly - no nuttx-configs directory found)
endif

# default to PX4NuttX above the PX4Firmware tree
ifeq ($(NUTTX_SRC),)
NUTTX_SRC := $(shell cd $(PX4_ROOT)/../PX4NuttX/nuttx && pwd)/
endif

# cope with relative paths for NUTTX_SRC
ifeq ($(wildcard $(NUTTX_SRC)/configs),)
NUTTX_SRC := $(shell cd $(SKETCHBOOK)/$(NUTTX_SRC) && pwd)/
endif

ifeq ($(wildcard $(NUTTX_SRC)configs),)
$(error ERROR: NUTTX_SRC not set correctly - no configs directory found)
endif

NUTTX_GIT_VERSION := $(shell cd $(NUTTX_SRC) && git rev-parse HEAD | cut -c1-8)
PX4_GIT_VERSION   := $(shell cd $(PX4_ROOT) && git rev-parse HEAD | cut -c1-8)

EXTRAFLAGS += -DNUTTX_GIT_VERSION="\"$(NUTTX_GIT_VERSION)\""
EXTRAFLAGS += -DPX4_GIT_VERSION="\"$(PX4_GIT_VERSION)\""

# we have different config files for V1 and V2
PX4_V1_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu-v1_APM.mk
PX4_V2_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu-v2_APM.mk

SKETCHFLAGS=$(SKETCHLIBINCLUDES) -I$(PWD) -DARDUPILOT_BUILD -DCONFIG_HAL_BOARD=HAL_BOARD_PX4 -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=ArduPilot_main

WARNFLAGS = -Wno-psabi -Wno-packed

PX4_MAKE = $(v) make -C $(SKETCHBOOK) -f $(PX4_ROOT)/Makefile EXTRADEFINES="$(SKETCHFLAGS) $(WARNFLAGS) "'$(EXTRAFLAGS)' APM_MODULE_DIR=$(SKETCHBOOK) SKETCHBOOK=$(SKETCHBOOK) PX4_ROOT=$(PX4_ROOT) NUTTX_SRC=$(NUTTX_SRC) MAXOPTIMIZATION="-Os"
PX4_MAKE_ARCHIVES = make -C $(PX4_ROOT) NUTTX_SRC=$(NUTTX_SRC) archives MAXOPTIMIZATION="-Os"

.PHONY: module_mk
module_mk:
	$(RULEHDR)
	$(v) echo "# Auto-generated file - do not edit" > $(SKETCHBOOK)/module.mk.new
	$(v) echo "MODULE_COMMAND = ArduPilot" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "SRCS = Build.$(SKETCH)/$(SKETCH).cpp $(SKETCHLIBSRCSRELATIVE)" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "MODULE_STACKSIZE = 4096" >> $(SKETCHBOOK)/module.mk.new
	$(v) cmp $(SKETCHBOOK)/module.mk $(SKETCHBOOK)/module.mk.new 2>/dev/null || mv $(SKETCHBOOK)/module.mk.new $(SKETCHBOOK)/module.mk
	$(v) rm -f $(SKETCHBOOK)/module.mk.new

px4-v1: showflags $(PX4_ROOT)/Archives/px4fmu-v1.export $(SKETCHCPP) module_mk px4-io-v1
	$(RULEHDR)
	$(v) rm -f $(PX4_ROOT)/makefiles/$(PX4_V1_CONFIG_FILE)
	$(v) cp $(PWD)/$(PX4_V1_CONFIG_FILE) $(PX4_ROOT)/makefiles/
	$(v) $(PX4_MAKE) px4fmu-v1_APM
	$(v) /bin/rm -f $(SKETCH)-v1.px4
	$(v) cp $(PX4_ROOT)/Images/px4fmu-v1_APM.px4 $(SKETCH)-v1.px4
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH)-v1.px4"

px4-v2: showflags $(PX4_ROOT)/Archives/px4fmu-v2.export $(SKETCHCPP) module_mk px4-io-v2
	$(RULEHDR)
	$(v) rm -f $(PX4_ROOT)/makefiles/$(PX4_V2_CONFIG_FILE)
	$(v) cp $(PWD)/$(PX4_V2_CONFIG_FILE) $(PX4_ROOT)/makefiles/
	$(PX4_MAKE) px4fmu-v2_APM
	$(v) /bin/rm -f $(SKETCH)-v2.px4
	$(v) cp $(PX4_ROOT)/Images/px4fmu-v2_APM.px4 $(SKETCH)-v2.px4
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH)-v2.px4"

px4: px4-v1 px4-v2

px4-clean: clean px4-archives-clean
	$(v) /bin/rm -rf $(PX4_ROOT)/makefiles/build $(PX4_ROOT)/Build

px4-cleandep: clean
	$(v) find $(PX4_ROOT)/Build -type f -name '*.d' | xargs rm -f

px4-v1-upload: px4-v1
	$(RULEHDR)
	$(v) $(PX4_MAKE) px4fmu-v1_APM upload

px4-v2-upload: px4-v2
	$(RULEHDR)
	$(v) $(PX4_MAKE) px4fmu-v2_APM upload

px4-upload: px4-v1-upload

px4-archives-clean:
	$(v) /bin/rm -rf $(PX4_ROOT)/Archives

px4-io-v1: $(PX4_ROOT)/Archives/px4io-v1.export
	$(v) make -C $(PX4_ROOT) px4io-v1_default
	$(v) /bin/rm -f px4io-v1.bin
	$(v) cp $(PX4_ROOT)/Images/px4io-v1_default.bin px4io-v1.bin
	$(v) cp $(PX4_ROOT)/Build/px4io-v1_default.build/firmware.elf px4io-v1.elf
	$(v) mkdir -p $(MK_DIR)/PX4/ROMFS/px4io/
	$(v) rm -f $(MK_DIR)/PX4/ROMFS/px4io/px4io.bin
	$(v) cp px4io-v1.bin $(MK_DIR)/PX4/ROMFS/px4io/px4io.bin
	$(v) mkdir -p $(MK_DIR)/PX4/ROMFS/bootloader/
	$(v) rm -f $(MK_DIR)/PX4/ROMFS/bootloader/fmu_bl.bin
	$(v) cp $(SKETCHBOOK)/mk/PX4/bootloader/px4fmu_bl.bin $(MK_DIR)/PX4/ROMFS/bootloader/fmu_bl.bin
	$(v) echo "PX4IOv1 Firmware is in px4io-v1.bin"


px4-io-v2: $(PX4_ROOT)/Archives/px4io-v2.export
	$(v) make -C $(PX4_ROOT) px4io-v2_default
	$(v) /bin/rm -f px4io-v2.bin
	$(v) cp $(PX4_ROOT)/Build/px4io-v2_default.build/firmware.bin px4io-v2.bin
	$(v) cp $(PX4_ROOT)/Images/px4io-v2_default.bin px4io-v2.bin
	$(v) cp $(PX4_ROOT)/Build/px4io-v2_default.build/firmware.elf px4io-v2.elf
	$(v) mkdir -p $(MK_DIR)/PX4/ROMFS/px4io/
	$(v) rm -f $(MK_DIR)/PX4/ROMFS/px4io/px4io.bin
	$(v) cp px4io-v2.bin $(MK_DIR)/PX4/ROMFS/px4io/px4io.bin
	$(v) mkdir -p $(MK_DIR)/PX4/ROMFS/bootloader/
	$(v) rm -f $(MK_DIR)/PX4/ROMFS/bootloader/fmu_bl.bin
	$(v) cp $(SKETCHBOOK)/mk/PX4/bootloader/px4fmuv2_bl.bin $(MK_DIR)/PX4/ROMFS/bootloader/fmu_bl.bin
	$(v) echo "PX4IOv2 Firmware is in px4io-v2.bin"

px4-io: px4-io-v1 px4-io-v2


$(PX4_ROOT)/Archives/px4fmu-v1.export:
	$(v) $(PX4_MAKE_ARCHIVES)

$(PX4_ROOT)/Archives/px4fmu-v2.export:
	$(v) $(PX4_MAKE_ARCHIVES)

$(PX4_ROOT)/Archives/px4io-v1.export:
	$(v) $(PX4_MAKE_ARCHIVES)

$(PX4_ROOT)/Archives/px4io-v2.export:
	$(v) $(PX4_MAKE_ARCHIVES)

px4-archives:
	$(v) $(PX4_MAKE_ARCHIVES)

else

px4:
	$(error ERROR: You need to add PX4_ROOT to your config.mk)

px4-clean: px4

px4-upload: px4

endif
