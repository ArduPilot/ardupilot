# VRBRAIN build is via external build system

ifneq ($(VRBRAIN_ROOT),)

# cope with relative paths
ifeq ($(wildcard $(VRBRAIN_ROOT)/nuttx-configs),)
VRBRAIN_ROOT := $(shell cd $(SKETCHBOOK)/$(VRBRAIN_ROOT) && pwd)
endif

# check it is a valid VRBRAIN Firmware tree
ifeq ($(wildcard $(VRBRAIN_ROOT)/nuttx-configs),)
$(error ERROR: VRBRAIN_ROOT not set correctly - no nuttx-configs directory found)
endif

# default to VRBRAIN NuttX above the VRBRAIN Firmware tree
ifeq ($(VRBRAIN_NUTTX_SRC),)
VRBRAIN_NUTTX_SRC := $(shell cd $(VRBRAIN_ROOT)/NuttX/nuttx && pwd)/
endif

# cope with relative paths for VRBRAIN_NUTTX_SRC
ifeq ($(wildcard $(VRBRAIN_NUTTX_SRC)/configs),)
VRBRAIN_NUTTX_SRC := $(shell cd $(SKETCHBOOK)/$(VRBRAIN_NUTTX_SRC) && pwd)/
endif

ifeq ($(wildcard $(VRBRAIN_NUTTX_SRC)configs),)
$(error ERROR: VRBRAIN_NUTTX_SRC not set correctly - no configs directory found)
endif







# we have different config files for vrbrain_v40, vrbrain_v45, vrbrain_v50, vrbrain_v51, vrubrain_v51 and vrhero_v10
VRBRAIN_MK_DIR=$(SRCROOT)/$(MK_DIR)/VRBRAIN
VRBRAIN_VB40_CONFIG_FILE=config_vrbrain-v40_APM.mk
VRBRAIN_VB45_CONFIG_FILE=config_vrbrain-v45_APM.mk
VRBRAIN_VB50_CONFIG_FILE=config_vrbrain-v50_APM.mk
VRBRAIN_VB51_CONFIG_FILE=config_vrbrain-v51_APM.mk
VRBRAIN_VU51_CONFIG_FILE=config_vrubrain-v51_APM.mk
VRBRAIN_VH10_CONFIG_FILE=config_vrhero-v10_APM.mk

SKETCHFLAGS=$(SKETCHLIBINCLUDES) -I$(PWD) -DARDUPILOT_BUILD -DTESTS_MATHLIB_DISABLE -DCONFIG_HAL_BOARD=HAL_BOARD_VRBRAIN -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=ArduPilot_main -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)

WARNFLAGS = -Wno-psabi -Wno-packed

VRBRAIN_MAKE = $(v) make -C $(SKETCHBOOK) -f $(VRBRAIN_ROOT)/Makefile EXTRADEFINES="$(SKETCHFLAGS) $(WARNFLAGS) "'$(EXTRAFLAGS)' APM_MODULE_DIR=$(SKETCHBOOK) SKETCHBOOK=$(SKETCHBOOK) VRBRAIN_ROOT=$(VRBRAIN_ROOT) VRBRAIN_NUTTX_SRC=$(VRBRAIN_NUTTX_SRC) MAXOPTIMIZATION="-Os"
VRBRAIN_MAKE_ARCHIVES = make -C $(VRBRAIN_ROOT) VRBRAIN_NUTTX_SRC=$(VRBRAIN_NUTTX_SRC) archives MAXOPTIMIZATION="-Os"

.PHONY: module_mk
module_mk:
	$(RULEHDR)
	$(v) echo "# Auto-generated file - do not edit" > $(SKETCHBOOK)/module.mk.new
	$(v) echo "MODULE_COMMAND = ArduPilot" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "SRCS = Build.$(SKETCH)/$(SKETCH).cpp $(SKETCHLIBSRCSRELATIVE)" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "MODULE_STACKSIZE = 4096" >> $(SKETCHBOOK)/module.mk.new
	$(v) cmp $(SKETCHBOOK)/module.mk $(SKETCHBOOK)/module.mk.new 2>/dev/null || mv $(SKETCHBOOK)/module.mk.new $(SKETCHBOOK)/module.mk
	$(v) rm -f $(SKETCHBOOK)/module.mk.new

vrbrain-v40: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v40.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VB40_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB40_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/
	$(v) $(VRBRAIN_MAKE) vrbrain-v40_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VB40_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v40.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v40.hex
	$(v) rm -f $(SKETCH)-vrbrain-v40.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v40_APM.vrx $(SKETCH)-vrbrain-v40.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v40_APM.hex $(SKETCH)-vrbrain-v40.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v40_APM.bin $(SKETCH)-vrbrain-v40.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v40.vrx"

vrbrain-v45: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v45.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VB45_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB45_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/
	$(v) $(VRBRAIN_MAKE) vrbrain-v45_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VB45_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v45.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v45.hex
	$(v) rm -f $(SKETCH)-vrbrain-v45.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v45_APM.vrx $(SKETCH)-vrbrain-v45.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v45_APM.hex $(SKETCH)-vrbrain-v45.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v45_APM.bin $(SKETCH)-vrbrain-v45.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v45.vrx"

vrbrain-v50: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v50.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VB50_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB50_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/
	$(v) $(VRBRAIN_MAKE) vrbrain-v50_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VB50_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v50.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v50.hex
	$(v) rm -f $(SKETCH)-vrbrain-v50.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v50_APM.vrx $(SKETCH)-vrbrain-v50.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v50_APM.hex $(SKETCH)-vrbrain-v50.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v50_APM.bin $(SKETCH)-vrbrain-v50.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v50.vrx"

vrbrain-v51: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrbrain-v51.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VB51_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VB51_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/
	$(v) $(VRBRAIN_MAKE) vrbrain-v51_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VB51_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v51.vrx
	$(v) rm -f $(SKETCH)-vrbrain-v51.hex
	$(v) rm -f $(SKETCH)-vrbrain-v51.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51_APM.vrx $(SKETCH)-vrbrain-v51.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51_APM.hex $(SKETCH)-vrbrain-v51.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51_APM.bin $(SKETCH)-vrbrain-v51.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v51.vrx"

vrubrain-v51: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrubrain-v51.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VU51_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VU51_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/
	$(v) $(VRBRAIN_MAKE) vrubrain-v51_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VU51_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrubrain-v51.vrx
	$(v) rm -f $(SKETCH)-vrubrain-v51.hex
	$(v) rm -f $(SKETCH)-vrubrain-v51.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v51_APM.vrx $(SKETCH)-vrubrain-v51.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v51_APM.hex $(SKETCH)-vrubrain-v51.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v51_APM.bin $(SKETCH)-vrubrain-v51.bin
	$(v) echo "MICRO VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrubrain-v51.vrx"

vrhero-v10: $(BUILDROOT)/make.flags $(VRBRAIN_ROOT)/Archives/vrhero-v10.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VH10_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_VH10_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/
	$(v) $(VRBRAIN_MAKE) vrhero-v10_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VH10_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrhero-v10.vrx
	$(v) rm -f $(SKETCH)-vrhero-v10.hex
	$(v) rm -f $(SKETCH)-vrhero-v10.bin
	$(v) cp $(VRBRAIN_ROOT)/Images/vrhero-v10_APM.vrx $(SKETCH)-vrhero-v10.vrx
	$(v) cp $(VRBRAIN_ROOT)/Images/vrhero-v10_APM.hex $(SKETCH)-vrhero-v10.hex
	$(v) cp $(VRBRAIN_ROOT)/Images/vrhero-v10_APM.bin $(SKETCH)-vrhero-v10.bin
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrhero-v10.vrx"

#vrbrain: vrbrain-v40 vrbrain-v45 vrbrain-v50 vrbrain-v51 vrubrain-v51 vrhero-v10
vrbrain: vrbrain-v45 vrbrain-v51 vrubrain-v51

vrbrain-clean: clean vrbrain-build-clean vrbrain-archives-clean

vrbrain-build-clean:
	$(v) /bin/rm -rf $(VRBRAIN_ROOT)/makefiles/build $(VRBRAIN_ROOT)/Build

vrbrain-cleandep: clean
	$(v) find $(VRBRAIN_ROOT)/Build -type f -name '*.d' | xargs rm -f

vrbrain-v40-upload: vrbrain-v40
	$(RULEHDR)
	$(v) $(VRBRAIN_MAKE) vrbrain-v40_APM upload

vrbrain-v45-upload: vrbrain-v45
	$(RULEHDR)
	$(v) $(VRBRAIN_MAKE) vrbrain-v45_APM upload

vrbrain-v50-upload: vrbrain-v50
	$(RULEHDR)
	$(v) $(VRBRAIN_MAKE) vrbrain-v50_APM upload

vrbrain-v51-upload: vrbrain-v51
	$(RULEHDR)
	$(v) $(VRBRAIN_MAKE) vrbrain-v51_APM upload

vrubrain-v51-upload: vrubrain-v51
	$(RULEHDR)
	$(v) $(VRBRAIN_MAKE) vrubrain-v51_APM upload

vrhero-v10-upload: vrhero-v10
	$(RULEHDR)
	$(v) $(VRBRAIN_MAKE) vrhero-v10_APM upload

vrbrain-upload: vrbrain-v40-upload

vrbrain-archives-clean:
	$(v) /bin/rm -rf $(VRBRAIN_ROOT)/Archives

$(VRBRAIN_ROOT)/Archives/vrbrain-v40.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

$(VRBRAIN_ROOT)/Archives/vrbrain-v45.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

$(VRBRAIN_ROOT)/Archives/vrbrain-v50.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

$(VRBRAIN_ROOT)/Archives/vrbrain-v51.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

$(VRBRAIN_ROOT)/Archives/vrubrain-v51.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

$(VRBRAIN_ROOT)/Archives/vrhero-v10.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

vrbrain-archives:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

else

vrbrain_nx:
	$(error ERROR: You need to add VRBRAIN_ROOT to your config.mk)

vrbrain-clean: vrbrain

vrbrain-upload: vrbrain

endif
