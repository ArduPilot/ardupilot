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







# we have different config files for vrbrain_v4, vrbrain_v5 and vrhero_v1
VRBRAIN_VB4_CONFIG_FILE=$(MK_DIR)/VRBRAIN/config_vrbrain-v4_APM.mk
VRBRAIN_VB5_CONFIG_FILE=$(MK_DIR)/VRBRAIN/config_vrbrain-v5_APM.mk
VRBRAIN_VH1_CONFIG_FILE=$(MK_DIR)/VRBRAIN/config_vrhero-v1_APM.mk

SKETCHFLAGS=$(SKETCHLIBINCLUDES) -I$(PWD) -DARDUPILOT_BUILD -DCONFIG_HAL_BOARD=HAL_BOARD_VRBRAIN -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=ArduPilot_main -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)

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

vrbrain-v4: showflags $(VRBRAIN_ROOT)/Archives/vrbrain-v4.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VB4_CONFIG_FILE)
	$(v) cp $(SRCROOT)/$(VRBRAIN_VB4_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/
	$(v) $(VRBRAIN_MAKE) vrbrain-v4_APM
	$(v) /bin/rm -f $(SKETCH)-vrbrain-v4.vrbrain
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v4_APM.vrbrain $(SKETCH)-vrbrain-v4.vbrain
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v4.vbrain"

vrbrain-v5: showflags $(VRBRAIN_ROOT)/Archives/vrbrain-v5.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VB5_CONFIG_FILE)
	$(v) cp $(SRCROOT)/$(VRBRAIN_VB5_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/
	$(v) $(VRBRAIN_MAKE) vrbrain-v5_APM
	$(v) /bin/rm -f $(SKETCH)-vrbrain-v5.vrbrain
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v5_APM.vrbrain $(SKETCH)-vrbrain-v5.vbrain
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v5.vbrain"

vrhero-v1: showflags $(VRBRAIN_ROOT)/Archives/vrhero-v1.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/$(VRBRAIN_VH1_CONFIG_FILE)
	$(v) cp $(SRCROOT)/$(VRBRAIN_VH1_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/
	$(v) $(VRBRAIN_MAKE) vrhero-v1_APM
	$(v) /bin/rm -f $(SKETCH)-vrhero-v1.vrbrain
	$(v) cp $(VRBRAIN_ROOT)/Images/vrhero-v1_APM.vrbrain $(SKETCH)-vrhero-v1.vbrain
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrhero-v1.vbrain"

#vrbrain: vrbrain-v4 vrbrain-v5 vrhero-v1
vrbrain: vrbrain-v4 vrbrain-v5
#vrbrain: vrbrain-v4

vrbrain-clean: clean vrbrain-archives-clean
	$(v) /bin/rm -rf $(VRBRAIN_ROOT)/makefiles/build $(VRBRAIN_ROOT)/Build

vrbrain-cleandep: clean
	$(v) find $(VRBRAIN_ROOT)/Build -type f -name '*.d' | xargs rm -f

vrbrain-v4-upload: vrbrain-v4
	$(RULEHDR)
	$(v) $(VRBRAIN_MAKE) vrbrain-v4_APM upload

vrbrain-v5-upload: vrbrain-v5
	$(RULEHDR)
	$(v) $(VRBRAIN_MAKE) vrbrain-v5_APM upload

vrhero-v1-upload: vrhero-v1
	$(RULEHDR)
	$(v) $(VRBRAIN_MAKE) vrhero-v1_APM upload

vrbrain-upload: vrbrain-v4-upload

vrbrain-archives-clean:
	$(v) /bin/rm -rf $(VRBRAIN_ROOT)/Archives

$(VRBRAIN_ROOT)/Archives/vrbrain-v4.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

$(VRBRAIN_ROOT)/Archives/vrbrain-v5.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

$(VRBRAIN_ROOT)/Archives/vrhero-v1.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

vrbrain-archives:
	$(v) $(VRBRAIN_MAKE_ARCHIVES)

else

vrbrain_nx:
	$(error ERROR: You need to add VRBRAIN_ROOT to your config.mk)

vrbrain-clean: vrbrain

vrbrain-upload: vrbrain

endif
