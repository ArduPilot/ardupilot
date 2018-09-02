# VRBRAIN build is via external build system

ifneq ($(VRBRAIN_ROOT),)
$(error VRBRAIN_ROOT found in config.mk - Please see http://dev.ardupilot.org/wiki/git-submodules/)
endif

ifneq ($(NUTTX_SRC),)
$(error NUTTX_SRC found in config.mk - Please see http://dev.ardupilot.org/wiki/git-submodules/)
endif

#ifneq ($(UAVCAN_DIR),)
#$(error UAVCAN_DIR found in config.mk - Please see http://dev.ardupilot.org/wiki/git-submodules/)
#endif

# these can be overridden in developer.mk
VRBRAINFIRMWARE_DIRECTORY ?= $(SKETCHBOOK)/modules/PX4Firmware
VRBRAINNUTTX_DIRECTORY ?= $(SKETCHBOOK)/modules/PX4NuttX
UAVCAN_DIRECTORY ?= $(SKETCHBOOK)/modules/uavcan

VRBRAIN_ROOT := $(shell cd $(VRBRAINFIRMWARE_DIRECTORY) && pwd)
NUTTX_ROOT := $(shell cd $(VRBRAINNUTTX_DIRECTORY) && pwd)
NUTTX_SRC := $(NUTTX_ROOT)/nuttx/
UAVCAN_DIR=$(shell cd $(UAVCAN_DIRECTORY) && pwd)/

NUTTX_GIT_VERSION ?= $(shell cd $(NUTTX_SRC) && git rev-parse HEAD | cut -c1-8)
PX4_GIT_VERSION   ?= $(shell cd $(VRBRAIN_ROOT) && git rev-parse HEAD | cut -c1-8)

EXTRAFLAGS += -DNUTTX_GIT_VERSION="\"$(NUTTX_GIT_VERSION)\""
EXTRAFLAGS += -DPX4_GIT_VERSION="\"$(PX4_GIT_VERSION)\""
EXTRAFLAGS += -DUAVCAN=1
EXTRAFLAGS += -D__STDC_FORMAT_MACROS

# Add missing parts from libc and libstdc++
EXTRAFLAGS += -DHAVE_STD_NULLPTR_T=0
EXTRAFLAGS += -DHAVE_ENDIAN_H=0
EXTRAFLAGS += -DHAVE_BYTESWAP_H=0
EXTRAFLAGS += -DHAVE_OCLOEXEC=0

EXTRAFLAGS += -I$(BUILDROOT)/libraries/GCS_MAVLink/include/mavlink
EXTRAFLAGS += -I$(UAVCAN_DIRECTORY)/libuavcan/include
EXTRAFLAGS += -I$(UAVCAN_DIRECTORY)/libuavcan/include/dsdlc_generated

VRBRAIN_MK_DIR=$(MK_DIR)/VRBRAIN

# we have different config files for vrbrain-v51, vrbrain-v52, vrbrain-v54, vrcore-v10, vrubrain-v51, vrubrain-v52
VRBRAIN_V51_CONFIG_FILE=config_vrbrain-v51_APM.mk
VRBRAIN_V52_CONFIG_FILE=config_vrbrain-v52_APM.mk
VRBRAIN_V52E_CONFIG_FILE=config_vrbrain-v52E_APM.mk
VRBRAIN_V54_CONFIG_FILE=config_vrbrain-v54_APM.mk
VRCORE_V10_CONFIG_FILE=config_vrcore-v10_APM.mk
VRUBRAIN_V51_CONFIG_FILE=config_vrubrain-v51_APM.mk
VRUBRAIN_V52_CONFIG_FILE=config_vrubrain-v52_APM.mk

# Since actual compiler mode is C++11, the library will default to UAVCAN_CPP11, but it will fail to compile
# because this platform lacks most of the standard library and STL. Hence we need to force C++03 mode.
SKETCHFLAGS=$(SKETCHLIBINCLUDES) -DUAVCAN_CPP_VERSION=UAVCAN_CPP03 -DUAVCAN_NO_ASSERTIONS -DUAVCAN_NULLPTR=nullptr -DARDUPILOT_BUILD -DTESTS_MATHLIB_DISABLE -DCONFIG_HAL_BOARD=HAL_BOARD_VRBRAIN -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=ArduPilot_main -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)

WARNFLAGS = -Wall -Wextra -Wlogical-op -Werror -Wno-attributes -Wno-unknown-pragmas -Wno-redundant-decls -Wno-psabi -Wno-packed -Wno-error=double-promotion -Wno-error=unused-variable -Wno-error=reorder -Wno-error=float-equal -Wno-error=pmf-conversions -Wno-error=missing-declarations -Wno-error=unused-function -Wno-trigraphs
OPTFLAGS = -fsingle-precision-constant

# avoid VRBRAIN submodules
export GIT_SUBMODULES_ARE_EVIL = 1

PYTHONPATH=$(VRBRAIN_ROOT)/Tools/genmsg/src:$(VRBRAIN_ROOT)/Tools/gencpp/src
export PYTHONPATH

VRBRAIN_MAKE = $(v)+ GIT_SUBMODULES_ARE_EVIL=1 ARDUPILOT_BUILD=1 $(MAKE) -C $(SKETCHBOOK) -f $(VRBRAIN_ROOT)/Makefile.make EXTRADEFINES="$(SKETCHFLAGS) $(WARNFLAGS) $(OPTFLAGS) "'$(EXTRAFLAGS)' APM_MODULE_DIR=$(SKETCHBOOK) SKETCHBOOK=$(SKETCHBOOK) CCACHE=$(CCACHE) VRBRAIN_ROOT=$(VRBRAIN_ROOT) NUTTX_SRC=$(NUTTX_SRC) MAXOPTIMIZATION="-Os" UAVCAN_DIR=$(UAVCAN_DIR)
VRBRAIN_MAKE_ARCHIVES = $(MAKE) -C $(VRBRAIN_ROOT) -f $(VRBRAIN_ROOT)/Makefile.make NUTTX_SRC=$(NUTTX_SRC) CCACHE=$(CCACHE) archives MAXOPTIMIZATION="-Os"

HASHADDER_FLAGS += --ardupilot "$(SKETCHBOOK)"

ifneq ($(wildcard $(VRBRAIN_ROOT)),)
HASHADDER_FLAGS += --px4 "$(VRBRAIN_ROOT)"
endif
ifneq ($(wildcard $(NUTTX_SRC)/..),)
HASHADDER_FLAGS += --nuttx "$(NUTTX_SRC)/.."
endif
HASHADDER_FLAGS += --uavcan "$(UAVCAN_DIR)"

.PHONY: module_mk
module_mk:
	$(v) echo "Building $(SKETCHBOOK)/module.mk"
	$(RULEHDR)
	$(v) echo "# Auto-generated file - do not edit" > $(SKETCHBOOK)/module.mk.new
	$(v) echo "MODULE_COMMAND = ArduPilot" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "SRCS = $(subst $(SKETCHBOOK)/,,$(wildcard $(SRCROOT)/*.cpp)) $(SKETCHLIBSRCSRELATIVE) $(LIBUAVCAN_SRC)" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "MODULE_STACKSIZE = 4096" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "EXTRACXXFLAGS = -Wframe-larger-than=1300" >> $(SKETCHBOOK)/module.mk.new
	$(v) cmp $(SKETCHBOOK)/module.mk $(SKETCHBOOK)/module.mk.new 2>/dev/null || mv $(SKETCHBOOK)/module.mk.new $(SKETCHBOOK)/module.mk
	$(v) rm -f $(SKETCHBOOK)/module.mk.new

vrbrain-v51: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(VRBRAIN_ROOT)/Archives/vrbrain-v51.export $(SKETCHCPP) module_mk
	$(v) echo Building vrbrain-v51
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_V51_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_V51_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v51_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_V51_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v51.vrx
	$(v) arm-none-eabi-size $(VRBRAIN_ROOT)/Build/vrbrain-v51_APM.build/firmware.elf
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v51_APM.px4 $(SKETCH)-vrbrain-v51.vrx
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-vrbrain-v51.vrx" "$(SKETCH)-vrbrain-v51.vrx"
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v51.vrx"

vrbrain-v52: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(VRBRAIN_ROOT)/Archives/vrbrain-v52.export $(SKETCHCPP) module_mk
	$(v) echo Building vrbrain-v52
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_V52_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_V52_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v52_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_V52_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v52.vrx
	$(v) arm-none-eabi-size $(VRBRAIN_ROOT)/Build/vrbrain-v52_APM.build/firmware.elf
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52_APM.px4 $(SKETCH)-vrbrain-v52.vrx
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-vrbrain-v52.vrx" "$(SKETCH)-vrbrain-v52.vrx"
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v52.vrx"

vrbrain-v52E: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(VRBRAIN_ROOT)/Archives/vrbrain-v52E.export $(SKETCHCPP) module_mk
	$(v) echo Building vrbrain-v52E
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_V52E_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_V52E_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v52E_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_V52E_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v52E.vrx
	$(v) arm-none-eabi-size $(VRBRAIN_ROOT)/Build/vrbrain-v52E_APM.build/firmware.elf
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v52E_APM.px4 $(SKETCH)-vrbrain-v52E.vrx
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-vrbrain-v52E.vrx" "$(SKETCH)-vrbrain-v52E.vrx"
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v52E.vrx"

vrbrain-v54: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(VRBRAIN_ROOT)/Archives/vrbrain-v54.export $(SKETCHCPP) module_mk
	$(v) echo Building vrbrain-v54
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_V54_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRBRAIN_V54_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrbrain-v54_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRBRAIN_V54_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrbrain-v54.vrx
	$(v) arm-none-eabi-size $(VRBRAIN_ROOT)/Build/vrbrain-v54_APM.build/firmware.elf
	$(v) cp $(VRBRAIN_ROOT)/Images/vrbrain-v54_APM.px4 $(SKETCH)-vrbrain-v54.vrx
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-vrbrain-v54.vrx" "$(SKETCH)-vrbrain-v54.vrx"
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrbrain-v54.vrx"

vrcore-v10: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(VRBRAIN_ROOT)/Archives/vrcore-v10.export $(SKETCHCPP) module_mk
	$(v) echo Building vrcore-v10
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRCORE_V10_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRCORE_V10_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrcore-v10_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRCORE_V10_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrcore-v10.vrx
	$(v) arm-none-eabi-size $(VRBRAIN_ROOT)/Build/vrcore-v10_APM.build/firmware.elf
	$(v) cp $(VRBRAIN_ROOT)/Images/vrcore-v10_APM.px4 $(SKETCH)-vrcore-v10.vrx
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-vrcore-v10.vrx" "$(SKETCH)-vrcore-v10.vrx"
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrcore-v10.vrx"

vrubrain-v51: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(VRBRAIN_ROOT)/Archives/vrubrain-v51.export $(SKETCHCPP) module_mk
	$(v) echo Building vrubrain-v51
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRUBRAIN_V51_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRUBRAIN_V51_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrubrain-v51_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRUBRAIN_V51_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrubrain-v51.vrx
	$(v) arm-none-eabi-size $(VRBRAIN_ROOT)/Build/vrubrain-v51_APM.build/firmware.elf
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v51_APM.px4 $(SKETCH)-vrubrain-v51.vrx
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-vrubrain-v51.vrx" "$(SKETCH)-vrubrain-v51.vrx"
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrubrain-v51.vrx"

vrubrain-v52: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(VRBRAIN_ROOT)/Archives/vrubrain-v52.export $(SKETCHCPP) module_mk
	$(v) echo Building vrubrain-v52
	$(RULEHDR)
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRUBRAIN_V52_CONFIG_FILE)
	$(v) cp $(VRBRAIN_MK_DIR)/$(VRUBRAIN_V52_CONFIG_FILE) $(VRBRAIN_ROOT)/makefiles/nuttx/
	$(v) $(VRBRAIN_MAKE) vrubrain-v52_APM
	$(v) rm -f $(VRBRAIN_ROOT)/makefiles/nuttx/$(VRUBRAIN_V52_CONFIG_FILE)
	$(v) rm -f $(SKETCH)-vrubrain-v52.vrx
	$(v) arm-none-eabi-size $(VRBRAIN_ROOT)/Build/vrubrain-v52_APM.build/firmware.elf
	$(v) cp $(VRBRAIN_ROOT)/Images/vrubrain-v52_APM.px4 $(SKETCH)-vrubrain-v52.vrx
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-vrubrain-v52.vrx" "$(SKETCH)-vrubrain-v52.vrx"
	$(v) echo "VRBRAIN $(SKETCH) Firmware is in $(SKETCH)-vrubrain-v52.vrx"
	
# force the 3 build types to not run in parallel. We got bad binaries with incorrect parameter handling
# when these were allowed to happen in parallel
vrbrainStd:
	$(MAKE) vrbrain-v51
	$(MAKE) vrbrain-v52
	$(MAKE) vrbrain-v52E
	$(MAKE) vrbrain-v54
	$(MAKE) vrcore-v10
	$(MAKE) vrubrain-v51
vrbrainStdP: 
vrbrainPro: 
vrbrainProP: 

vrbrain: vrbrainStd vrbrainStdP vrbrainPro vrbrainProP

vrbrain-clean: clean CHECK_MODULES vrbrain-archives-clean vrbrain-cleandep
	$(v) /bin/rm -rf $(VRBRAIN_ROOT)/makefiles/build $(VRBRAIN_ROOT)/Build $(VRBRAIN_ROOT)/Images/*.px4 $(VRBRAIN_ROOT)/Images/*.bin
	$(v) /bin/rm -rf $(VRBRAIN_ROOT)/src/modules/uORB/topics $(VRBRAIN_ROOT)/src/platforms/nuttx/px4_messages
	$(v) /bin/rm -f $(SRCROOT)/*.o

vrbrain-cleandep: clean
	$(v) mkdir -p $(VRBRAIN_ROOT)/Build
	$(v) find $(VRBRAIN_ROOT)/Build -type f -name '*.d' | xargs rm -f
	$(v) find $(UAVCAN_DIRECTORY) -type f -name '*.d' | xargs rm -f
	$(v) find $(SKETCHBOOK)/$(SKETCH) -type f -name '*.d' | xargs rm -f

























	


vrbrain-archives-clean:
	$(v) /bin/rm -rf $(VRBRAIN_ROOT)/Archives
























# These targets can't run in parallel because they all need to generate a tool
# to generate the config.h inside them. This could trigger races if done in
# parallel, trying to generate the tool and replacing it while the header is already
# being generated
#
# We could serialize inside PX4Firmware, but it's easier to serialize here
# while maintaining the rest of the build parallelized

.NOTPARALLEL: \
	$(VRBRAIN_ROOT)/Archives/vrbrain-v51.export \
	$(VRBRAIN_ROOT)/Archives/vrbrain-v52.export \
	$(VRBRAIN_ROOT)/Archives/vrbrain-v52E.export \
	$(VRBRAIN_ROOT)/Archives/vrbrain-v54.export \
	$(VRBRAIN_ROOT)/Archives/vrcore-v10.export \
	$(VRBRAIN_ROOT)/Archives/vrubrain-v51.export \
	$(VRBRAIN_ROOT)/Archives/vrubrain-v52.export

$(VRBRAIN_ROOT)/Archives/vrbrain-v51.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v51"

$(VRBRAIN_ROOT)/Archives/vrbrain-v52.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v52"

$(VRBRAIN_ROOT)/Archives/vrbrain-v52E.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v52E"

$(VRBRAIN_ROOT)/Archives/vrbrain-v54.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrbrain-v54"

$(VRBRAIN_ROOT)/Archives/vrcore-v10.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrcore-v10"

$(VRBRAIN_ROOT)/Archives/vrubrain-v51.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrubrain-v51"

$(VRBRAIN_ROOT)/Archives/vrubrain-v52.export:
	$(v) $(VRBRAIN_MAKE_ARCHIVES) BOARDS="vrubrain-v52"

vrbrain-archives:
	$(v) $(PX4_MAKE_ARCHIVES) BOARDS="vrbrain-v51 vrbrain-v52 vrbrain-v52E vrbrain-v54 vrcore-v10 vrubrain-v51 vrubrain-v52"

vrbrain-info: module_mk
	@echo "BUILDROOT                    $(BUILDROOT)"
	@echo "VRBRAINFIRMWARE_DIRECTORY    $(VRBRAINFIRMWARE_DIRECTORY)"
	@echo "VRBRAINNUTTX_DIRECTORY       $(VRBRAINNUTTX_DIRECTORY)"
	@echo "NUTTX_ROOT                   $(NUTTX_ROOT)"
	@echo "VRBRAIN_ROOT                 $(VRBRAIN_ROOT)"
	@echo "NUTTX_SRC                    $(NUTTX_SRC)"
	@echo "SKETCHLIBS                   $(SKETCHLIBS)"
	@echo "SKETCHLIBNAMES               $(SKETCHLIBNAMES)"
	@echo "SKETCHLIBSRCDIRS             $(SKETCHLIBSRCDIRS)"
	@echo "SKETCHLIBSRCS                $(SKETCHLIBSRCS)"
	@echo "SKETCHLIBOBJS                $(SKETCHLIBOBJS)"
	@echo "SKETCHLIBINCLUDES            $(SKETCHLIBINCLUDES)"
	@echo "SKETCHLIBSRCSRELATIVE        $(SKETCHLIBSRCSRELATIVE)"
	@echo "SRCS                         $(subst $(SKETCHBOOK)/,,$(wildcard $(SRCROOT)/*.cpp))"
	@echo "SRCS                         $(wildcard $(SRCROOT)/*.cpp)"
	@echo "HASHADDER_FLAGS              $(HASHADDER_FLAGS)"