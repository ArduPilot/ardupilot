# PX4 build is via external build system

ifneq ($(PX4_ROOT),)
$(error PX4_ROOT found in config.mk - Please see http://dev.ardupilot.org/wiki/git-submodules/)
endif

ifneq ($(NUTTX_SRC),)
$(error NUTTX_SRC found in config.mk - Please see http://dev.ardupilot.org/wiki/git-submodules/)
endif

#ifneq ($(UAVCAN_DIR),)
#$(error UAVCAN_DIR found in config.mk - Please see http://dev.ardupilot.org/wiki/git-submodules/)
#endif

# these can be overridden in developer.mk
PX4FIRMWARE_DIRECTORY ?= $(SKETCHBOOK)/modules/PX4Firmware
PX4NUTTX_DIRECTORY ?= $(SKETCHBOOK)/modules/PX4NuttX
UAVCAN_DIRECTORY ?= $(SKETCHBOOK)/modules/uavcan

PX4_ROOT := $(shell cd $(PX4FIRMWARE_DIRECTORY) && pwd)
NUTTX_ROOT := $(shell cd $(PX4NUTTX_DIRECTORY) && pwd)
NUTTX_SRC := $(NUTTX_ROOT)/nuttx/
UAVCAN_DIR=$(shell cd $(UAVCAN_DIRECTORY) && pwd)/

NUTTX_GIT_VERSION ?= $(shell cd $(NUTTX_SRC) && git rev-parse HEAD | cut -c1-8)
PX4_GIT_VERSION   ?= $(shell cd $(PX4_ROOT) && git rev-parse HEAD | cut -c1-8)

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

# we have different config files for V1 and V2
PX4_V1_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu-v1_APM.mk
PX4_V2_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu-v2_APM.mk
PX4_V3_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu-v3_APM.mk
PX4_V4_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu-v4_APM.mk
PX4_V4PRO_CONFIG_FILE=$(MK_DIR)/PX4/config_px4fmu-v4pro_APM.mk

# Since actual compiler mode is C++11, the library will default to UAVCAN_CPP11, but it will fail to compile
# because this platform lacks most of the standard library and STL. Hence we need to force C++03 mode.
SKETCHFLAGS=$(SKETCHLIBINCLUDES) -DUAVCAN_CPP_VERSION=UAVCAN_CPP03 -DUAVCAN_NO_ASSERTIONS -DUAVCAN_NULLPTR=nullptr -DARDUPILOT_BUILD -DTESTS_MATHLIB_DISABLE -DCONFIG_HAL_BOARD=HAL_BOARD_PX4 -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=ArduPilot_main -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)

WARNFLAGS = -Wall -Wextra -Wlogical-op -Werror -Wno-attributes -Wno-unknown-pragmas -Wno-redundant-decls -Wno-psabi -Wno-packed -Wno-error=double-promotion -Wno-error=unused-variable -Wno-error=reorder -Wno-error=float-equal -Wno-error=pmf-conversions -Wno-error=missing-declarations -Wno-error=unused-function -Wno-trigraphs
OPTFLAGS = -fsingle-precision-constant

# avoid PX4 submodules
export GIT_SUBMODULES_ARE_EVIL = 1

PYTHONPATH=$(SKETCHBOOK)/mk/PX4/Tools/genmsg/src:$(SKETCHBOOK)/mk/PX4/Tools/gencpp/src
export PYTHONPATH

PX4_MAKE = $(v)+ GIT_SUBMODULES_ARE_EVIL=1 ARDUPILOT_BUILD=1 $(MAKE) -C $(SKETCHBOOK) -f $(PX4_ROOT)/Makefile.make EXTRADEFINES="$(SKETCHFLAGS) $(WARNFLAGS) $(OPTFLAGS) "'$(EXTRAFLAGS)' APM_MODULE_DIR=$(SKETCHBOOK) SKETCHBOOK=$(SKETCHBOOK) CCACHE=$(CCACHE) PX4_ROOT=$(PX4_ROOT) NUTTX_SRC=$(NUTTX_SRC) MAXOPTIMIZATION="-Os" UAVCAN_DIR=$(UAVCAN_DIR)
PX4_MAKE_ARCHIVES = $(MAKE) -C $(PX4_ROOT) -f $(PX4_ROOT)/Makefile.make NUTTX_SRC=$(NUTTX_SRC) CCACHE=$(CCACHE) archives MAXOPTIMIZATION="-Os"

HASHADDER_FLAGS += --ardupilot "$(SKETCHBOOK)"

ifneq ($(wildcard $(PX4_ROOT)),)
HASHADDER_FLAGS += --px4 "$(PX4_ROOT)"
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
	$(v) echo "SRCS = $(wildcard $(SRCROOT)/*.cpp) $(SKETCHLIBSRCSRELATIVE) $(LIBUAVCAN_SRC)" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "MODULE_STACKSIZE = 4096" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "EXTRACXXFLAGS = -Wframe-larger-than=1300" >> $(SKETCHBOOK)/module.mk.new
	$(v) cmp $(SKETCHBOOK)/module.mk $(SKETCHBOOK)/module.mk.new 2>/dev/null || mv $(SKETCHBOOK)/module.mk.new $(SKETCHBOOK)/module.mk
	$(v) rm -f $(SKETCHBOOK)/module.mk.new

px4-v1: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(PX4_ROOT)/Archives/px4fmu-v1.export $(SKETCHCPP) module_mk px4-io-v1
	$(v) echo Building px4-v1
	$(RULEHDR)
	$(v) cp $(PX4_V1_CONFIG_FILE) $(PX4_ROOT)/makefiles/nuttx/
	$(v) $(PX4_MAKE) px4fmu-v1_APM
	$(v) arm-none-eabi-size $(PX4_ROOT)/Build/px4fmu-v1_APM.build/firmware.elf
	$(v) cp $(PX4_ROOT)/Images/px4fmu-v1_APM.px4 $(SKETCH)-v1.px4
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-v1.px4" "$(SKETCH)-v1.px4"
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH)-v1.px4"

px4-v2: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(PX4_ROOT)/Archives/px4fmu-v2.export $(SKETCHCPP) module_mk px4-io-v2
	$(v) echo Building px4-v2
	$(RULEHDR)
	$(v) cp $(PX4_V2_CONFIG_FILE) $(PX4_ROOT)/makefiles/nuttx/
	$(PX4_MAKE) px4fmu-v2_APM
	$(v) arm-none-eabi-size $(PX4_ROOT)/Build/px4fmu-v2_APM.build/firmware.elf
	$(v) cp $(PX4_ROOT)/Images/px4fmu-v2_APM.px4 $(SKETCH)-v2.px4
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-v2.px4" "$(SKETCH)-v2.px4"
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH)-v2.px4"

px4-v3: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(PX4_ROOT)/Archives/px4fmu-v3.export $(SKETCHCPP) module_mk px4-io-v2
	$(v) echo Building px4-v3
	$(RULEHDR)
	$(v) cp $(PX4_V3_CONFIG_FILE) $(PX4_ROOT)/makefiles/nuttx/
	$(PX4_MAKE) px4fmu-v3_APM
	$(v) arm-none-eabi-size $(PX4_ROOT)/Build/px4fmu-v3_APM.build/firmware.elf
	$(v) cp $(PX4_ROOT)/Images/px4fmu-v3_APM.px4 $(SKETCH)-v3.px4
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-v3.px4" "$(SKETCH)-v3.px4"
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH)-v3.px4"

px4-v4: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(PX4_ROOT)/Archives/px4fmu-v4.export $(SKETCHCPP) module_mk
	$(v) echo Building px4-v4
	$(RULEHDR)
	$(v) cp $(PX4_V4_CONFIG_FILE) $(PX4_ROOT)/makefiles/nuttx/
	$(PX4_MAKE) px4fmu-v4_APM
	$(v) arm-none-eabi-size $(PX4_ROOT)/Build/px4fmu-v4_APM.build/firmware.elf
	$(v) cp $(PX4_ROOT)/Images/px4fmu-v4_APM.px4 $(SKETCH)-v4.px4
	$(v) mkdir -p $(MK_DIR)/PX4/ROMFS/bootloader/
	$(v) cp $(SKETCHBOOK)/Tools/bootloaders/px4fmuv4_bl.bin $(MK_DIR)/PX4/ROMFS/bootloader/fmu_bl.bin
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-v4.px4" "$(SKETCH)-v4.px4"
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH)-v4.px4"

px4-v4pro: $(BUILDROOT)/make.flags CHECK_MODULES $(MAVLINK_HEADERS) $(UAVCAN_HEADERS) $(PX4_ROOT)/Archives/px4fmu-v4pro.export $(SKETCHCPP) module_mk px4-io-v2
	$(v) echo Building px4-v4pro
	$(RULEHDR)
	$(v) cp $(PX4_V4PRO_CONFIG_FILE) $(PX4_ROOT)/makefiles/nuttx/
	$(PX4_MAKE) px4fmu-v4pro_APM
	$(v) arm-none-eabi-size $(PX4_ROOT)/Build/px4fmu-v4pro_APM.build/firmware.elf
	$(v) cp $(PX4_ROOT)/Images/px4fmu-v4pro_APM.px4 $(SKETCH)-v4pro.px4
	$(v) $(SKETCHBOOK)/Tools/scripts/add_git_hashes.py $(HASHADDER_FLAGS) "$(SKETCH)-v4pro.px4" "$(SKETCH)-v4pro.px4"
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH)-v4pro.px4"

# force the 3 build types to not run in parallel. We got bad binaries with incorrect parameter handling
# when these were allowed to happen in parallel
px4:
	$(MAKE) px4-v1
	$(MAKE) px4-v2
	$(MAKE) px4-v3
	$(MAKE) px4-v4
	$(MAKE) px4-v4pro

px4-clean: clean CHECK_MODULES px4-archives-clean px4-cleandep
	$(v) /bin/rm -rf $(PX4_ROOT)/makefiles/build $(PX4_ROOT)/Build $(PX4_ROOT)/Images/*.px4 $(PX4_ROOT)/Images/*.bin
	$(v) /bin/rm -rf $(PX4_ROOT)/src/modules/uORB/topics $(PX4_ROOT)/src/platforms/nuttx/px4_messages
	$(v) /bin/rm -rf $(SKETCHBOOK)/modules/uavcan/libuavcan/include/dsdlc_generated
	$(v) /bin/rm -f $(SRCROOT)/*.o

px4-cleandep: clean
	$(v) mkdir -p $(PX4_ROOT)/Build
	$(v) find $(PX4_ROOT)/Build -type f -name '*.d' | xargs rm -f
	$(v) find $(UAVCAN_DIRECTORY) -type f -name '*.d' | xargs rm -f
	$(v) find $(SKETCHBOOK)/$(SKETCH) -type f -name '*.d' | xargs rm -f
	$(v) /bin/rm -rf $(SKETCHBOOK)/modules/uavcan/libuavcan/include/dsdlc_generated

px4-v2-upload-solo: px4-v2
	scp $(SKETCH)-v2.px4 root@10.1.1.10:/tmp/
	ssh root@10.1.1.10 PYTHONUNBUFFERED=1 loadPixhawk.py /tmp/ArduCopter-v2.px4
	ssh root@10.1.1.10 rm /tmp/ArduCopter-v2.px4;

px4-v1-upload: px4-v1
	$(RULEHDR)
	$(v) $(PX4_MAKE) px4fmu-v1_APM upload

px4-v2-upload: px4-v2
	$(RULEHDR)
	$(v) $(PX4_MAKE) px4fmu-v2_APM upload

px4-v3-upload: px4-v3
	$(RULEHDR)
	$(v) $(PX4_MAKE) px4fmu-v3_APM upload

px4-v4-upload: px4-v4
	$(RULEHDR)
	$(v) $(PX4_MAKE) px4fmu-v4_APM upload

px4-v4pro-upload: px4-v4pro
	$(RULEHDR)
	$(v) $(PX4_MAKE) px4fmu-v4pro_APM upload	

px4-upload: px4-v1-upload

px4-archives-clean:
	$(v) /bin/rm -rf $(PX4_ROOT)/Archives

px4-io-v1: $(PX4_ROOT)/Archives/px4io-v1.export
	$(v)+ $(MAKE) -C $(PX4_ROOT) -f $(PX4_ROOT)/Makefile.make px4io-v1_default EXTRADEFINES="-DARDUPILOT_BUILD"
	$(v) cp $(PX4_ROOT)/Images/px4io-v1_default.bin px4io-v1.bin
	$(v) cp $(PX4_ROOT)/Build/px4io-v1_default.build/firmware.elf px4io-v1.elf
	$(v) mkdir -p $(MK_DIR)/PX4/ROMFS/px4io/
	$(v) cp px4io-v1.bin $(MK_DIR)/PX4/ROMFS/px4io/px4io.bin
	$(v) mkdir -p $(MK_DIR)/PX4/ROMFS/bootloader/
	$(v) cp $(SKETCHBOOK)/Tools/bootloaders/px4fmu_bl.bin $(MK_DIR)/PX4/ROMFS/bootloader/fmu_bl.bin
	$(v) echo "PX4IOv1 Firmware is in px4io-v1.bin"


px4-io-v2: $(PX4_ROOT)/Archives/px4io-v2.export
	$(v)+ $(MAKE) -C $(PX4_ROOT) -f $(PX4_ROOT)/Makefile.make px4io-v2_default EXTRADEFINES="-DARDUPILOT_BUILD"
	$(v) cp $(PX4_ROOT)/Images/px4io-v2_default.bin px4io-v2.bin
	$(v) cp $(PX4_ROOT)/Build/px4io-v2_default.build/firmware.elf px4io-v2.elf
	$(v) mkdir -p $(MK_DIR)/PX4/ROMFS/px4io/
	$(v) cp px4io-v2.bin $(MK_DIR)/PX4/ROMFS/px4io/px4io.bin
	$(v) mkdir -p $(MK_DIR)/PX4/ROMFS/bootloader/
	$(v) cp $(SKETCHBOOK)/Tools/bootloaders/px4fmuv2_bl.bin $(MK_DIR)/PX4/ROMFS/bootloader/fmu_bl.bin
	$(v) echo "PX4IOv2 Firmware is in px4io-v2.bin"

px4-io: px4-io-v1 px4-io-v2

# These targets can't run in parallel because they all need to generate a tool
# to generate the config.h inside them. This could trigger races if done in
# parallel, trying to generate the tool and replacing it while the header is already
# being generated
#
# We could serialize inside PX4Firmware, but it's easier to serialize here
# while maintaining the rest of the build parallelized

.NOTPARALLEL: \
	$(PX4_ROOT)/Archives/px4fmu-v1.export \
	$(PX4_ROOT)/Archives/px4fmu-v2.export \
	$(PX4_ROOT)/Archives/px4fmu-v3.export \
	$(PX4_ROOT)/Archives/px4fmu-v4.export \
	$(PX4_ROOT)/Archives/px4fmu-v4pro.export \
	$(PX4_ROOT)/Archives/px4io-v1.export \
	$(PX4_ROOT)/Archives/px4io-v2.export

$(PX4_ROOT)/Archives/px4fmu-v1.export:
	$(v) $(PX4_MAKE_ARCHIVES) BOARDS="px4fmu-v1"

$(PX4_ROOT)/Archives/px4fmu-v2.export:
	$(v) $(PX4_MAKE_ARCHIVES) BOARDS="px4fmu-v2"

$(PX4_ROOT)/Archives/px4fmu-v3.export:
	$(v) $(PX4_MAKE_ARCHIVES) BOARDS="px4fmu-v3"

$(PX4_ROOT)/Archives/px4fmu-v4.export:
	$(v) $(PX4_MAKE_ARCHIVES) BOARDS="px4fmu-v4"

$(PX4_ROOT)/Archives/px4fmu-v4pro.export:
	$(v) $(PX4_MAKE_ARCHIVES) BOARDS="px4fmu-v4pro"

$(PX4_ROOT)/Archives/px4io-v1.export:
	$(v) $(PX4_MAKE_ARCHIVES) BOARDS="px4io-v1"

$(PX4_ROOT)/Archives/px4io-v2.export:
	$(v) $(PX4_MAKE_ARCHIVES) BOARDS="px4io-v2"

px4-archives:
	$(v) $(PX4_MAKE_ARCHIVES) BOARDS="px4io-v1 px4io-v2 px4fmu-v1 px4fmu-v2 px4fmu-v3 px4fmu-v4 px4fmu-v4pro"
