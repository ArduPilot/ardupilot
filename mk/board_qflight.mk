# extra rules for qualcomm flight linux build

FLIGHT_BOARD		 ?= flight4

HEXAGON_TOOLS_ROOT	 ?= $(HOME)/Qualcomm/HEXAGON_Tools/7.2.11
HEXAGON_SDK_ROOT	 ?= $(HOME)/Qualcomm/Hexagon_SDK/2.0
HEXAGON_FC_ADDON	 ?= $(HOME)/Qualcomm/HexagonFCAddon/flight_controller
V_ARCH			 = v5
CROSSDEV		 = hexagon-
HEXAGON_BIN		 = $(addsuffix /Tools/bin,$(HEXAGON_TOOLS_ROOT))
HEXAGON_CLANG_BIN	 = $(addsuffix /Tools/bin,$(HEXAGON_TOOLS_ROOT))

QURT_CC			 = $(HEXAGON_CLANG_BIN)/$(CROSSDEV)clang
QURT_CXX		 = $(HEXAGON_CLANG_BIN)/$(CROSSDEV)clang++
HEXAGON_GCC_BIN		 = $(HEXAGON_TOOLS_ROOT)/gnu/bin

LINUX_QAIC		 = $(HEXAGON_SDK_ROOT)/tools/qaic/Linux/qaic

QURT_ARM_GCC_ROOT	 = $(HEXAGON_SDK_ROOT)/gcc-linaro-arm-linux-gnueabihf-4.8-2013.08_linux
QURT_ARM_CC		 = $(QURT_ARM_GCC_ROOT)/bin/arm-linux-gnueabihf-gcc
QURT_ARM_CXX		 = $(QURT_ARM_GCC_ROOT)/bin/arm-linux-gnueabihf-g++

HEXAGON_LINK		 = $(QURT_CC)
LIBSTDCXX		 = $(HEXAGON_TOOLS_ROOT)/Tools/target/hexagon/lib/v5/G0/pic/libstdc++.a

QFLIGHT_CC	= $(QURT_ARM_CC)
QFLIGHT_CXX	= $(QURT_ARM_CXX)
QFLIGHT_LD	= $(QURT_ARM_CXX)

QFLIGHT_SRC	 = $(SKETCHBOOK)/libraries/AP_HAL_Linux/qflight
QFLIGHT_BUILD	 = $(BUILDROOT)/libraries/AP_HAL_Linux/qflight

GENERATE_DSP_C = $(QFLIGHT_BUILD)/qflight_skel.c
GENERATE_ARM_C = $(QFLIGHT_BUILD)/qflight_stub.c
GENERATE_TARGETS = $(GENERATE_DSP_C) $(GENERATE_ARM_C) $(QFLIGHT_BUILD)/qflight_dsp.h

LIBOBJS += $(QFLIGHT_BUILD)/qflight_stub.o

# Add missing parts from libc and libstdc++
MISSING_TOOLCHAIN_FLAGS += -DHAVE_STD_NULLPTR_T=0 -DHAVE_STD_MOVE=0 -DHAVE_STD_REMOVE_REFERENCE=0 -DHAVE_TYPE_TRAITS_H=0 -DHAVE_BYTESWAP_H=0 -DHAVE_ENDIAN_H=1

# Hardcoded libraries/AP_Common/missing/cmath defines in "make" to retain the current behavior
EXTRAFLAGS += -DHAVE_CMATH_ISFINITE -DNEED_CMATH_ISFINITE_STD_NAMESPACE

EXTRAFLAGS += -D__STDC_FORMAT_MACROS

# DSP build flags
DSP_INC=$(MISSING_TOOLCHAIN_FLAGS) $(SHARED_INC) -I$(HEXAGON_FC_ADDON)/hexagon/inc -I$(HEXAGON_FC_ADDON)/hexagon/inc/dspal/sys -I$(HEXAGON_FC_ADDON)/hexagon/inc/dspal/sys/sys -I$(HEXAGON_FC_ADDON)/hexagon/inc/dspal/sys/machine -I$(HEXAGON_FC_ADDON)/hexagon/inc/dspal/include -I$(HEXAGON_SDK_ROOT)/lib/common/qurt/ADSPv5MP/include -I$(HEXAGON_SDK_ROOT)/lib/common/remote/ship/hexagon_ReleaseG -I$(QFLIGHT_BUILD) -I$(HEXAGON_SDK_ROOT)/inc/stddef -I$(SKETCHBOOK)/libraries
DSP_FLAGS=-mv5 -G0 -g -O3 -fno-exceptions -fno-strict-aliasing -fno-zero-initialized-in-bss -fdata-sections -fpic -D__V_DYNAMIC__  $(DSP_INC) -D_PID_T -D_UID_T -D_TIMER_T -D_HAS_C9X

$(GENERATE_TARGETS): $(QFLIGHT_SRC)/qflight_dsp.idl
	@echo Generating DSP IDL sources for $^
	$(v)mkdir -p $(QFLIGHT_BUILD)
	$(v)$(LINUX_QAIC) -mdll -o $(QFLIGHT_BUILD) -I$(QFLIGHT_SRC) -I$(HEXAGON_SDK_ROOT)/inc/stddef $<

DSP_LINK_FLAGS = -mv5 -O3 -G0 -fpic -shared -Wl,-Bsymbolic -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free -Wl,--wrap=realloc -Wl,--wrap=memalign -Wl,--wrap=__stack_chk_fail -Wl,-soname=libqflight_skel.so 

ARM_INC=-I$(HEXAGON_SDK_ROOT)/lib/common/remote/ship/UbuntuARM_ReleaseG -I$(HEXAGON_SDK_ROOT)/lib/common/rpcmem/UbuntuARM_ReleaseG/ship -I$(HEXAGON_SDK_ROOT)/inc/stddef -I$(BUILDROOT)/libraries -I$(HEXAGON_SDK_ROOT)/inc/stddef
ARM_CFLAGS=-fPIC -mword-relocations -mthumb-interwork -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 -march=armv7-a -g -O3 -fno-strict-aliasing -DARM_ARCH_7A -DUSE_SYSLOG $(ARM_INC)

ARM_LINK_LIBS1  = -L$(HEXAGON_SDK_ROOT)/lib/common/remote/ship/UbuntuARM_ReleaseG -ladsprpc $(HEXAGON_SDK_ROOT)/lib/common/rpcmem/UbuntuARM_ReleaseG/ship/rpcmem.a $(HEXAGON_SDK_ROOT)/lib/common/adspmsgd/ship/UbuntuARM_ReleaseG/adspmsgd.a
ARM_LINK_LIBS2  = -lm -lc -lsupc++ -lgcc_eh -lgcc
ARM_LINK_FLAGS = -march=armv7-a -mthumb-interwork -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 -Wl,-unresolved-symbols=ignore-in-shared-libs -Wl,-ldl

LIBS += $(ARM_LINK_FLAGS) $(ARM_LINK_LIBS1) $(ARM_LINK_LIBS2)

CXXOPTS += $(ARM_CFLAGS)
COPTS += $(ARM_CFLAGS)

LIBSKETCH_SKEL_SO	=	$(BUILDROOT)/libqflight_skel.so

.SUFFIXES: .o .c .cpp .so .do

# build DSP object from C file
$(BUILDROOT)/libraries/%.do: $(SKETCHBOOK)/libraries/%.c $(MAVLINK_HEADERS)
	$(RULEHDR)
	$(v)$(QURT_CC) $(DSP_FLAGS) -c -o $@ $<

$(BUILDROOT)/libraries/AP_HAL_Linux/qflight/%.o: $(BUILDROOT)/libraries/AP_HAL_Linux/qflight/%.c
	$(RULEHDR)
	$(v)$(CC) -c -fPIC -mword-relocations -mthumb-interwork -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 -march=armv7-a -g -O3 -fno-strict-aliasing -DARM_ARCH_7A -DUSE_SYSLOG -I$(HEXAGON_SDK_ROOT)/inc/stddef -I$(HEXAGON_SDK_ROOT)/lib/common/adspmsgd/ship/UbuntuARM_ReleaseG -I$(HEXAGON_SDK_ROOT)/lib/common/rpcmem/UbuntuARM_ReleaseG/ship -I$(HEXAGON_SDK_ROOT)/lib/common/rpcmem/UbuntuARM_ReleaseG/ship -I$(HEXAGON_SDK_ROOT)/lib/common/remote/ship/UbuntuARM_ReleaseG -std=gnu99 -o $@ $<

$(BUILDROOT)/libraries/%.do: $(SKETCHBOOK)/libraries/%.cpp
	$(RULEHDR)
	$(v)$(QURT_CXX) -std=gnu++11 $(DSP_FLAGS) -DSKEL_INVOKE=$(SKETCH)_skel_invoke -c -o $@ $<

.c.do:
	$(RULEHDR)
	$(v)$(QURT_CC) $(DSP_FLAGS) -DSKEL_INVOKE=$(SKETCH)_skel_invoke -c -o $@ $<

.cpp.do:
	$(RULEHDR)
	$(v)$(QURT_CXX) -std=gnu++11 $(DSP_FLAGS) -DSKEL_INVOKE=$(SKETCH)_skel_invoke -c -o $@ $<

DSPSKELOBJS=$(QFLIGHT_BUILD)/qflight_skel.do $(QFLIGHT_BUILD)/dsp_functions.do $(BUILDROOT)/libraries/AP_HAL/utility/RingBuffer.do

$(SKETCHELF): $(LIBSKETCH_SKEL_SO)

$(LIBSKETCH_SKEL_SO): $(DSPSKELOBJS)
	@echo Linking $@
	$(v)$(HEXAGON_LINK) -L$(HEXAGON_FC_ADDON)/hexagon/libs -lmpu9x50 -lbmp280 -g -mv5 -mG0lib -G0 -fpic -shared -Wl,-Bsymbolic -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free -Wl,--wrap=realloc -Wl,--wrap=memalign -Wl,--wrap=__stack_chk_fail -lc -lm -Wl,-soname=libqflight_skel.so  -o $@ -Wl,--whole-archive  $^ -Wl,--no-whole-archive -Wl,$(LIBSTDCXX)
	$(v)cp $(LIBSKETCH_SKEL_SO) .
	@echo "DSP skel firmware is in libqflight_skel.so"

# Link the final object
$(SKETCHELF): $(SKETCHOBJS) $(LIBOBJS)
	@echo "Building $(SKETCHELF)"
	$(RULEHDR)
	$(v)$(QFLIGHT_CC) -march=armv7-a -mthumb-interwork -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 -Wl,-unresolved-symbols=ignore-in-shared-libs -Wl,-ldl -o $@ -Wl,"-(" $(SKETCHOBJS) $(LIBOBJS) -L$(HEXAGON_SDK_ROOT)/lib/common/remote/ship/UbuntuARM_ReleaseG/ -ladsprpc $(HEXAGON_SDK_ROOT)/lib/common/rpcmem/UbuntuARM_ReleaseG/ship/rpcmem.a $(HEXAGON_SDK_ROOT)/lib/common/adspmsgd/ship/UbuntuARM_ReleaseG/adspmsgd.a -Wl,"-)" -lm -lc -lsupc++ -lgcc_eh -lgcc -lpthread -lstdc++
#$(v)$(LD) $(LDFLAGS) -o $@ $(SKETCHOBJS) $(LIBOBJS) $(LIBS)
	$(v)cp $(SKETCHELF) .
	@echo "Firmware is in $(BUILDELF)"

qflight_send: qflight
	rsync -av $(LIBSKETCH_SKEL_SO) root@$(FLIGHT_BOARD):/usr/share/data/adsp
	rsync -av $(SKETCHELF) root@$(FLIGHT_BOARD):
