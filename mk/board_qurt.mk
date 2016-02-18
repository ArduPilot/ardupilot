# extra rules for qualcomm flight build targetting the DSPs

# host name of flight board for "qurt_send" target
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

LINUX_QAIC		 = $(HEXAGON_SDK_ROOT)/tools/qaic/Linux/qaic

QURT_ARM_GCC_ROOT	 = $(HEXAGON_SDK_ROOT)/gcc-linaro-arm-linux-gnueabihf-4.8-2013.08_linux
QURT_ARM_CC		 = $(QURT_ARM_GCC_ROOT)/bin/arm-linux-gnueabihf-gcc
QURT_ARM_CXX		 = $(QURT_ARM_GCC_ROOT)/bin/arm-linux-gnueabihf-g++

HEXAGON_LINK		 = $(QURT_CC)
LIBSTDCXX		 = $(HEXAGON_TOOLS_ROOT)/Tools/target/hexagon/lib/v5/G0/pic/libstdc++.a

LIBQCC			 = $(HEXAGON_TOOLS_ROOT)/Tools/target/hexagon/lib/v5/G0/pic/libqcc.a

QFLIGHT_CC	= $(QURT_ARM_CC)
QFLIGHT_CXX	= $(QURT_ARM_CXX)

OPT		= -O3

include $(MK_DIR)/find_tools.mk

DEFINES        +=   -DSKETCH=\"$(SKETCH)\" -DSKETCHNAME="\"$(SKETCH)\"" -DSKETCHBOOK="\"$(SKETCHBOOK)\"" -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)
DEFINES        +=   $(EXTRAFLAGS)
DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD) -DCONFIG_HAL_BOARD_SUBTYPE=$(HAL_BOARD_SUBTYPE) -DAP_MAIN=ArduPilot_main

DEPFLAGS        =   -MD -MP -MT $@

CXXOPTS         =   -ffunction-sections -fdata-sections -fno-exceptions -fsigned-char $(DSP_CFLAGS) $(WARNFLAGSCXX)
COPTS           =   -ffunction-sections -fdata-sections -fsigned-char $(DSP_CFLAGS)

# DSP build flags
DSP_INC=-I$(HEXAGON_SDK_ROOT)/gnu/hexagon/include -I$(HEXAGON_FC_ADDON)/hexagon/inc -I$(HEXAGON_FC_ADDON)/hexagon/inc/dspal/sys -I$(HEXAGON_FC_ADDON)/hexagon/inc/dspal/sys/sys -I$(HEXAGON_FC_ADDON)/hexagon/inc/dspal/include -I$(HEXAGON_SDK_ROOT)/lib/common/qurt/ADSPv5MP/include -I$(HEXAGON_SDK_ROOT)/lib/common/remote/ship/hexagon_Debug_dynamic -I$(HEXAGON_SDK_ROOT)/inc/stddef -I$(HAL_QURT_BUILD)
DSP_WARN=-Wno-unused-parameter -Wno-gnu-variable-sized-type-not-at-end -Wno-gnu-designator -Wno-absolute-value
DSP_FLAGS=-mv5 -G0 -g $(OPT) $(DSP_WARN) -fno-exceptions -fno-strict-aliasing -fno-zero-initialized-in-bss -fdata-sections -fpic -D__V_DYNAMIC__ -D_DEBUG $(DSP_INC) -D_PID_T -D_UID_T -D_TIMER_T -D_HAS_C9X

DSP_LINK_FLAGS = -mv5 -G0 -fpic -shared -Wl,-Bsymbolic -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free -Wl,--wrap=realloc -Wl,--wrap=memalign -Wl,--wrap=__stack_chk_fail -Wl,-soname=lib$(SKETCHNAME)_skel.so 

# Add missing parts from libc and libstdc++
MISSING_TOOLCHAIN_FLAGS += -DHAVE_STD_NULLPTR_T=0 -DHAVE_STD_MOVE=0 -DHAVE_STD_REMOVE_REFERENCE=0 -DHAVE_TYPE_TRAITS_H=0
MISSING_TOOLCHAIN_FLAGS += -I$(SKETCHBOOK)/libraries/AP_Common/missing

CXXFLAGS       +=   -std=gnu++11 $(WARNFLAGS) $(WARNFLAGSCXX) $(DEPFLAGS) $(MISSING_TOOLCHAIN_FLAGS) $(CXXOPTS) $(DEFINES) $(DSP_FLAGS)
CFLAGS         +=   $(WARNFLAGS) $(DEPFLAGS) $(COPTS) $(DEFINES) $(DSP_FLAGS)

ARM_INC=-I$(HEXAGON_SDK_ROOT)/lib/common/remote/ship/UbuntuARM_Debug -I$(SKETCHBOOK)/libraries/
ARM_WARN=-Wno-unused-parameter
ARM_CFLAGS=$(ARM_WARN) -fPIC -mword-relocations -mthumb-interwork -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 -march=armv7-a -g -O0 -fno-strict-aliasing -DARM_ARCH_7A -DUSE_SYSLOG $(ARM_INC) -I$(HEXAGON_SDK_ROOT)/inc/stddef -DCONFIG_HAL_BOARD=HAL_BOARD_LINUX -DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

ifeq ($(VERBOSE),)
v = @
else
v =
endif

# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS)

# The ELF file
SKETCHELF		=	$(BUILDROOT)/$(SKETCH).elf
LIBSKETCH_SKEL_SO	=	$(BUILDROOT)/libardupilot_skel.so
BUILDELF                =       $(notdir $(SKETCHELF))

# All of the objects that may be built
ALLOBJS			=	$(SKETCHOBJS) $(LIBOBJS)

# All of the dependency files that may be generated
ALLDEPS			=	$(ALLOBJS:%.o=%.d)

HAL_QURT_SRC	 = $(SKETCHBOOK)/libraries/AP_HAL_QURT
HAL_QURT_BUILD	 = $(BUILDROOT)/libraries/AP_HAL_QURT

# main app sources
MAINAPP_SRC		=  $(wildcard $(HAL_QURT_SRC)/mainapp/*.cpp) $(wildcard $(SKETCHBOOK)/libraries/AP_HAL/utility/*.cpp)
MAINAPPOBJS		=  $(subst $(SKETCHBOOK)/,$(BUILDROOT)/,$(MAINAPP_SRC:%.cpp=%.ao)) $(HAL_QURT_BUILD)/ardupilot_stub.ao


.SUFFIXES: .ao .o .c .so .do

# build arm object
$(BUILDROOT)/libraries/%.ao: $(SKETCHBOOK)/libraries/%.c
	$(RULEHDR)
	$(v)$(QURT_ARM_CC) -c $(ARM_CFLAGS) -I$(HAL_QURT_BUILD) -o $@ $< -DSKETCHNAME="\"$(SKETCH)\""

# build arm C++ object
$(BUILDROOT)/libraries/%.ao: $(SKETCHBOOK)/libraries/%.cpp $(HAL_QURT_BUILD)/qurt_dsp.h
	$(RULEHDR)
	$(v)$(QURT_ARM_CXX) -std=gnu++11 -c $(ARM_CFLAGS) -I$(HAL_QURT_BUILD) -o $@ $< -DSKETCHNAME="\"$(SKETCH)\""

$(BUILDROOT)/libraries/%.ao: $(BUILDROOT)/libraries/%.c
	$(RULEHDR)
	$(v)$(QURT_ARM_CC) -c $(ARM_CFLAGS) -I$(HAL_QURT_BUILD) -o $@ $< -DSKETCHNAME="\"$(SKETCH)\""

# build DSP object from C file
$(BUILDROOT)/libraries/%.do: $(BUILDROOT)/libraries/%.c
	$(RULEHDR)
	$(v)$(QURT_CC) $(CFLAGS) -I$(HEXAGON_SDK_ROOT)/inc/stddef -DSKEL_INVOKE=$(SKETCH)_skel_invoke -c -o $@ $<

################################################################################
# Targets
#
all: $(SKETCHELF)

print-%:
	echo "$*=$($*)"

################################################################################
# Rules
#

SKETCH_INCLUDES	=	$(SKETCHLIBINCLUDES)
SLIB_INCLUDES	=	$(SKETCHLIBINCLUDES)

# fetch dependency info from a previous build if any of it exists
-include $(ALLDEPS)

GENERATE_DSP_C = $(HAL_QURT_BUILD)/ardupilot_skel.c
GENERATE_ARM_C = $(HAL_QURT_BUILD)/ardupilot_stub.c
GENERATE_TARGETS = $(HAL_QURT_BUILD)/qurt_dsp.h $(GENERATE_DSP_C) $(GENERATE_ARM_C)

$(GENERATE_TARGETS): $(HAL_QURT_SRC)/qurt_dsp.idl
	@echo Generating DSP IDL sources for $^
	$(v)mkdir -p $(HAL_QURT_BUILD)
	$(v)$(LINUX_QAIC) -mdll -o $(HAL_QURT_BUILD) -I$(HAL_QURT_SRC) -I$(HEXAGON_SDK_ROOT)/inc/stddef $<

# Link the final object
$(SKETCHELF): $(MAINAPPOBJS) $(LIBSKETCH_SKEL_SO) $(GENERATE_TARGETS)
	@echo "Building $(SKETCHELF)"
	$(RULEHDR)
	$(v)$(QFLIGHT_CXX) -march=armv7-a -mthumb-interwork -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 -Wl,-unresolved-symbols=ignore-in-shared-libs -Wl,-ldl -o $@ -Wl,"-(" $(MAINAPPOBJS) -L$(HEXAGON_SDK_ROOT)/lib/common/remote/ship/UbuntuARM_ReleaseG/ -ladsprpc $(HEXAGON_SDK_ROOT)/lib/common/rpcmem/UbuntuARM_ReleaseG/ship/rpcmem.a $(HEXAGON_SDK_ROOT)/lib/common/adspmsgd/ship/UbuntuARM_ReleaseG/adspmsgd.a -Wl,"-)" -lm -lc -lsupc++ -lgcc_eh -lgcc -lpthread -lstdc++
	$(v)cp $(SKETCHELF) .
	@echo "Firmware is in $(BUILDELF)"

DSPSKELOBJS=$(HAL_QURT_BUILD)/ardupilot_skel.do

# this is quite bizarre. The provided libc mixes PIC and non-PIC code,
# which seems to confuse the linker. In order to link to some critical
# system objects we need to extract them from libc into a new library
LIBC_SRC=$(HEXAGON_TOOLS_ROOT)/Tools/target/hexagon/lib/v5/G0/pic/libc.a
LIBC_EXTRACTDIR=$(BUILDROOT)/libc_extracted
LIBC_OBJECTS=xfdtest.o xdtest.o feclearexcept.o fegetenv.o fesetenv.o
LIBC_EXTRACTED=$(LIBC_EXTRACTDIR)/libc_extracted.a

$(LIBC_EXTRACTED): $(LIBC_SRC)
	$(v)echo Extracting libc objects into $@
	$(v)mkdir -p $(LIBC_EXTRACTDIR)
	$(v)(cd $(LIBC_EXTRACTDIR) && ar x $(LIBC_SRC) $(LIBC_OBJECTS) && ar crs libc_extracted.a $(LIBC_OBJECTS))

$(LIBSKETCH_SKEL_SO): $(DSPSKELOBJS) $(ALLOBJS) $(GENERATE_TARGETS) $(LIBC_EXTRACTED)
	@echo Linking $@
	$(v)$(HEXAGON_LINK) -L$(HEXAGON_FC_ADDON)/hexagon/libs -lmpu9x50 -lbmp280 -lcsr_gps -g -mv5 -mG0lib -G0 -fpic -shared -Wl,-Bsymbolic -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free -Wl,--wrap=realloc -Wl,--wrap=memalign -Wl,--wrap=__stack_chk_fail -lc -lm -Wl,-soname=libqflight_skel.so  -o $@ -Wl,--whole-archive $(DSPSKELOBJS) $(ALLOBJS) $(LIBC_EXTRACTED) -Wl,--no-whole-archive -Wl,$(LIBSTDCXX) -Wl,$(LIBQCC) -Wl,$(HEXAGON_TOOLS_ROOT)/Tools/target/hexagon/lib/v5/G0/libdl.a 
	$(v)cp $(LIBSKETCH_SKEL_SO) .
	@echo "DSP skel firmware is in $(BUILDELF)"

include $(MK_DIR)/build_rules.mk

qurt_send: qurt
	rsync -av $(LIBSKETCH_SKEL_SO) root@$(FLIGHT_BOARD):/usr/share/data/adsp
	rsync -av $(SKETCHELF) root@$(FLIGHT_BOARD):

