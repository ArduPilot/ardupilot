###############################################################################
# File name: board_yuneec.mk
#    Author: Maelok
###############################################################################
###############################################################################
# Specify the MCU info for libopencm3

TOOLCHAIN   := ARM
MCU 		:= STM32F372CC
FAMILY 		:= cortex-m4
F_CPU 		:= 72000000L
LIBNAME		:= opencm3_stm32f3

# Put the 'libopencm3' within the same folder with 'ardupilot'
OPENCM3_DIR = $(SKETCHBOOK)/../libopencm3
LDSCRIPT 	= $(SKETCHBOOK)/libraries/AP_HAL_YUNEEC/utility/ld/$(MCU).ld

###############################################################################
# Source files

LDSCRIPT	?= $(SKETCH).ld

ifeq ($(strip $(OPENCM3_DIR)),)
# user has not specified the library path, so we try to detect it

# where we search for the library
LIBPATHS := ./libopencm3 ../../../../libopencm3 ../../../../../libopencm3

OPENCM3_DIR := $(wildcard $(LIBPATHS:=/locm3.sublime-project))
OPENCM3_DIR := $(firstword $(dir $(OPENCM3_DIR)))

ifeq ($(strip $(OPENCM3_DIR)),)
$(warning Cannot find libopencm3 library in the standard search paths.)
$(error Please specify it through OPENCM3_DIR variable!)
endif
endif

ifeq ($(V),1)
$(info Using $(OPENCM3_DIR) path to library)
endif

OPENCM3_INCLUDE_DIR	= $(OPENCM3_DIR)/include
OPENCM3_LIB_DIR		= $(OPENCM3_DIR)/lib
OPENCM3_SCRIPT_DIR	= $(OPENCM3_DIR)/scripts

###############################################################################
# Tool options

include $(MK_DIR)/find_tools.mk

DEFINES         =   -DF_CPU=$(F_CPU) -DSTM32F3
DEFINES        +=   -DSKETCH=\"$(SKETCH)\" -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)
DEFINES        +=   $(EXTRAFLAGS) # from user config.mk
DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD)
WARNFLAGS       =   -Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wno-psabi 
WARNFLAGS      +=   -Wwrite-strings -Wformat=2 #-Wundef -Wredundant-decls
WARNFLAGSCXX    =   -Wno-reorder -Wredundant-decls # -Weffc++
WARNFLAGSC      =   -Wimplicit-function-declaration  -Wmissing-prototypes -Wstrict-prototypes

DEPFLAGS        =   -MD -MT $@

CXXOPTS         =   -ffunction-sections -fdata-sections -fno-exceptions -fsigned-char -fno-rtti -fno-common
COPTS           =   -ffunction-sections -fdata-sections -fsigned-char -fno-common
ASOPTS          =   -x assembler-with-cpp
LISTOPTS        =   -adhlns=$(@:.o=.lst)

NATIVE_CPUFLAGS     = -D_GNU_SOURCE
NATIVE_CPULDFLAGS   = -g
NATIVE_OPTFLAGS     = -O0 -g

ARM_CPUFLAGS        = -mthumb -mcpu=$(FAMILY) -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARM_CPULDFLAGS      = -L$(OPENCM3_LIB_DIR) -T$(LDSCRIPT)
ARM_OPTFLAGS        = -Os -g

CPUFLAGS= $($(TOOLCHAIN)_CPUFLAGS)
CPULDFLAGS= $($(TOOLCHAIN)_CPULDFLAGS)
OPTFLAGS= $($(TOOLCHAIN)_OPTFLAGS)

CXXFLAGS        =   $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS) -Wl,--gc-sections  
CXXFLAGS       +=   $(WARNFLAGS) $(WARNFLAGSCXX) $(DEPFLAGS) $(CXXOPTS)
CFLAGS          =   $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS) -Wl,--gc-sections  
CFLAGS         +=   $(WARNFLAGS) $(WARNFLAGSC) $(DEPFLAGS) $(COPTS)
ASFLAGS         =   $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(DEPFLAGS)
ASFLAGS        +=   $(ASOPTS)
LDFLAGS         =   $(CPUFLAGS) $(CPULDFLAGS) $(OPTFLAGS) $(WARNFLAGS)
LDFLAGS        +=   --static -nostartfiles -Wl,-Map=$(SKETCHMAP) -Wl,--gc-sections  

LDLIBS		    += -lm 
LDLIBS		    += -l$(LIBNAME)
LDLIBS			+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

ifeq ($(VERBOSE),)
v = @
else
v =
endif

COREOBJS = $(OPENCM3_LIB_DIR)/lib$(LIBNAME).a
COREINCLUDES = -I$(OPENCM3_INCLUDE_DIR)

# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS)

################################################################################
# Built products

# The ELF file
SKETCHELF		=	$(BUILDROOT)/$(SKETCH).elf

# HEX file
SKETCHHEX		=	$(BUILDROOT)/$(SKETCH).hex

# BIN file
SKETCHBIN		=	$(BUILDROOT)/$(SKETCH).bin

# EEP file
SKETCHEEP		=	$(BUILDROOT)/$(SKETCH).eep

# Map file
SKETCHMAP		=	$(BUILDROOT)/$(SKETCH).map

# All of the objects that may be built
ALLOBJS			=	$(SKETCHOBJS) $(LIBOBJS)

# All of the dependency files that may be generated
ALLDEPS			=	$(ALLOBJS:%.o=%.d)

################################################################################
# Targets

all: $(SKETCHELF) $(SKETCHEEP) $(SKETCHHEX) $(SKETCHBIN)

print-%:
	echo "$*=$($*)"

################################################################################
# Rules

# fetch dependency info from a previous build if any of it exists
-include $(ALLDEPS)

# Link the final object
$(SKETCHELF):	$(SKETCHOBJS) $(LIBOBJS) $(LDSCRIPT) $(COREOBJS)
	$(RULEHDR)
	$(v)$(LD) $(LDFLAGS) $(SKETCHOBJS) $(LIBOBJS) $(LDLIBS) -o $@

# Create the hex file
$(SKETCHHEX):	$(SKETCHELF)
	$(RULEHDR)
	$(v)$(OBJCOPY) -O ihex -R .eeprom $< $@

# Create the bin file
$(SKETCHBIN):	$(SKETCHELF)
	$(RULEHDR)
	$(v)$(OBJCOPY) -v -O binary $< $@

# Create the eep file
$(SKETCHEEP):	$(SKETCHELF)
	$(RULEHDR)
	$(v)$(OBJCOPY) -O ihex -j.eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $@

#
# Build sketch objects
#
SKETCH_INCLUDES	=	$(SKETCHLIBINCLUDES) $(COREINCLUDES)

$(BUILDROOT)/%.o: $(BUILDROOT)/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< -I$(SRCROOT) $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(SRCROOT)/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(SRCROOT)/%.c
	$(RULEHDR)
	$(v)$(CC) $(CFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

$(BUILDROOT)/%.o: $(SRCROOT)/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< $(SKETCH_INCLUDES)

#
# Build library objects from sources in the sketchbook
#
SLIB_INCLUDES	=	-I$(dir $<)/utility $(SKETCHLIBINCLUDES) $(COREINCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.c
	$(RULEHDR)
	$(v)$(CC) $(CFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

###############################################################################
# JLink as the programmer

OUTPUT_BINARY_DIRECTORY   := $(BUILDROOT)
JLINK                     := $(shell which JLinkExe)
JLINKFLAGS          	  := -If SWD -Device $(MCU) -Speed 1000 -CommanderScript

# Check if there is JLinkExe
ifeq ($(JLINK),)
$(error Cannot find official JLink software, Please install JLink for Linux(http://www.segger.com/j-link-software.html) and export PATH for JLinkExe, then try again.)
endif

## Program device
.PHONY: jlink-flash jlink-erase $(MCU)-flash.jlink erase-all.jlink

jlink-flash: $(MCU)-flash.jlink
	$(JLINK) $(JLINKFLAGS) $(OUTPUT_BINARY_DIRECTORY)/$(MCU)_flash.jlink

jlink-erase: erase-all.jlink
	$(JLINK) $(JLINKFLAGS) $(OUTPUT_BINARY_DIRECTORY)/erase_all.jlink

$(MCU)-flash.jlink: 
	@echo > $(OUTPUT_BINARY_DIRECTORY)/$(MCU)_flash.jlink
	@echo "r" >> $(OUTPUT_BINARY_DIRECTORY)/$(MCU)_flash.jlink
	@echo "h" >> $(OUTPUT_BINARY_DIRECTORY)/$(MCU)_flash.jlink
	@echo "loadbin $(SKETCHBIN),0x08000000" >> $(OUTPUT_BINARY_DIRECTORY)/$(MCU)_flash.jlink
	@echo "r" >> $(OUTPUT_BINARY_DIRECTORY)/$(MCU)_flash.jlink
	@echo "g" >> $(OUTPUT_BINARY_DIRECTORY)/$(MCU)_flash.jlink

erase-all.jlink:
	@echo > $(OUTPUT_BINARY_DIRECTORY)/erase_all.jlink
	@echo "r" >> $(OUTPUT_BINARY_DIRECTORY)/erase_all.jlink
	@echo "h" >> $(OUTPUT_BINARY_DIRECTORY)/erase_all.jlink
	@echo "erase" >> $(OUTPUT_BINARY_DIRECTORY)/erase_all.jlink
	@echo "r" >> $(OUTPUT_BINARY_DIRECTORY)/erase_all.jlink

###############################################################################
