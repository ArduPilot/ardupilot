###############################################################################
# File name: board_yuneec.mk
#    Author: Maelok
###############################################################################
###############################################################################
# Specify the MCU info for libopencm3

TOOLCHAIN   := ARM
MCU 		:= STM32F372RC
FAMILY 		:= cortex-m4
F_CPU 		:= 72000000L
HSE_VALUE	:= 16000000L
LIBNAME		:= stm32f37x
TypeOfMCU	:= STM32F37X

# Put the 'libs' within the same folder with 'ardupilot'
LIBDIR 		= $(SKETCHBOOK)/../libraries
STMLIBDIR	= $(LIBDIR)/STM32_USB-FS-Device_Lib_V4.0.0/Libraries
LDSCRIPT 	= $(LIBDIR)/STM32F3_FLASH.ld
STARTUP 	= $(LIBDIR)/startup_stm32f37x.s

###############################################################################
# Source files

COREINCLUDES  = -I$(STMLIBDIR)/CMSIS/Include
COREINCLUDES += -I$(STMLIBDIR)/CMSIS/Device/ST/STM32F37x/Include
COREINCLUDES += -I$(STMLIBDIR)/CMSIS/Device/ST/STM32F37x/Source/Templates
COREINCLUDES += -I$(STMLIBDIR)/STM32F37x_StdPeriph_Driver/inc
COREINCLUDES += -I$(STMLIBDIR)/STM32_USB-FS-Device_Driver/inc
COREINCLUDES += -I$(LIBDIR)/STM32F37x_I2C_CPAL_Driver/inc

###############################################################################
# Tool options

include $(MK_DIR)/find_tools.mk

DEFINES         =   -DF_CPU=$(F_CPU) -DHSE_VALUE=$(HSE_VALUE) -D$(TypeOfMCU) -DVECT_TAB_FLASH 
DEFINES        +=   -DSKETCH=\"$(SKETCH)\" -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)
DEFINES        +=   $(EXTRAFLAGS) # from user config.mk
DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD)
WARNFLAGS       =   -Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wno-psabi 
WARNFLAGS      +=   -Wwrite-strings -Wformat=2
WARNFLAGSCXX    =   -Wno-reorder -Wredundant-decls
WARNFLAGSC      =   -Wimplicit-function-declaration  -Wmissing-prototypes -Wstrict-prototypes

DEPFLAGS        =   -MD -MT $@
DEPFLAGS		=	-include $(LIBDIR)/stm32f37x_conf.h

CXXOPTS         =   -ffunction-sections -fdata-sections -fno-exceptions -fsigned-char -fno-rtti -fno-common -Wl,--gc-sections
COPTS           =   -ffunction-sections -fdata-sections -fsigned-char -fno-common  -Wl,--gc-sections
ASOPTS          =   -x assembler-with-cpp
LISTOPTS        =   -adhlns=$(@:.o=.lst)

NATIVE_CPUFLAGS     = -D_GNU_SOURCE
NATIVE_CPULDFLAGS   = -g
NATIVE_OPTFLAGS     = -O0 -g

ARM_CPUFLAGS        = -mlittle-endian -mthumb -mcpu=$(FAMILY) -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARM_CPULDFLAGS      = -L$(LIBDIR) -T$(LDSCRIPT)
ARM_OPTFLAGS        = -g -Os

CPUFLAGS			= $($(TOOLCHAIN)_CPUFLAGS)
CPULDFLAGS			= $($(TOOLCHAIN)_CPULDFLAGS)
OPTFLAGS			= $($(TOOLCHAIN)_OPTFLAGS)

CXXFLAGS       +=   $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS) $(COREINCLUDES) -std=gnu++11
CXXFLAGS       +=   $(WARNFLAGS) $(WARNFLAGSCXX) $(DEPFLAGS) $(CXXOPTS)
CFLAGS         +=   $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS) $(COREINCLUDES)
CFLAGS         +=   $(WARNFLAGS) $(WARNFLAGSC) $(DEPFLAGS) $(COPTS)
ASFLAGS        +=   $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS)  $(COREINCLUDES)
ASFLAGS        +=   $(ASOPTS) $(DEPFLAGS)
LDFLAGS        +=   $(CPUFLAGS) $(DEFINES) $(CPULDFLAGS) $(OPTFLAGS) $(COREINCLUDES)
LDFLAGS        +=	-Wl,--gc-sections -Wl,-Map=$(SKETCHMAP) -Wl,--no-whole-archive

LDLIBS		   +=	-lm 
LDLIBS		   +=	-l$(LIBNAME)
LDLIBS		   +=	-Wl,--start-group -lc -lgcc -lstdc++ -lnosys -Wl,--end-group

ifeq ($(VERBOSE),)
v = @
else
v =
endif

# Library object files
#COREOBJS		:=	$(STMLIBDIR)/CMSIS/Device/ST/STM32F37x/Source/Templates/*.o $(STMLIBDIR)/STM32F37x_StdPeriph_Driver/src/*.o #$(STMLIBDIR)/STM32_USB-FS-Device_Driver/src/*.o
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

all: lib $(SKETCHELF) $(SKETCHEEP) $(SKETCHHEX) $(SKETCHBIN) 
	
lib:
	$(MAKE) -C $(LIBDIR)

################################################################################
# Rules

# fetch dependency info from a previous build if any of it exists
-include $(ALLDEPS)

# Link the final object
$(SKETCHELF):	$(SKETCHOBJS) $(LIBOBJS)
	$(RULEHDR)
	$(v)$(LD) $(LDFLAGS) $^ -o $@ $(STARTUP) -L$(STMLIBDIR) $(LDLIBS)
	$(v)$(SIZE) $(SKETCHELF)

# Create the hex file
$(SKETCHHEX):	$(SKETCHELF)
	$(RULEHDR)
	$(v)$(OBJCOPY) -O ihex -R .eeprom $< $@

# Create the bin file
$(SKETCHBIN):	$(SKETCHELF)
	$(RULEHDR)
	$(v)$(OBJCOPY) -O binary $< $@

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
