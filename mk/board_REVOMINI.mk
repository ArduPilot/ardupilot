# Try "make help" first

.DEFAULT_GOAL := all

##
## Target-specific configuration.  This determines some compiler and
## linker options/flags.
##

TOOLCHAIN = ARM

PATH:=$(PATH):/usr/local/bin

# these default to a Maple Flash build.
BOARD            ?= revomini_MP32V1F4
MEMORY_TARGET    ?= flash

##
## Useful paths, constants, etc.
##


BLDROOT := $(SRCROOT)/..

#BUILD_PATH       := $(BLDROOT)/build
BUILD_PATH       := $(BUILDROOT)

HARDWARE_PATH    := $(BLDROOT)/libraries/AP_HAL_REVOMINI/hardware
HAL_PATH         := $(BLDROOT)/libraries/AP_HAL_REVOMINI/hardware/hal
STM32_PATH       := $(HARDWARE_PATH)/STM32F4xx_DSP_StdPeriph_Lib_V1.1.0
STM32USB_PATH    := $(HARDWARE_PATH)/STM32_USB_Driver
cpu_flags        := -mcpu=cortex-m4
WIRISH_PATH      := $(BLDROOT)/libraries/AP_HAL_REVOMINI/wirish
LIBRARIES_PATH   := $(BLDROOT)/libraries
UCLIBC_PATH      := $(BLDROOT)/modules/PX4NuttX/misc/uClibc++/include
#FIRMWARE_PATH    := $(BLDROOT)/Firmware/test_RCInputOutput
#SKETCH           := RC_Channel.pde
#FIRMWARE_PATH    := $(BLDROOT)/Firmware/SPIDriver_MS5611
#SKETCH           := SPIDriver_MS5611
#SKETCH           := AP_Baro_MS5611_test
#FIRMWARE_PATH    := $(BLDROOT)/Firmware/SPIDriver_MPU6000
#SKETCH          := SPIDriver_MPU6000
#FIRMWARE_PATH    := $(BLDROOT)/Firmware/Test_Compass
#SKETCH          := I2CDriver_HMC5883L
#FIRMWARE_PATH    := $(BLDROOT)/Firmware/Test_Scheduler
#SKETCH           := Scheduler
#FIRMWARE_PATH    := $(BLDROOT)/Firmware/Test_Blink
#SKETCH           := Blink
#FIRMWARE_PATH    := $(BLDROOT)/libraries/AP_HAL_REVOMINI/examples/DataFlash_test_MP32
#SKETCH           := DataFlash_test_MP32
#FIRMWARE_PATH    := $(BLDROOT)/libraries/AP_HAL_REVOMINI/examples/DataFlash_test
#SKETCH           := DataFlash_test_MP32

#FIRMWARE_PATH    := $(BLDROOT)/libraries/AP_HAL_REVOMINI/examples/DF_Parameters_test
#SKETCH           := DF_Parameters_test

FIRMWARE_PATH    := $(BLDROOT)/ArduCopter
SKETCH           := ArduCopter

#FIRMWARE_PATH    := $(BLDROOT)/APMRover2
#SKETCH          := APMRover2
#FIRMWARE_PATH    := $(BLDROOT)/ArduPlane
#SKETCH          := ArduPlane

# List of input files to the sketch.cpp file in the order they should
# be appended to create it


LIBRARY_INCLUDES := 

SUPPORT_PATH     := $(BLDROOT)/libraries/AP_HAL_REVOMINI/support
# Support files for linker
LDDIR            := $(SUPPORT_PATH)/ld
# Support files for this Makefile
MAKEDIR          := $(SUPPORT_PATH)/make

# $(BOARD)- and $(MEMORY_TARGET)-specific configuration
include $(MAKEDIR)/target-config.mk

SIZE     := arm-none-eabi-size
DISAS    := arm-none-eabi-objdump

##
## Compilation flags
##

WARNFLAGS       =   -Wall -Wshadow -Wpointer-arith -Wcast-align -Wno-psabi
WARNFLAGS      +=   -Wwrite-strings -Wformat=2
WARNFLAGSCXX    =   -Wno-reorder
DEPFLAGS        =   -MD -MT $@

GLOBAL_FLAGS    := -D$(VECT_BASE_ADDR)
GLOBAL_FLAGS    += -DBOARD_$(BOARD)
GLOBAL_FLAGS    += -DMCU_$(MCU)
GLOBAL_FLAGS    += -DCONFIG_HAL_BOARD=$(HAL_BOARD)
GLOBAL_FLAGS    += -DSTM32F4XX
GLOBAL_FLAGS    += -DUSE_STDPERIPH_DRIVER
GLOBAL_FLAGS    += -DHSE_VALUE=8000000
GLOBAL_FLAGS    += -DARM_MATH_CM4
GLOBAL_FLAGS    += -DUSE_OTG_FS_CORE
GLOBAL_FLAGS    += -DUSE_USB_OTG_FS
GLOBAL_FLAGS    += -DUSE_EMBEDDED_PHY
GLOBAL_FLAGS    += -D__FPU_PRESENT
GLOBAL_FLAGS    += -D__FPU_USED=1
GLOBAL_FLAGS    += -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)
GLOBAL_FLAGS    += $(WARNFLAGS) $(DEPFLAGS) -nostdlib
# GLOBAL_CFLAGS -----------------------------------------------------------------------------------
GLOBAL_CFLAGS   := $(cpu_flags)
GLOBAL_CFLAGS   += -mthumb             #Generate code for the Thumb instruction set
GLOBAL_CFLAGS   += -ggdb               #Produce debugging information in the operating system’s native format
#GLOBAL_CFLAGS   += -O0                 #Reduce compilation time and make debugging produce the expected results. This is the default
GLOBAL_CFLAGS   += -Os                 #Reduce size
#GLOBAL_CFLAGS   += -nostdlib           #Do not use the standard system startup files or libraries when linking
GLOBAL_CFLAGS   += -Wall               #This enables all the warnings about constructions that some users consider questionable, and that are easy to avoid (or modify to prevent the warning), even in conjunction with macros
GLOBAL_CFLAGS   += -ffunction-sections
GLOBAL_CFLAGS   += -fdata-sections
GLOBAL_CFLAGS   += -mfpu=fpv4-sp-d16
GLOBAL_CFLAGS   += -mfloat-abi=softfp
GLOBAL_CFLAGS   += $(GLOBAL_FLAGS)
# GLOBAL_CXXFLAGS ---------------------------------------------------------------------------------
#GLOBAL_CXXFLAGS := -fno-rtti       #Disable generation of information about every class with virtual functions for use by the C++ runtime type identification features
#GLOBAL_CXXFLAGS += -fno-exceptions
GLOBAL_CXXFLAGS += -Wall  -std=gnu++11 
GLOBAL_CXXFLAGS += -fpermissive    #Downgrade some diagnostics about nonconformant code from errors to warnings. Thus, using ‘-fpermissive’ will allow some nonconforming code to compile.
GLOBAL_CXXFLAGS += -Wno-psabi
GLOBAL_CXXFLAGS += $(GLOBAL_FLAGS)
# GLOBAL_ASFLAGS ----------------------------------------------------------------------------------
GLOBAL_ASFLAGS  := $(cpu_flags)
GLOBAL_ASFLAGS  += -mthumb
GLOBAL_CFLAGS   += -ggdb                 #Produce debugging information in the operating system’s native format
#GLOBAL_CFLAGS   += -O0                #Reduce compilation time and make debugging produce the expected results. This is the default
GLOBAL_CFLAGS   += -Os
GLOBAL_ASFLAGS  += -x assembler-with-cpp
GLOBAL_ASFLAGS  += $(GLOBAL_FLAGS)
# GLOBAL_LDFLAGS ----------------------------------------------------------------------------------
LDFLAGS         := $(cpu_flags)
LDFLAGS         += -mthumb
LDFLAGS         += -static
LDFLAGS         += -Wall
LDFLAGS         += -Wl,--gc-sections
LDFLAGS         += -Wl,--cref
LDFLAGS         += -Wl,-Map -Wl,$(BUILD_PATH)/$(BOARD).map
#LDFLAGS         += -Xlinker --gc-sections --print-gc-sections
LDFLAGS         += -T$(LDDIR)/$(LDSCRIPT)
LDFLAGS         += -L$(LDDIR) 
#-flto -fwhole-program
#--plugin=/usr/local/arm-none-eabi/lib/gcc/arm-none-eabi/5.2.1/liblto_plugin.so

TGT_BIN := 

##
## Build rules and useful templates
##

include $(MK_DIR)/find_tools.mk

#include $(SUPPORT_PATH)/make/build-rules.mk
#include $(SUPPORT_PATH)/make/build-templates.mk

include $(WIRISH_PATH)/rules.mk
include $(HARDWARE_PATH)/hal/rules.mk
include $(HARDWARE_PATH)/STM32_USB_Driver/rules.mk
include $(HARDWARE_PATH)/STM32F4xx_DSP_StdPeriph_Lib_V1.1.0/rules.mk


#COREOBJS = $(HARDWARE_PATH)/hal/*.o $(HARDWARE_PATH)/STM32_USB_Driver/*.o \
#$(HARDWARE_PATH)/STM32F4xx_DSP_StdPeriph_Lib_V1.1.0/Libraries/STM32F4xx_StdPeriph_Driver/src/*.o \
#$(WIRISH_PATH)/*.o $(WIRISH_PATH)/comm/*.o $(WIRISH_PATH)/boards/revomini_MP32V1F4/*.o

COREINCLUDES = -I$(HARDWARE_PATH) -I$(HAL_PATH) -I$(STM32_PATH) -I$(STM32_PATH)/Libraries/STM32F4xx_StdPeriph_Driver/inc \
-I$(STM32_PATH)/Libraries/CMSIS/Include  -I$(STM32_PATH)/Libraries/CMSIS/Device/ST/STM32F4xx/Include  \
-I$(STM32USB_PATH) -I$(WIRISH_PATH) -I$(WIRISH_PATH)/boards/revomini_MP32V1F4 -I$(UCLIBC_PATH)




CFLAGS   = $(GLOBAL_CFLAGS) $(TGT_CFLAGS) $(COREINCLUDES) -I$(LIBRARIES_PATH)  
CXXFLAGS = $(GLOBAL_CXXFLAGS) $(GLOBAL_CFLAGS)  $(TGT_CXXFLAGS) $(COREINCLUDES) -I$(LIBRARIES_PATH) 
ASFLAGS  = $(GLOBAL_ASFLAGS) $(TGT_ASFLAGS) 
#-flto



##
## Set all submodules here
##


SKETCH_INCLUDES = $(SKETCHLIBINCLUDES) $(ARDUINOLIBINCLUDES) 
#$(COREINCLUDES)

SLIB_INCLUDES  =       -I$(dir $<)/utility $(SKETCHLIBINCLUDES) $(ARDUINOLIBINCLUDES) 
#$(COREINCLUDES)

################################################################################
# Built products
#



# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS) $(COREOBJS)

# The ELF file
SKETCHELF		=	$(BUILDROOT)/$(SKETCH).elf
BUILDELF                =       $(notdir $(SKETCHELF))

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



CAT := cat

################################################################################
# Targets
#

all: $(SKETCHELF) $(SKETCHEEP) $(BUILD_PATH)/$(BOARD).bin

print-%:
	echo "$*=$($*)"

flymaple-upload: upload

.PHONY: upload
upload: $(SKETCHBIN)
	$(LIBMAPLE_PATH)/support/scripts/reset.py && sleep 1 &&  $(UPLOADER) -a1 -d $(USBID) -D $(SKETCHBIN) -R

debug:
	$(AVARICE) --mkII --capture --jtag usb :4242 & \
	gnome-terminal -x $(GDB) $(SKETCHELF) & \
	echo -e '\n\nat the gdb prompt type "target remote localhost:4242"'

# this allows you to flash your image via JTAG for when you
# have completely broken your USB
jtag-program:
	$(AVARICE) --mkII --jtag usb --erase --program --file $(SKETCHELF)

################################################################################
# Rules
#


# fetch dependency info from a previous build if any of it exists
-include $(ALLDEPS)


revomini-clean: clean
	$(v) /bin/rm -rf $(WIRISH_PATH)/*.d $(HARDWARE_PATH)/*.d $(HAL_PATH)/*.d $(STM32_PATH)/*.d $(STM32USB_PATH)/*.d \
	$(WIRISH_PATH)/boards/revomini_MP32V1F4/*.d $(STM32USB_PATH)/MDK-ARM-GCC/build/*.d $(STM32_PATH)/src/*.d $(WIRISH_PATH)/comm/*.d $(STM32_PATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/*.d

# Link the final object
$(SKETCHELF):	$(LIBOBJS) $(SKETCHOBJS) $(TGT_BIN) $(BUILD_PATH)/main.o
#	@echo LIBOBJS=$(LIBOBJS)
	@echo TGT_BIN=$(TGT_BIN)
	$(RULEHDR)
#	$(v)$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(SILENT_LD) $(CXX) $(LDFLAGS) -o $@ $(TGT_BIN) $(BUILD_PATH)/main.o $(SKETCHOBJS) $(LIBOBJS) $(LIBS)  -Wl,-Map,$(BUILD_PATH)/$(BOARD).map
	$(v)cp $(SKETCHELF) .
	@echo "Firmware is in $(BUILDELF)"

# Create the eep file
$(SKETCHEEP):	$(SKETCHELF)
	$(RULEHDR)
	$(v)$(OBJCOPY) -O ihex -j.eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $@

$(BUILD_PATH)/main.o: $(WIRISH_PATH)/main.cxx
	$(SILENT_CXX) $(CXX) $(CFLAGS) $(CXXFLAGS) $(LIBRARY_INCLUDES) -o $@ -c $<

$(BUILD_PATH)/$(BOARD).bin: $(SKETCHELF)
	$(SILENT_OBJCOPY) $(OBJCOPY) -v -Obinary $(SKETCHELF) $@ 1>/dev/null
	$(OBJCOPY) -v -Oihex $(SKETCHELF) $(BUILD_PATH)/$(BOARD).hex 1>/dev/null
	$(SILENT_DISAS) $(DISAS) -d $(SKETCHELF) > $(BUILD_PATH)/$(BOARD).disas
	@echo " "
	@echo "Object file sizes:"
	@find $(BUILD_PATH) -iname *.o | xargs $(SIZE) -t > $(BUILD_PATH)/$(BOARD).sizes
	@cat $(BUILD_PATH)/$(BOARD).sizes
	@echo " "
	@echo "Final Size:"
	@$(SIZE) $<
	@echo $(MEMORY_TARGET) > $(BUILD_PATH)/build-type

#
# Build sketch objects
#

include $(MK_DIR)/build_rules.mk

