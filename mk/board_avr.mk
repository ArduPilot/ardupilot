TOOLCHAIN = AVR

include $(MK_DIR)/find_arduino.mk
include $(MK_DIR)/find_tools.mk

#
# Tool options
#
DEFINES         =   -DF_CPU=$(F_CPU)
DEFINES        +=   -DSKETCH=\"$(SKETCH)\" -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)
DEFINES        +=   $(EXTRAFLAGS) # from user config.mk
DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD)
WARNFLAGS       =   -Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align
WARNFLAGS      +=   -Wwrite-strings -Wformat=2 -Wno-unused-parameter -Wno-missing-field-initializers
WARNFLAGSCXX    =   -Wno-reorder
DEPFLAGS        =   -MD -MT $@

CXXOPTS         =   -ffunction-sections -fdata-sections -fno-exceptions -fsigned-char
COPTS           =   -ffunction-sections -fdata-sections -fsigned-char

ASOPTS          =   -x assembler-with-cpp 
LISTOPTS        =   -adhlns=$(@:.o=.lst)

NATIVE_CPUFLAGS     = -D_GNU_SOURCE
NATIVE_CPULDFLAGS   = -g
NATIVE_OPTFLAGS     = -O0 -g

AVR_CPUFLAGS        = -mmcu=$(MCU) -mcall-prologues 
AVR_CPULDFLAGS      = -Wl,-m,avr6
AVR_OPTFLAGS        = -Os

CPUFLAGS= $($(TOOLCHAIN)_CPUFLAGS)
CPULDFLAGS= $($(TOOLCHAIN)_CPULDFLAGS)
OPTFLAGS= $($(TOOLCHAIN)_OPTFLAGS)

CXXFLAGS        =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS)
CXXFLAGS       +=   $(WARNFLAGS) $(WARNFLAGSCXX) $(DEPFLAGS) $(CXXOPTS)
CFLAGS          =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS)
CFLAGS         +=   $(WARNFLAGS) $(DEPFLAGS) $(COPTS)
ASFLAGS         =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(DEPFLAGS)
ASFLAGS        +=   $(ASOPTS)
LDFLAGS         =   -g $(CPUFLAGS) $(OPTFLAGS) $(WARNFLAGS)
LDFLAGS        +=   -Wl,--gc-sections -Wl,-Map -Wl,$(SKETCHMAP)

ifneq ($(BOARD),mega)
  LDFLAGS      +=   $(CPULDFLAGS)
endif

# under certain situations with certain avr-gcc versions the --relax flag causes
# a bug. Give the user a way to disable this flag per-sketch.
# I know this is a rotten hack but we're really close to sunset on AVR.
EXCLUDE_RELAX := $(wildcard $(SRCROOT)/norelax.inoflag)
ifeq ($(EXCLUDE_RELAX),)
  LDFLAGS      +=   -Wl,--relax
endif

LIBS = -lm

ifeq ($(VERBOSE),)
v = @
else
v =
endif

# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS)

# Find the hardware directory to use
HARDWARE_DIR		:=	$(firstword $(wildcard $(SKETCHBOOK)/hardware/$(HARDWARE) \
							$(ARDUINO)/hardware/$(HARDWARE)))
ifeq ($(HARDWARE_DIR),)
$(error ERROR: hardware directory for $(HARDWARE) not found)
endif

# Find the boards.txt that we are using
BOARDFILE		:=	$(wildcard $(HARDWARE_DIR)/boards.txt)
ifeq ($(BOARDFILE),)
$(error ERROR: could not locate boards.txt for hardware $(HARDWARE))
endif

# Extract needed build parameters from the boardfile
MCU			:=	$(shell grep $(BOARD).build.mcu $(BOARDFILE) | cut -d = -f 2)
F_CPU		:=	$(shell grep $(BOARD).build.f_cpu $(BOARDFILE) | cut -d = -f 2)
HARDWARE_CORE :=	$(shell grep $(BOARD).build.core $(BOARDFILE) | cut -d = -f 2)
UPLOAD_SPEED :=	$(shell grep $(BOARD).upload.speed $(BOARDFILE) | cut -d = -f 2)

# User can define USERAVRDUDEFLAGS = -V in their config.mk to skip verification
USERAVRDUDEFLAGS ?= 
#make sure the avrdude conf file is referenced correctly in cygwin
ifneq ($(findstring CYGWIN, $(SYSTYPE)),) 
  USERAVRDUDEFLAGS := -C $(ARDUINO)/hardware/tools/avr/etc/avrdude.conf
endif
#make sure the avrdude conf file is referenced correctly in mingw
ifneq ($(findstring MINGW, $(SYSTYPE)),) 
  USERAVRDUDEFLAGS := -C $(ARDUINO)/hardware/tools/avr/etc/avrdude.conf
endif
#make sure the avrdude conf file is referenced correctly in darwin
ifneq ($(findstring Darwin, $(SYSTYPE)),)
  USERAVRDUDEFLAGS := -C $(ARDUINO)/hardware/tools/avr/etc/avrdude.conf
endif

ifeq ($(UPLOAD_PROTOCOL),)
  UPLOAD_PROTOCOL	:=	$(shell grep $(BOARD).upload.protocol $(BOARDFILE) | cut -d = -f 2)
endif

# Adding override for mega since boards.txt uses stk500 instead of
# arduino on 22 release
ifeq ($(BOARD),mega)
  UPLOAD_PROTOCOL	:=	arduino
endif

#On Cygwin, the wiring programmer will perform the DTR reset for us
ifneq ($(findstring CYGWIN, $(SYSTYPE)),) 
  UPLOAD_PROTOCOL	:=	wiring
endif
 
ifeq ($(MCU),)
$(error ERROR: Could not locate board $(BOARD) in $(BOARDFILE))
endif

################################################################################
# Built products
#

# The ELF file
SKETCHELF		=	$(BUILDROOT)/$(SKETCH).elf
BUILDELF                =       $(notdir $(SKETCHELF))

# HEX file
SKETCHHEX		=	$(BUILDROOT)/$(SKETCH).hex

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
#

all: $(SKETCHELF) $(SKETCHEEP) $(SKETCHHEX)

print-%:
	echo "$*=$($*)"

.PHONY: upload
upload: $(SKETCHHEX)
	$(AVRDUDE) -c $(UPLOAD_PROTOCOL) -p $(MCU) -P $(PORT) -b$(UPLOAD_SPEED) $(USERAVRDUDEFLAGS) -U flash:w:$(SKETCHHEX):i

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

# Link the final object
$(SKETCHELF):	$(SKETCHOBJS) $(LIBOBJS)
	$(RULEHDR)
	$(v)$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(v)cp $(SKETCHELF) $(BUILDELF)
	@echo "Firmware is in $(BUILDELF)"

# Create the hex file
$(SKETCHHEX):	$(SKETCHELF)
	$(RULEHDR)
	$(v)$(OBJCOPY) -O ihex -R .eeprom $< $@

# Create the eep file
$(SKETCHEEP):	$(SKETCHELF)
	$(RULEHDR)
	$(v)$(OBJCOPY) -O ihex -j.eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $@

#
# Build sketch objects
#
SKETCH_INCLUDES	=	$(SKETCHLIBINCLUDES) $(ARDUINOLIBINCLUDES) $(COREINCLUDES)

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
SLIB_INCLUDES	=	-I$(dir $<)/utility $(SKETCHLIBINCLUDES) $(ARDUINOLIBINCLUDES) $(COREINCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.c
	$(RULEHDR)
	$(v)$(CC) $(CFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

