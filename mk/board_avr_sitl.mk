TOOLCHAIN = NATIVE

include $(MK_DIR)/find_tools.mk

#
# Tool options
#
DEFINES         =   -DF_CPU=$(F_CPU)
DEFINES        +=   -DSKETCH=\"$(SKETCH)\"
DEFINES        +=   $(EXTRAFLAGS) # from user config.mk
DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD)
WARNFLAGS       =   -Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align
WARNFLAGS      +=   -Wwrite-strings -Wformat=2
WARNFLAGSCXX    =   -Wno-reorder
DEPFLAGS        =   -MD -MT $@

CXXOPTS         =   -ffunction-sections -fdata-sections -fno-exceptions -fsigned-char
COPTS           =   -ffunction-sections -fdata-sections -fsigned-char

ASOPTS          =   -x assembler-with-cpp 

ifneq ($(SYSTYPE),Darwin)
LISTOPTS        =   -adhlns=$(@:.o=.lst)
endif

CPUFLAGS     = -D_GNU_SOURCE
CPULDFLAGS   = -g
OPTFLAGS     = -O0 -g

CXXFLAGS        =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS)
CXXFLAGS       +=   $(WARNFLAGS) $(WARNFLAGSCXX) $(DEPFLAGS) $(CXXOPTS)
CFLAGS          =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS)
CFLAGS         +=   $(WARNFLAGS) $(DEPFLAGS) $(COPTS)
ASFLAGS         =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(DEPFLAGS)
ASFLAGS        +=   $(ASOPTS)
LDFLAGS         =   -g $(CPUFLAGS) $(OPTFLAGS) $(WARNFLAGS)

ifneq ($(SYSTYPE),Darwin)
LDFLAGS        +=   -Wl,--gc-sections -Wl,-Map -Wl,$(SKETCHMAP)
endif

LIBS = -lm

ifeq ($(VERBOSE),)
v = @
else
v =
endif

# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS)


################################################################################
# Built products
#

# The ELF file
SKETCHELF		=	$(BUILDROOT)/$(SKETCH).elf

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

all: $(SKETCHELF)

print-%:
	echo "$*=$($*)"

################################################################################
# Rules
#

# fetch dependency info from a previous build if any of it exists
-include $(ALLDEPS)

# Link the final object
$(SKETCHELF):	$(SKETCHOBJS) $(LIBOBJS)
	@echo "Building $(SKETCHELF)"
	$(RULEHDR)
	$(v)$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)

#
# Build sketch objects
#
SKETCH_INCLUDES	=	$(SKETCHLIBINCLUDES)

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
SLIB_INCLUDES	=	-I$(dir $<)/utility $(SKETCHLIBINCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.cpp
	$(RULEHDR)
	$(v)$(CXX) $(CXXFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.c
	$(RULEHDR)
	$(v)$(CC) $(CFLAGS) -c -o $@ $< $(SLIB_INCLUDES)

$(BUILDROOT)/libraries/%.o: $(SKETCHBOOK)/libraries/%.S
	$(RULEHDR)
	$(v)$(AS) $(ASFLAGS) -c -o $@ $< $(SLIB_INCLUDES)
