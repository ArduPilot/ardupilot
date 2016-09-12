################################################################################
# Sketch
#

SRCSUFFIXES = *.cpp

MAKE_INC=$(wildcard $(SRCROOT)/make.inc)
ifeq (,$(MAKE_INC))
$(error You must have a make.inc file to list library dependencies)
endif

GLOBAL_MAKE_INC=$(wildcard $(SKETCHBOOK)/mk/make.inc)
ifeq (,$(GLOBAL_MAKE_INC))
$(error You must have a make.inc in mk/ directory)
endif

# Sketch source files
SKETCHSRCS     := $(wildcard $(addprefix $(SRCROOT)/,$(SRCSUFFIXES)))
SKETCHCPP      := $(SRCROOT)/$(SKETCH).cpp

# Sketch object files
SKETCHOBJS := $(subst $(SRCROOT),$(BUILDROOT),$(SKETCHSRCS))
SKETCHOBJS := $(addsuffix .o,$(basename $(SKETCHOBJS)))

# get list of libraries from make.inc
include $(MAKE_INC)
include $(GLOBAL_MAKE_INC)
LIBTOKENS := $(LIBRARIES)


# HAL and board specific libraries are included here.
LIBTOKENS += \
	AP_HAL \
	AP_HAL_Empty

ifeq ($(HAL_BOARD),HAL_BOARD_APM1)
LIBTOKENS += \
	AP_HAL_APM
endif

ifeq ($(HAL_BOARD),HAL_BOARD_APM2)
LIBTOKENS += \
	AP_HAL_APM
endif

ifeq ($(HAL_BOARD),HAL_BOARD_SITL)
LIBTOKENS += \
	AP_HAL_SITL \
	SITL
endif

ifeq ($(HAL_BOARD),HAL_BOARD_LINUX)
LIBTOKENS += \
	AP_HAL_Linux
endif

ifeq ($(HAL_BOARD),HAL_BOARD_PX4)
LIBTOKENS += \
	AP_HAL_PX4
endif

ifeq ($(HAL_BOARD),HAL_BOARD_VRBRAIN)
LIBTOKENS += \
	AP_HAL_VRBRAIN
endif

ifeq ($(HAL_BOARD),HAL_BOARD_QURT)
LIBTOKENS += \
	AP_HAL_QURT
endif

#
# Find sketchbook libraries referenced by the sketch.
#
# Include paths for sketch libraries 
#
SKETCHLIBS		:=	$(wildcard $(addprefix $(SKETCHBOOK)/libraries/,$(LIBTOKENS)))
SKETCHLIBNAMES		:=	$(notdir $(SKETCHLIBS))
SKETCHLIBSRCDIRS	:=	$(SKETCHLIBS) $(addsuffix /utility,$(SKETCHLIBS))
SKETCHLIBSRCS		:=	$(wildcard $(foreach suffix,$(SRCSUFFIXES),$(addsuffix /$(suffix),$(SKETCHLIBSRCDIRS))))
SKETCHLIBOBJS		:=	$(addsuffix .o,$(basename $(subst $(SKETCHBOOK),$(BUILDROOT),$(SKETCHLIBSRCS))))
SKETCHLIBINCLUDES	:=	-I$(SKETCHBOOK)/libraries/ -I$(BUILDROOT)/libraries/ -I$(BUILDROOT)/libraries/GCS_MAVLink/
SKETCHLIBSRCSRELATIVE	:=	$(subst $(SKETCHBOOK)/,,$(SKETCHLIBSRCS))

ifeq ($(VERBOSE),)
v = @
else
v =
endif

FORCE:

$(BUILDROOT)/make.flags: FORCE
	@mkdir -p $(BUILDROOT)
	@echo "// BUILDROOT=$(BUILDROOT) HAL_BOARD=$(HAL_BOARD) HAL_BOARD_SUBTYPE=$(HAL_BOARD_SUBTYPE) TOOLCHAIN=$(TOOLCHAIN) EXTRAFLAGS=$(EXTRAFLAGS)" > $(BUILDROOT)/make.flags.new
	@cmp $(BUILDROOT)/make.flags $(BUILDROOT)/make.flags.new > /dev/null 2>&1 || rm -f $(SRCROOT)/*.o
	@cmp $(BUILDROOT)/make.flags $(BUILDROOT)/make.flags.new > /dev/null 2>&1 || mv $(BUILDROOT)/make.flags.new $(BUILDROOT)/make.flags
	@rm -f $(BUILDROOT)/make.flags.new
	@cat $(BUILDROOT)/make.flags

# common header for rules, prints what is being built
define RULEHDR
	@echo %% $(subst $(BUILDROOT)/,,$@)
	@mkdir -p $(dir $@)
endef
