# find key paths and system type

# Save the system type for later use.
#
SYSTYPE			:=	$(shell uname)

GIT_VERSION := $(shell git rev-parse HEAD | cut -c1-8)
EXTRAFLAGS += -DGIT_VERSION="\"$(GIT_VERSION)\""

# force LANG to C so awk works sanely on MacOS
export LANG=C

#
# Locate the sketch sources based on the initial Makefile's path
#
SRCROOT			:=	$(realpath $(dir $(firstword $(MAKEFILE_LIST))))
ifneq ($(findstring CYGWIN, $(SYSTYPE)),) 
  # Workaround a $(realpath ) bug on cygwin
  ifeq ($(SRCROOT),)
    SRCROOT	:=	$(shell cygpath -m ${CURDIR})
    $(warning your realpath function is not working)
    $(warning > setting SRCROOT to $(SRCROOT))
  endif
endif

#
# We need to know the location of the sketchbook.  If it hasn't been overridden,
# try the parent of the current directory.  If there is no libraries directory
# there, assume that we are in a library's examples directory and try backing up
# further.
#
ifeq ($(SKETCHBOOK),)
  SKETCHBOOK		:=	$(shell cd $(SRCROOT)/.. && pwd)
  ifeq ($(wildcard $(SKETCHBOOK)/libraries),)
    SKETCHBOOK		:=	$(shell cd $(SRCROOT)/../.. && pwd)
  endif
  ifeq ($(wildcard $(SKETCHBOOK)/libraries),)
    SKETCHBOOK		:=	$(shell cd $(SRCROOT)/../../.. && pwd)
  endif
  ifeq ($(wildcard $(SKETCHBOOK)/libraries),)
    SKETCHBOOK		:=	$(shell cd $(SRCROOT)/../../../.. && pwd)
  endif
  ifeq ($(wildcard $(SKETCHBOOK)/libraries),)
    $(error ERROR: cannot determine sketchbook location - please specify on the commandline with SKETCHBOOK=<path>)
  endif
else
  ifeq ($(wildcard $(SKETCHBOOK)/libraries),)
    $(warning WARNING: sketchbook directory $(SKETCHBOOK) contains no libraries)
  endif
endif
ifneq ($(findstring CYGWIN, $(SYSTYPE)),) 
    # Convert cygwin path into a windows normal path
    SKETCHBOOK	:= $(shell cygpath ${SKETCHBOOK})
endif

#
# Work out the sketch name from the name of the source directory.
#
SKETCH			:=	$(lastword $(subst /, ,$(SRCROOT)))
# Workaround a $(lastword ) bug on cygwin
ifeq ($(SKETCH),)
  WORDLIST		:=	$(subst /, ,$(SRCROOT))
  SKETCH		:=	$(word $(words $(WORDLIST)),$(WORDLIST))
endif

#
# Work out where we are going to be building things
#
TMPDIR			?=	/tmp

ifneq ($(findstring px4, $(MAKECMDGOALS)),)
# when building px4 we need all sources to be inside the sketchbook directory
# as the NuttX build system relies on it
BUILDROOT		:=	$(SKETCHBOOK)/Build.$(SKETCH)
else
BUILDROOT		:=	$(abspath $(TMPDIR)/$(SKETCH).build)
endif

ifneq ($(findstring CYGWIN, $(SYSTYPE)),)
  # Workaround a $(abspath ) bug on cygwin
  ifeq ($(BUILDROOT),)
    BUILDROOT	:=	C:$(TMPDIR)/$(SKETCH).build
    $(warning your abspath function is not working)
    $(warning > setting BUILDROOT to $(BUILDROOT))
  else
    BUILDROOT	:=	$(shell cygpath ${BUILDROOT})
  endif
endif

# Jump over the next makefile sections when runing a "make configure"
ifneq ($(MAKECMDGOALS),configure)

################################################################################
# Config options
#
# The Makefile calling us must specify BOARD
#
include $(SKETCHBOOK)/config.mk
ifeq ($(PORT),)
$(error ERROR: could not locate $(SKETCHBOOK)/config.mk, please run 'make configure' first and edit config.mk)
endif

ifneq ($(APPDIR),)
# this is a recusive PX4 build
HAL_BOARD = HAL_BOARD_PX4
endif

# handle target based overrides for board type
ifneq ($(findstring px4, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_PX4
endif

ifneq ($(findstring sitl, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_AVR_SITL
endif

ifneq ($(findstring linux, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
endif

ifneq ($(findstring vrbrain, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_VRBRAIN
endif

ifneq ($(findstring apm1, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_APM1
endif

ifneq ($(findstring apm2, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_APM2
endif

ifneq ($(findstring flymaple, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_FLYMAPLE
endif

# default to APM2
ifeq ($(HAL_BOARD),)
#$(warning No HAL_BOARD in config.mk - defaulting to HAL_BOARD_APM2)
HAL_BOARD = HAL_BOARD_APM2
endif

HARDWARE		?=	arduino
ifeq ($(BOARD),)
BOARD = mega2560
endif

ifneq ($(findstring apm1-1280, $(MAKECMDGOALS)),)
BOARD = mega
endif

endif

