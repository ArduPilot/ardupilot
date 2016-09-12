# find key paths and system type

# Save the system type for later use.
#
SYSTYPE			:=	$(shell uname)

GIT_VERSION ?= $(shell git rev-parse HEAD | cut -c1-8)
EXTRAFLAGS += -DGIT_VERSION="\"$(GIT_VERSION)\""

# Add missing parts from libc and libstdc++ for all boards
EXTRAFLAGS += -I$(SKETCHBOOK)/libraries/AP_Common/missing

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

ifneq ($(wildcard $(SKETCHBOOK)/config.mk),)
$(info Reading $(SKETCHBOOK)/config.mk)
include $(SKETCHBOOK)/config.mk
endif

ifneq ($(wildcard $(SKETCHBOOK)/developer.mk),)
$(info Reading $(SKETCHBOOK)/developer.mk)
include $(SKETCHBOOK)/developer.mk
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
endif

ifneq ($(findstring vrbrain, $(MAKECMDGOALS)),)
# when building vrbrain we need all sources to be inside the sketchbook directory
# as the NuttX build system relies on it
BUILDROOT		:=	$(SKETCHBOOK)/Build.$(SKETCH)
endif

ifneq ($(findstring vrubrain, $(MAKECMDGOALS)),)
# when building vrbrain we need all sources to be inside the sketchbook directory
# as the NuttX build system relies on it
BUILDROOT		:=	$(SKETCHBOOK)/Build.$(SKETCH)
endif

ifneq ($(findstring vrcore, $(MAKECMDGOALS)),)
# when building vrcore we need all sources to be inside the sketchbook directory
# as the NuttX build system relies on it
BUILDROOT		:=	$(SKETCHBOOK)/Build.$(SKETCH)
endif

ifeq ($(BUILDROOT),)
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

ifneq ($(findstring mavlink1, $(MAKECMDGOALS)),)
EXTRAFLAGS += -DMAVLINK_PROTOCOL_VERSION=1
MAVLINK_SUBDIR=v1.0
MAVLINK_WIRE_PROTOCOL=1.0
else
EXTRAFLAGS += -DMAVLINK_PROTOCOL_VERSION=2
MAVLINK_SUBDIR=v2.0
MAVLINK_WIRE_PROTOCOL=2.0
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
HAL_BOARD = HAL_BOARD_SITL
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_NONE
endif

ifneq ($(findstring linux, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_NONE
endif

ifneq ($(findstring erleboard, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD
endif

ifneq ($(findstring zynq, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_ZYNQ
endif

ifneq ($(findstring pxf, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_PXF
endif

ifneq ($(findstring bebop, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_BEBOP
endif


ifneq ($(findstring navio, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_NAVIO
endif

ifneq ($(findstring raspilot, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_RASPILOT
endif

ifneq ($(findstring erlebrain2, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2
endif

ifneq ($(findstring bbbmini, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_BBBMINI
endif

ifneq ($(findstring minlure, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_MINLURE
endif

ifneq ($(findstring vrbrain, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_VRBRAIN
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_NONE
endif

ifneq ($(findstring vrubrain, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_VRBRAIN
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_NONE
endif

ifneq ($(findstring vrcore, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_VRBRAIN
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_NONE
endif

ifneq ($(findstring bhat, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_BH
endif

ifneq ($(findstring qflight, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_QFLIGHT
endif

ifneq ($(findstring qurt, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_QURT
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_NONE
endif

ifneq ($(findstring pxfmini, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_PXFMINI
endif

# default to SITL
ifeq ($(HAL_BOARD),)
HAL_BOARD = HAL_BOARD_SITL
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_NONE
endif

ifneq ($(findstring navio2, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_LINUX
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_LINUX_NAVIO2
endif
