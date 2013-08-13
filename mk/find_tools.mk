################################################################################
# Tools
#

#
# Decide where we are going to look for tools
#
ifeq ($(SYSTYPE),Darwin)
  # first search in the macports directory, then on path, 
  # otherwise use the tools that come with Arduino
  TOOLPATH 	      := /opt/local/bin $(subst :, ,$(PATH)) $(ARDUINOS)/hardware/tools/avr/bin
  # unfortunately the macports avr-size version doesn't support --format=avr
  # so we have to use the one that ships with Arduino
  TOOLPATHAVRSIZE := $(ARDUINOS)/hardware/tools/avr/bin
  # use BWK awk
  AWK =  awk
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1),$(TOOLPATH))))
  FIND_AVRSIZE =  $(firstword $(wildcard $(addsuffix /$(1),$(TOOLPATHAVRSIZE))))    
endif
ifeq ($(SYSTYPE),Linux)
  # expect that tools are on the path
  TOOLPATH :=  $(subst :, ,$(PATH))
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1),$(TOOLPATH))))
  FIND_AVRSIZE = $(FIND_TOOL)
endif
ifeq ($(findstring CYGWIN, $(SYSTYPE)),CYGWIN) 
  TOOLPATH :=  $(ARDUINO)/hardware/tools/avr/bin
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1).exe,$(TOOLPATH))))
  FIND_AVRSIZE = $(FIND_TOOL)
endif
ifeq ($(findstring MINGW, $(SYSTYPE)),MINGW) 
  TOOLPATH :=  $(ARDUINO)/hardware/tools/avr/bin
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1).exe,$(TOOLPATH))))
  FIND_AVRSIZE = $(FIND_TOOL)
endif

NATIVE_CXX     :=  g++
NATIVE_CC      :=  gcc
NATIVE_AS      :=  gcc
NATIVE_AR      :=  ar
NATIVE_LD      :=  g++
NATIVE_GDB     :=  gdb
NATIVE_OBJCOPY :=  objcopy

AVR_CXX     :=  $(call FIND_TOOL,avr-g++)
AVR_CC      :=  $(call FIND_TOOL,avr-gcc)
AVR_AS      :=  $(call FIND_TOOL,avr-gcc)
AVR_AR      :=  $(call FIND_TOOL,avr-ar)
AVR_LD      :=  $(call FIND_TOOL,avr-gcc)
AVR_GDB     :=  $(call FIND_TOOL,avr-gdb)
AVR_OBJCOPY :=  $(call FIND_TOOL,avr-objcopy)

AVRDUDE      :=  $(call FIND_TOOL,avrdude)
AVARICE      :=  $(call FIND_TOOL,avarice)
AVRSIZE      :=  $(call FIND_AVRSIZE,avr-size)

CXX = $($(TOOLCHAIN)_CXX)
CC = $($(TOOLCHAIN)_CC)
AS = $($(TOOLCHAIN)_AS)
AR = $($(TOOLCHAIN)_AR)
LD = $($(TOOLCHAIN)_LD)
GDB = $($(TOOLCHAIN)_GDB)
OBJCOPY = $($(TOOLCHAIN)_OBJCOPY)

ifeq ($(CXX),)
$(error ERROR: cannot find the compiler tools for $(TOOLCHAIN) anywhere on the path $(TOOLPATH))
endif

# Find awk
AWK			?=	gawk
ifeq ($(shell which $(AWK)),)
$(error ERROR: cannot find $(AWK) - you may need to install GNU awk)
endif

