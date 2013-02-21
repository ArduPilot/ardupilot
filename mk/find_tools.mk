################################################################################
# Tools
#

#
# Decide where we are going to look for tools
#
ifeq ($(SYSTYPE),Darwin)
  # use the tools that come with Arduino
  TOOLPATH :=  $(ARDUINOS)/hardware/tools/avr/bin
  # use BWK awk
  AWK =  awk
endif
ifeq ($(SYSTYPE),Linux)
  # expect that tools are on the path
  TOOLPATH :=  $(subst :, ,$(PATH))
endif
ifeq ($(findstring CYGWIN, $(SYSTYPE)),CYGWIN) 
  TOOLPATH :=  $(ARDUINO)/hardware/tools/avr/bin
endif

ifeq ($(findstring CYGWIN, $(SYSTYPE)),) 
FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1),$(TOOLPATH))))
else
FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1).exe,$(TOOLPATH))))
endif

NATIVE_CXX     :=  $(call FIND_TOOL,g++)
NATIVE_CC      :=  $(call FIND_TOOL,gcc)
NATIVE_AS      :=  $(call FIND_TOOL,gcc)
NATIVE_AR      :=  $(call FIND_TOOL,ar)
NATIVE_LD      :=  $(call FIND_TOOL,g++)
NATIVE_GDB     :=  $(call FIND_TOOL,gdb)
NATIVE_OBJCOPY :=  $(call FIND_TOOL,objcopy)

AVR_CXX     :=  $(call FIND_TOOL,avr-g++)
AVR_CC      :=  $(call FIND_TOOL,avr-gcc)
AVR_AS      :=  $(call FIND_TOOL,avr-gcc)
AVR_AR      :=  $(call FIND_TOOL,avr-ar)
AVR_LD      :=  $(call FIND_TOOL,avr-gcc)
AVR_GDB     :=  $(call FIND_TOOL,avr-gdb)
AVR_OBJCOPY :=  $(call FIND_TOOL,avr-objcopy)

AVRDUDE      :=  $(call FIND_TOOL,avrdude)
AVARICE      :=  $(call FIND_TOOL,avarice)

CXX = $($(TOOLCHAIN)_CXX)
CC = $($(TOOLCHAIN)_CC)
AS = $($(TOOLCHAIN)_AS)
AR = $($(TOOLCHAIN)_AR)
LD = $($(TOOLCHAIN)_LD)
GDB = $($(TOOLCHAIN)_GDB)
OBJCOPY = $($(TOOLCHAIN)_OBJCOPY)

ifeq ($(AVR_CXX),)
$(error ERROR: cannot find the AVR compiler tools anywhere on the path $(TOOLPATH))
endif

# Find awk
AWK			?=	gawk
ifeq ($(shell which $(AWK)),)
$(error ERROR: cannot find $(AWK) - you may need to install GNU awk)
endif

