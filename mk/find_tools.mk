################################################################################
# Tools
#

#
# Decide where we are going to look for tools
#
ifeq ($(SYSTYPE),Darwin)
  # use the tools that come with Arduino
  TOOLPATH :=  $(ARDUINO)/hardware/tools/avr/bin
  # use BWK awk
  AWK =  awk
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1),$(TOOLPATH))))
endif
ifeq ($(SYSTYPE),Linux)
  # expect that tools are on the path
  TOOLPATH :=  $(subst :, ,$(PATH))
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1),$(TOOLPATH))))
endif
ifeq ($(findstring CYGWIN, $(SYSTYPE)),CYGWIN) 
  TOOLPATH :=  $(ARDUINO)/hardware/tools/avr/bin
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1).exe,$(TOOLPATH))))
endif
ifeq ($(findstring MINGW, $(SYSTYPE)),MINGW) 
  TOOLPATH :=  $(ARDUINO)/hardware/tools/avr/bin
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1).exe,$(TOOLPATH))))
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

# Tools for Maple/Flymaple
# Toolchain is expected to be on the PATH
ARM_CXX     :=  $(call FIND_TOOL,arm-none-eabi-g++)
ARM_CC      :=  $(call FIND_TOOL,arm-none-eabi-gcc)
ARM_AS      :=  $(call FIND_TOOL,arm-none-eabi-gcc)
ARM_AR      :=  $(call FIND_TOOL,arm-none-eabi-ar)
ARM_LD      :=  $(call FIND_TOOL,arm-none-eabi-g++)
ARM_GDB     :=  $(call FIND_TOOL,arm-none-eabi-gdb)
ARM_OBJCOPY :=  $(call FIND_TOOL,arm-none-eabi-objcopy)

# toolchains for beagleboneblack
BBONE_CXX     :=  arm-linux-gnueabihf-g++-4.7
BBONE_CC      :=  arm-linux-gnueabihf-gcc-4.7
BBONE_AS      :=  arm-linux-gnueabihf-gcc-4.7
BBONE_AR      :=  ar
BBONE_LD      :=  arm-linux-gnueabihf-g++-4.7
BBONE_GDB     :=  gdb
BBONE_OBJCOPY :=  objcopy

# enable ccache if installed
CCACHE :=  $(call FIND_TOOL,ccache)

CXX = $(CCACHE) $($(TOOLCHAIN)_CXX)
CC = $(CCACHE) $($(TOOLCHAIN)_CC)
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

