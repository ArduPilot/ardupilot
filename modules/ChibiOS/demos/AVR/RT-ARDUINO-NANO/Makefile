
##############################################################################
#
# @file   Makefile.
#
# @brief  AVR Make file, it can be use to build, and program an application to
#         an AVR MCU like atmega328p, atmega2560 and so on.
#
# @author Theodore Ateba, tf.ateba@gmail.com
#
##############################################################################

##############################################################################
# Building and programming global options.
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT =
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = --defsym=__main_thread_stack_base__=0,--defsym=__main_thread_stack_end__=0
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

# If enabled, this option avoid to build the generic Flash Driver class.
# This class use 32 bits pointers that generate some warning at AVR compilation process.
ifeq ($(EXCLUDE_FLASH),)
  EXCLUDE_FLASH = yes
endif

# If enable, this option arase the counter cycle after device programming.
ifeq ($(USE_AVRDUDE_ERASE_COUNTER),)
	USE_AVRDUDE_ERASE_COUNTER = no
endif

# If enable, this option perform a verification after device programming.
ifeq ($(USE_AVRDUDE_NO_VERIFY),)
	USE_AVRDUDE_NO_VERIFY = no
endif

# If enabled, this option increase the programming verbosity level.
ifeq ($(USE_VERBOSE_PROGRAMMATION),)
  USE_VERBOSE_PROGRAMMATION = no
endif

# Enable this if you want to use AVRDUDE programmer.
ifeq ($(USE_AVRDUDE_PROGRAMMER),)
  USE_AVRDUDE_PROGRAMMER = yes
endif

# Enable this if you want to use DFU programmer.
ifeq ($(USE_DFU_PROGRAMMER),)
  USE_DFU_PROGRAMMER = no
endif

# Enable this if you want to use MICRONUCLEUS programmer.
ifeq ($(USE_MICRONUCLEUS_PROGRAMMER),)
  USE_MICRONUCLEUS_PROGRAMMER = no
endif

#
# Building and programming global options.
##############################################################################

##############################################################################
# Project, sources and paths.
#

# Define project name here.
PROJECT = ch

# Imported source files and paths.
CHIBIOS  := ../../..
CONFDIR  := ./cfg
BUILDDIR := ./build
DEPDIR   := ./.dep

# Licensing files.
include $(CHIBIOS)/os/license/license.mk

# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/boards/ARDUINO_NANO/board.mk
include $(CHIBIOS)/os/hal/ports/AVR/MEGA/ATMEGAxx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk

# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/AVR/compilers/GCC/mk/port.mk

# List C source files here. (C dependencies are automatically generated.)
CSRC =  $(ALLCSRC) \
        main.c

# List C++ sources file here.
CPPSRC = $(ALLCPPSRC)

# Header files here.
INCDIR = $(CONFDIR) $(ALLINC)

#
# Project, sources and paths.
##############################################################################

##############################################################################
# Compiler settings.
#

# Micro-Controller Unit.
MCU  = atmega328p

# MCU frequency (Hz).
F_CPU = 16000000

# Output format. (can be srec, ihex, binary)
FORMAT = ihex

# C and C++ Compiler name.
TRGT = avr-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++

# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD    = $(TRGT)gcc
CP    = $(TRGT)objcopy
AR    = $(TRGT)ar rcs
OD    = $(TRGT)objdump
NM    = $(TRGT)nm
SZ    = $(TRGT)size
HEX   = $(CP) -O ihex
BIN   = $(CP) -O binary

# Size of the elf binary file.
ELFSIZE = $(SZ) $(BUILDDIR)/$(PROJECT).elf

# MCU specific options here.
MOPT =

# Define C warning options here.
CWARN = -Wall -Wstrict-prototypes

# Define C++ warning options here.
CPPWARN =

#
# Compiler settings.
##############################################################################

##############################################################################
# Start of user section.
#

# List all user C define here, like -D_DEBUG=1.
UDEFS = -DTEST_CFG_SIZE_REPORT=0

# Define ASM defines here.
UADEFS =

# List all user directories here.
UINCDIR =

# List the user directory to look for the libraries here.
ULIBDIR =

# List all user libraries here.
ULIBS =

#
# End of user defines.
##############################################################################

##############################################################################
# Start of programming Options.
#

# List of available AVR programmer.
AVRDUDE_PROGRAMMER    = avrdude
AVRDUDE_PROGRAMMER_ID = wiring
DFU_PROGRAMMER        = dfu-programmer
MICRONUCLEUS          = micronucleus

# Set the AVR programmer according to the selection..
ifeq ($(USE_AVRDUDE_PROGRAMMER),yes)
	AVR_PROGRAMMER = $(AVRDUDE_PROGRAMMER)
else ifeq ($(USE_DFU_PROGRAMMER),yes)
	AVR_PROGRAMMER = $(DFU_PROGRAMMER)
else ifeq ($(USE_MICRONUCLEUS_PROGRAMMER),yes)
	AVR_PROGRAMMER = $(MICRONUCLEUS_PROGRAMMER)
else
  $(error ERROR: Please you need to configure the AVR programmer!)
endif

# AVR serial port.
AVRDUDE_PORT = /dev/ttyUSB0

AVRDUDE_WRITE_FLASH = -D -U flash:w:$(BUILDDIR)/$(PROJECT).hex

# Check if the counter cycle erase must be performed after device programming.
ifeq ($(USE_AVRDUDE_ERASE_COUNTER),yes)
	AVRDUDE_ERASE_COUNTER = -y
endif

# Check if a verification must be performed after device programming.
ifeq ($(USE_AVRDUDE_NO_VERIFY),no)
	AVRDUDE_NO_VERIFY = -V
endif

# Check verbosity level activation.
ifeq ($(USE_VERBOSE_PROGRAMMATION),yes)
	AVRDUDE_VERBOSE = -v -v
endif

# AVR programmer flags for AVRDUDE programmer.
ifeq ($(AVR_PROGRAMMER),$(AVRDUDE_PROGRAMMER))
AVRDUDE_FLAGS =  -p $(MCU)
AVRDUDE_FLAGS += -P $(AVRDUDE_PORT)
AVRDUDE_FLAGS += -b 115200
AVRDUDE_FLAGS += -c $(AVRDUDE_PROGRAMMER_ID)
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)
endif

# AVR programmer flags for DFU programmer.
ifeq ($(AVR_PROGRAMMER),$(DFU_PROGRAMMER))
DFU_WRITE_FLASH = flash --force
DFU_ERASE_FLASH = erase
DFU_RESET       = reset
endif

# AVR programmer flags for MICRONUCLEUS programmer.
ifeq ($(AVR_PROGRAMMER),$(MICRONUCLEUS_PROGRAMMER))
MICRONUCLEUS_TIMEOUT_ARG = --timeout 60
MICRONUCLEUS_RUN_ARG  = --run
MICRONUCLEUS_TYPE_ARG = --type raw
MICRONUCLEUS_DUMP_PROGRESS = --dump-progress
MICRONUCLEUS_FLAGS =  $(MICRONUCLEUS_TYPE_ARG)
MICRONUCLEUS_FLAGS += $(MICRONUCLEUS_TIMEOUT_ARG)
MICRONUCLEUS_FLAGS += $(MICRONUCLEUS_RUN_ARG)
endif

#
# End of Programming Options.
##############################################################################

##############################################################################
# Include file.
#

RULESPATH = $(CHIBIOS)/os/common/ports/AVR/compilers/GCC
include $(RULESPATH)/rules.mk

#
# End of include file.
##############################################################################

##############################################################################
# Programming rules
#

# AVRDUDE programming rules.
ifeq ($(AVR_PROGRAMMER),$(AVRDUDE_PROGRAMMER))
program: $(BUILDDIR)/$(PROJECT).hex
	@echo
	@echo Programming $(MCU) device.
	$(AVR_PROGRAMMER) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $<
	@echo Done.
endif

# DFU programming rules.
ifeq ($(AVR_PROGRAMMER),$(DFU_PROGRAMMER))
program: $(BUILDDIR)/$(PROJECT).hex
	@echo
	@echo Programming $(MCU) device.
	$(AVR_PROGRAMMER) $(MCU) $(DFU_WRITE_FLASH) $<
	$(AVR_PROGRAMMER) $(MCU) $(DFU_RESET)
	@echo Done.

erase:
	@echo
	@echo Erasing $(MCU) device.
	$(AVR_PROGRAMMER) $(MCU) $(DFU_ERASE_FLASH)
	@echo Done.
endif

# MICRONUCLEUS programming rules.
ifeq ($(AVR_PROGRAMMER),$(MICRONUCLEUS_PROGRAMMER))
program: $(BUILDDIR)/$(PROJECT).bin
	@echo
	@echo Programming $(MCU) device.
	$(AVR_PROGRAMMER) $(MICRONUCLEUS_FLAGS) $<
	@echo Done.
endif

#
# End of programming rules.
##############################################################################

# EOF
