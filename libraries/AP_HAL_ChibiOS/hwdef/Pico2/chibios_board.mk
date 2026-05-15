# ############################################################################# Build global options NOTE: Can be overridden externally.- started with ardupilot../common/chibios_board.mk and merged bits from git master ChibiOS/ChibiOS/demos/RP/RT-RP2350-BLINK/Makefile by buzz

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O3 -ggdb -fomit-frame-pointer -falign-functions=16 -DCRT0_AREAS_NUMBER=0
endif

ifeq ($(ENABLE_DEBUG_SYMBOLS), yes)
  #USE_OPT += -g3
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -std=gnu++11
endif

# Assembly specific options here (added to USE_OPT).
ifeq ($(USE_ASOPT),)
  USE_ASOPT = 
endif

# Assembly specific options here (added to USE_ASXOPT).
ifeq ($(USE_ASXOPT),)
  USE_ASXOPT =
endif

# Enable this if you want the linker to remove unused code and data.
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable this if you want link time optimizations (LTO).
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = no
endif

include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk
ifeq ($(USE_FATFS),yes)
include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk
endif

# Build global options #############################################################################

# ############################################################################# Architecture or project specific options
HWDEF = $(AP_HAL)/hwdef
# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU (no, softfp, hard).
# RP2350 has a full FPv5-D16 FPU (hardware double), so use 'hard' ABI so float/double arguments are passed in FPU registers, eliminating the integer-register round-trip overhead present with 'softfp'.
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# FPU-related options.
# Keep fpv5-sp-d16 (single-precision).
# the RP2350 Cortex-M33 implements FPv5-SP only
# causing hard faults.
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-sp-d16
endif

# Architecture or project specific options #############################################################################

# ############################################################################# Project, target, sources and paths

# Define project name here
PROJECT = ch

# Target settings.
MCU  = cortex-m33

# Imported source files and paths

# Licensing files.
#include $(CHIBIOS)/os/license/license.mk - done below
# Startup files. CHIBIOS_STARTUP_MK=os/common/startup/ARMCMx/compilers/GCC/mk/startup_rp2350.mk set correct in PICO2.py
include $(CHIBIOS)/$(CHIBIOS_STARTUP_MK)
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
# CHIBIOS_PLATFORM_MK=os/hal/ports/RP/RP2350/platform.mk set correct in PICO2.py
include $(CHIBIOS)/$(CHIBIOS_PLATFORM_MK)
# this line is extra compared to stm32..
include $(CHIBIOS)/os/hal/boards/RP_PICO2_RP2350/board.mk
# Override: use ArduPilot's board.c (which defines strong __late_init calling halInit and chSysInit before C++ global constructors run).
# Without this the weak crt1.o stub would be used and ChibiOS never inits.
ALLCSRC := $(filter-out $(CHIBIOS)/os/hal/boards/RP_PICO2_RP2350/board.c,$(ALLCSRC))

include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
# SMP port: includes chcoresmp.c (hardware spinlock + inter-core FIFO) and
# sets PORT_CORES_NUMBER=2.  CH_CFG_SMP_MODE=TRUE activates spinlocks in
# chSysLock/chSysUnlock for real dual-core operation.
include $(CHIBIOS)/os/common/ports/ARMv8-M-ML-ALT/compilers/GCC/mk/port_rp2.mk
# Other files (optional).
#include $(CHIBIOS)/test/rt/test.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk

ifeq ($(USE_FATFS),yes)
include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk
include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk
endif


# Define linker script file here For RAM execution (useful for debugging): LDSCRIPT= $(STARTUPLD)/RP2350_RAM.ld For flash (XIP) execution:
LDSCRIPT= $(STARTUPLD)/RP2350_FLASH.ld



# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.

CSRC = $(sort $(ALLCSRC))

# hal_safety.c is provided by current ChibiOS hal/xhal make logic.

CSRC += $(HWDEF)/common/stubs.c \
        $(HWDEF)/common/board.c \
        $(HWDEF)/common/board_rp2350.c \
        $(HWDEF)/common/usbcfg.c \
        $(HWDEF)/common/usbcfg_dualcdc.c \
        $(HWDEF)/common/usbcfg_common.c \
        $(HWDEF)/common/flash.c \
        $(HWDEF)/common/malloc.c \
        $(HWDEF)/common/hrt.c \
        $(HWDEF)/common/stm32_util.c \
        $(HWDEF)/common/bouncebuffer.c \
        $(HWDEF)/common/watchdog.c \
        $(HWDEF)/common/sysperf.c \
        $(HWDEF)/Pico2/c1_main.c

ifeq ($(USE_USB_MSD),yes)
CSRC += $(CHIBIOS)/os/various/scsi_bindings/lib_scsi.c \
        $(CHIBIOS)/os/hal/src/hal_usb_msd.c
endif

#	   $(TESTSRC) \
#	   test.c
ifneq ($(CRASHCATCHER),)
LIBCC_CSRC = $(CRASHCATCHER)/Core/src/CrashCatcher.c \
             $(HWDEF)/common/crashdump.c

LIBCC_ASMXSRC = $(CRASHCATCHER)/Core/src/CrashCatcher_armv7m.S
endif

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(sort $(ALLCPPSRC))

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

INCDIR = $(CHIBIOS)/os/license \
         $(ALLINC) $(HWDEF)/common

ifneq ($(CRASHCATCHER),)
INCDIR += $(CRASHCATCHER)/include
endif

ifeq ($(USE_USB_MSD),yes)
INCDIR += $(CHIBIOS)/os/various/scsi_bindings
endif

# Project, sources and paths #############################################################################

# ############################################################################# Compiler settings

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# these two are non-error on rp2350 as we somtimes redefine eg HAL_USE_SERIAL_USB from true to false in hwdef.dat

# Define C warning options here
#CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes -Werror
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here
#CPPWARN = -Wall -Wextra -Wundef -Werror
CPPWARN = -Wall -Wextra -Wundef

# Compiler settings #############################################################################

# ############################################################################# Start of user section

# List all user C define here, like -D_DEBUG=1
UDEFS = $(ENV_UDEFS) $(FATFS_FLAGS) -DHAL_BOARD_NAME=\"$(HAL_BOARD_NAME)\" \
  -DHAL_MAX_STACK_FRAME_SIZE=$(HAL_MAX_STACK_FRAME_SIZE) \
  -DCRT0_EXTRA_CORES_NUMBER=1

# Single-core demo: disable core 1 startup code
#UDEFS = -DCRT0_VTOR_INIT=1 -DNDEBUG

ifeq ($(ENABLE_ASSERTS),yes)
 UDEFS += -DHAL_CHIBIOS_ENABLE_ASSERTS
 ASXFLAGS += -DHAL_CHIBIOS_ENABLE_ASSERTS
endif

ifeq ($(ENABLE_MALLOC_GUARD),yes)
 UDEFS += -DHAL_CHIBIOS_ENABLE_MALLOC_GUARD
 ASXFLAGS += -DHAL_CHIBIOS_ENABLE_MALLOC_GUARD
endif

ifeq ($(ENABLE_STATS),yes)
 UDEFS += -DHAL_ENABLE_THREAD_STATISTICS
 ASXFLAGS += -DHAL_ENABLE_THREAD_STATISTICS
endif

ifneq ($(AP_BOARD_START_TIME),)
 UDEFS += -DAP_BOARD_START_TIME=$(AP_BOARD_START_TIME)
endif

# SMP: generate _crt0_c1_entry trampoline for bare-metal core1 FIFO dispatcher.
UADEFS = -DCRT0_EXTRA_CORES_NUMBER=1

ifeq ($(COPY_VECTORS_TO_RAM),yes)
 UADEFS += -DCRT0_INIT_VECTORS=1
endif

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

# End of user defines ############################################################################# ############################################################################# Common rules
include $(HWDEF)/common/chibios_common.mk
# Common rules #############################################################################

# ############################################################################# Custom rules

# Picotool binary location in ./
PICOTOOL = picotool/picotool
# Need picotool https://github.com/raspberrypi/picotool, picotool binaries: https://github.com/raspberrypi/pico-sdk-tools/releases
# Download and extract picotool binary if not present, on linux x86_64
$(PICOTOOL):
	@echo "Downloading picotool..."
	wget -q https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.2.0-3/picotool-2.2.0-a4-x86_64-lin.tar.gz
	@echo "Extracting picotool..."
	tar -xzf picotool-2.2.0-a4-x86_64-lin.tar.gz
	@echo "Picotool ready at $(PICOTOOL)"

uf2: $(BUILDDIR)/$(PROJECT).elf $(PICOTOOL)
	$(PICOTOOL) uf2 convert $(BUILDDIR)/$(PROJECT).elf $(BUILDDIR)/$(PROJECT).uf2


upload: uf2
	$(info Uploading UF2 file... please press the BOOTSEL button on the board while connecting the USB cable in the next 5 seconds)
	@sleep 5
	$(PICOTOOL) load -v -x $(BUILDDIR)/$(PROJECT).uf2 -f
	$(info UF2 file uploaded)
# u can alternatively just copy the uf2 file to the RPI-RP2350 mass storage device if it exists. it only appears when the board is in BOOTSEL mode
#     @cp $(BUILDDIR)/$(PROJECT).uf2 /media/$(USER)/RP2350




# Custom rules #############################################################################
