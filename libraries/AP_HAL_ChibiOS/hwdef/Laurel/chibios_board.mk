# ############################################################################# Build global options NOTE: Standalone Laurel RP2350 board makefile.
# #############################################################################

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
# ArduPilot provides ff_memalloc/ff_memfree in hwdef/common/malloc.c.
ALLCSRC := $(filter-out $(CHIBIOS)/os/various/fatfs_bindings/fatfs_syscall.c, $(ALLCSRC))
endif

# ############################################################################# Architecture or project specific options #############################################################################

HWDEF = $(AP_HAL)/hwdef

ifeq ($(USE_PROCESS_STACKSIZE),)
	USE_PROCESS_STACKSIZE = 0x400
endif

ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
	USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# RP2350 has an FPv5 single-precision FPU, use hard-float ABI.
ifeq ($(USE_FPU),)
	USE_FPU = hard
endif

ifeq ($(USE_FPU_OPT),)
	USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-sp-d16
endif

# ############################################################################# Project, target, sources and paths #############################################################################

PROJECT = ch

MCU  = cortex-m33

include $(CHIBIOS)/$(CHIBIOS_STARTUP_MK)
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/$(CHIBIOS_PLATFORM_MK)
include $(CHIBIOS)/os/hal/boards/RP_PICO2_RP2350/board.mk

# Use ArduPilot's board.c so __late_init() runs before C++ constructors.
ALLCSRC := $(filter-out $(CHIBIOS)/os/hal/boards/RP_PICO2_RP2350/board.c,$(ALLCSRC))

include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
# SMP port: includes chcoresmp.c (hardware spinlock + inter-core FIFO) and
# sets PORT_CORES_NUMBER=2.  CH_CFG_SMP_MODE=TRUE activates spinlocks in
# chSysLock/chSysUnlock for real dual-core operation.
include $(CHIBIOS)/os/common/ports/ARMv8-M-ML-ALT/compilers/GCC/mk/port_rp2.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk

ifeq ($(USE_FATFS),yes)
include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk
include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk
endif

# ArduPilot provides better ff_memalloc/ff_memfree in hwdef/common/malloc.c.
ALLCSRC := $(filter-out $(CHIBIOS)/os/various/fatfs_bindings/fatfs_syscall.c, $(ALLCSRC))

LDSCRIPT= $(STARTUPLD)/RP2350_FLASH.ld

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
				$(HWDEF)/Laurel/c1_main.c

ifeq ($(USE_USB_MSD),yes)
CSRC += $(CHIBIOS)/os/various/scsi_bindings/lib_scsi.c \
				$(CHIBIOS)/os/hal/src/hal_usb_msd.c
endif

ifneq ($(CRASHCATCHER),)
LIBCC_CSRC = $(CRASHCATCHER)/Core/src/CrashCatcher.c \
						 $(HWDEF)/common/crashdump.c

LIBCC_ASMXSRC = $(CRASHCATCHER)/Core/src/CrashCatcher_armv7m.S
endif

CPPSRC = $(sort $(ALLCPPSRC))
ACSRC =
ACPPSRC =
TCSRC =
TCPPSRC =
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

# ############################################################################# Compiler settings #############################################################################

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
LD   = $(TRGT)gcc
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

AOPT =
TOPT = -mthumb -DTHUMB

CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

# ############################################################################# Start of user section #############################################################################

UDEFS = $(ENV_UDEFS) $(FATFS_FLAGS) -DHAL_BOARD_NAME=\"$(HAL_BOARD_NAME)\" \
				-DHAL_MAX_STACK_FRAME_SIZE=$(HAL_MAX_STACK_FRAME_SIZE) \
				-DCRT0_EXTRA_CORES_NUMBER=1 -DRP2350B_QFN80=1 \
				-DHAL_ENABLE_THREAD_STATISTICS

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

UINCDIR =
ULIBDIR =
ULIBS =

# ############################################################################# End of user defines #############################################################################

include $(HWDEF)/common/chibios_common.mk

# ############################################################################# Custom rules #############################################################################

PICOTOOL = picotool/picotool

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