# List of the ChibiOS generic sandbox startup files.
STARTUPSRC =
          
STARTUPASM = $(CHIBIOS)/os/common/startup/ARMCMx-SB/compilers/GCC/crt0.S

STARTUPINC = $(CHIBIOS)/os/common/portability/GCC \
             $(CHIBIOS)/os/common/startup/ARMCMx-SB/compilers/GCC

STARTUPLD  = $(CHIBIOS)/os/common/startup/ARMCMx-SB/compilers/GCC/ld

# Shared variables
ALLXASMSRC += $(STARTUPASM)
ALLCSRC    += $(STARTUPSRC)
ALLINC     += $(STARTUPINC)
