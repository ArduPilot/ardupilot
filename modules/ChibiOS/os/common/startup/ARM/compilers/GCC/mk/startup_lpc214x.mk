# List of the ChibiOS generic LPC214x file.
STARTUPSRC = $(CHIBIOS)/os/common/startup/ARM/compilers/GCC/crt1.c

STARTUPASM = $(CHIBIOS)/os/common/startup/ARM/compilers/GCC/vectors.S \
             $(CHIBIOS)/os/common/startup/ARM/compilers/GCC/crt0.S

STARTUPINC = $(CHIBIOS)/os/common/portability/GCC \
             ${CHIBIOS}/os/common/startup/ARM/devices/LPC214x

STARTUPLD  = ${CHIBIOS}/os/common/startup/ARM/compilers/GCC/ld

# Shared variables
ALLXASMSRC += $(STARTUPASM)
ALLCSRC    += $(STARTUPSRC)
ALLINC     += $(STARTUPINC)
