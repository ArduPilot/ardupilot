# List of the ChibiOS e200z0 SPC560Bxx startup files.
STARTUPSRC =
          
STARTUPASM = $(CHIBIOS)/os/common/startup/e200/devices/SPC560Bxx/boot.S \
             $(CHIBIOS)/os/common/startup/e200/compilers/GCC/vectors.S \
             $(CHIBIOS)/os/common/startup/e200/compilers/GCC/crt0.S

STARTUPINC = $(CHIBIOS)/os/common/portability/GCC \
             ${CHIBIOS}/os/common/startup/e200/compilers/GCC \
             ${CHIBIOS}/os/common/startup/e200/devices/SPC560Bxx

STARTUPLD  = ${CHIBIOS}/os/common/startup/e200/compilers/GCC/ld

# Shared variables
ALLXASMSRC += $(STARTUPASM)
ALLCSRC    += $(STARTUPSRC)
ALLINC     += $(STARTUPINC)
