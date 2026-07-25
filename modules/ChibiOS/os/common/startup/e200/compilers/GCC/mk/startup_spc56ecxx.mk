# List of the ChibiOS e200z4 SPC56ECxx startup files.
STARTUPSRC =
          
STARTUPASM = $(CHIBIOS)/os/common/startup/e200/devices/SPC56ECxx/boot.S \
             $(CHIBIOS)/os/common/startup/e200/compilers/GCC/vectors.S \
             $(CHIBIOS)/os/common/startup/e200/compilers/GCC/crt0.S

STARTUPINC = $(CHIBIOS)/os/common/portability/GCC \
             ${CHIBIOS}/os/common/startup/e200/compilers/GCC \
             ${CHIBIOS}/os/common/startup/e200/devices/SPC56ECxx

STARTUPLD  = ${CHIBIOS}/os/common/startup/e200/compilers/GCC/ld

# Shared variables
ALLXASMSRC += $(STARTUPASM)
ALLCSRC    += $(STARTUPSRC)
ALLINC     += $(STARTUPINC)
