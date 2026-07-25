# List of the ChibiOS e200z3 SPC563Mxx startup files.
STARTUPSRC =
          
STARTUPASM = $(CHIBIOS)/os/common/startup/e200/devices/SPC563Mxx/boot.S \
             $(CHIBIOS)/os/common/startup/e200/compilers/GCC/vectors.S \
             $(CHIBIOS)/os/common/startup/e200/compilers/GCC/crt0.S

STARTUPINC = $(CHIBIOS)/os/common/portability/GCC \
             ${CHIBIOS}/os/common/startup/e200/compilers/GCC \
             ${CHIBIOS}/os/common/startup/e200/devices/SPC563Mxx

STARTUPLD  = ${CHIBIOS}/os/common/startup/e200/compilers/GCC/ld

# Shared variables
ALLXASMSRC += $(STARTUPASM)
ALLCSRC    += $(STARTUPSRC)
ALLINC     += $(STARTUPINC)
