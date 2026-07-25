# List of the ChibiOS e200z0 SPC560BCxx startup files.
STARTUPSRC =
          
STARTUPASM = $(CHIBIOS)/os/common/startup/e200/devices/SPC560BCxx/boot_ghs.s \
             $(CHIBIOS)/os/common/startup/e200/compilers/GHS/vectors.s \
             $(CHIBIOS)/os/common/startup/e200/compilers/GHS/crt0.s

STARTUPINC = ${CHIBIOS}/os/common/startup/e200/compilers/GHS \
             ${CHIBIOS}/os/common/startup/e200/devices/SPC560BCxx

STARTUPLD  = ${CHIBIOS}/os/common/startup/e200/compilers/GHS/ld

# Shared variables
ALLASMSRC  += $(STARTUPASM)
ALLCSRC    += $(STARTUPSRC)
ALLINC     += $(STARTUPINC)
