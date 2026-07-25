# List of the ChibiOS e200z0 SPC560Bxx startup files.
STARTUPSRC =
          
STARTUPASM = $(CHIBIOS)/os/common/startup/e200/devices/SPC560Bxx/boot_ghs.s \
             $(CHIBIOS)/os/common/startup/e200/compilers/GHS/vectors.s \
             $(CHIBIOS)/os/common/startup/e200/compilers/GHS/crt0.s

STARTUPINC = ${CHIBIOS}/os/common/startup/e200/compilers/GHS \
             ${CHIBIOS}/os/common/startup/e200/devices/SPC560Bxx

STARTUPLD  = ${CHIBIOS}/os/common/startup/e200/compilers/GHS/ld

# Shared variables
ALLASMSRC  += $(STARTUPASM)
ALLCSRC    += $(STARTUPSRC)
ALLINC     += $(STARTUPINC)
