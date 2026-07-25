# List of the ChibiOS/RT ARM generic port files.
PORTSRC = ${CHIBIOS}/os/common/ports/ARM/chcore.c

PORTASM = $(CHIBIOS)/os/common/ports/ARM/compilers/GCC/chcoreasm.S

PORTINC = $(CHIBIOS)/os/common/portability/GCC \
          $(CHIBIOS)/os/common/ports/ARM-common \
          ${CHIBIOS}/os/common/ports/ARM \
          ${CHIBIOS}/os/common/ports/ARM/compilers/GCC

# Shared variables
ALLXASMSRC += $(PORTASM)
ALLCSRC    += $(PORTSRC)
ALLINC     += $(PORTINC)
