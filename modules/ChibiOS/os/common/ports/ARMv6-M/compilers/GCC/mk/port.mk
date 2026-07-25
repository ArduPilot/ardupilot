# List of the ChibiOS/RT Cortex-M0 port files.
PORTSRC = $(CHIBIOS)/os/common/ports/ARMv6-M/chcore.c
          
PORTASM = $(CHIBIOS)/os/common/ports/ARMv6-M/compilers/GCC/chcoreasm.S

PORTINC = $(CHIBIOS)/os/common/portability/GCC \
          $(CHIBIOS)/os/common/ports/ARM-common \
          $(CHIBIOS)/os/common/ports/ARMv6-M

# Shared variables
ALLXASMSRC += $(PORTASM)
ALLCSRC    += $(PORTSRC)
ALLINC     += $(PORTINC)

