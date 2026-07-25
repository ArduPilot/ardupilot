# List of the ChibiOS ARMv8M-MainLine generic port files.
PORTSRC = $(CHIBIOS)/os/common/ports/ARMv8-M-ML/chcore.c
          
PORTASM = $(CHIBIOS)/os/common/ports/ARMv8-M-ML/compilers/GCC/chcoreasm.S

PORTINC = $(CHIBIOS)/os/common/portability/GCC \
          $(CHIBIOS)/os/common/ports/ARM-common \
          $(CHIBIOS)/os/common/ports/ARMv8-M-ML

# Shared variables
ALLXASMSRC += $(PORTASM)
ALLCSRC    += $(PORTSRC)
ALLINC     += $(PORTINC)
