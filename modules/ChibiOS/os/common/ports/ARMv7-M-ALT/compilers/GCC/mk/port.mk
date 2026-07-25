# List of the ChibiOS/RT ARMv7M generic port files.
PORTSRC = $(CHIBIOS)/os/common/ports/ARMv7-M-ALT/chcore.c \
          $(CHIBIOS)/os/common/ports/ARMv7-M-ALT/chcoreapi.c
          
PORTASM = $(CHIBIOS)/os/common/ports/ARMv7-M-ALT/compilers/GCC/chcoreasm.S

PORTINC = $(CHIBIOS)/os/common/portability/GCC \
          $(CHIBIOS)/os/common/ports/ARM-common \
          $(CHIBIOS)/os/common/ports/ARMv7-M-ALT

# Shared variables
ALLXASMSRC += $(PORTASM)
ALLCSRC    += $(PORTSRC)
ALLINC     += $(PORTINC)
