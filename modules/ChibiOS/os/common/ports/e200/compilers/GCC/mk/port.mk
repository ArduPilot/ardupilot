# List of the ChibiOS/RT e200 generic port files.
PORTSRC = $(CHIBIOS)/os/common/ports/e200/chcore.c
          
PORTASM = $(CHIBIOS)/os/common/ports/e200/compilers/GCC/ivor.S \
          $(CHIBIOS)/os/common/ports/e200/compilers/GCC/chcoreasm.S

PORTINC = $(CHIBIOS)/os/common/portability/GCC \
          $(CHIBIOS)/os/common/ports/e200 \
          $(CHIBIOS)/os/common/ports/e200/compilers/GCC

# Shared variables
ALLXASMSRC += $(PORTASM)
ALLCSRC    += $(PORTSRC)
ALLINC     += $(PORTINC)
