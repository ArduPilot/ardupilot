# List of the ChibiOS/RT e200 generic port files.
PORTSRC = $(CHIBIOS)/os/common/ports/e200/chcore.c
          
PORTASM = $(CHIBIOS)/os/common/ports/e200/compilers/GHS/ivor.s \
          $(CHIBIOS)/os/common/ports/e200/compilers/GHS/chcoreasm.s

PORTINC = $(CHIBIOS)/os/common/ports/e200 \
          $(CHIBIOS)/os/common/ports/e200/compilers/GHS

# Shared variables
ALLASMSRC  += $(PORTASM)
ALLCSRC    += $(PORTSRC)
ALLINC     += $(PORTINC)
