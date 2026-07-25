# List of the ChibiOS/RT SIMIA32 port files.
PORTSRC = ${CHIBIOS}/os/common/ports/SIMIA32/chcore.c

PORTASM = 

PORTINC = $(CHIBIOS)/os/common/portability/GCC \
          ${CHIBIOS}/os/common/ports/SIMIA32/compilers/GCC \
          ${CHIBIOS}/os/common/ports/SIMIA32

# Shared variables
ALLXASMSRC += $(PORTASM)
ALLCSRC    += $(PORTSRC)
ALLINC     += $(PORTINC)
