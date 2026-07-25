# List of the ChibiOS ARMv7-M sandbox host files.
SBHOSTSRC = $(CHIBIOS)/os/sb/host/sbhost.c \
			$(CHIBIOS)/os/sb/host/sbapi.c \
			$(CHIBIOS)/os/sb/host/sbposix.c
          
SBHOSTASM = $(CHIBIOS)/os/sb/host/compilers/GCC/sbexc.S

SBHOSTINC = $(CHIBIOS)/os/sb/common \
            $(CHIBIOS)/os/sb/host

# Shared variables
ALLXASMSRC += $(SBHOSTASM)
ALLCSRC    += $(SBHOSTSRC)
ALLINC     += $(SBHOSTINC)
