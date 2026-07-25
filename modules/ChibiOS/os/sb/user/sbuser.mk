# List of the ChibiOS ARMv7-M sandbox user files.
SBUSERSRC = $(CHIBIOS)/os/sb/user/sbuser.c
          
SBUSERASM =

SBUSERINC = $(CHIBIOS)/os/sb/common \
            $(CHIBIOS)/os/sb/user

# Shared variables
ALLXASMSRC += $(SBUSERASM)
ALLCSRC    += $(SBUSERSRC)
ALLINC     += $(SBUSERINC)
