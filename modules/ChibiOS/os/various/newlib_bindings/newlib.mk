# NewLib files.
NEWLIBSSRC = $(CHIBIOS)/os/various/newlib_bindings/syscalls.c

NEWLIBINC  = $(CHIBIOS)/os/various/newlib_bindings

# Shared variables
ALLCSRC += $(NEWLIBSSRC)
ALLINC  += $(NEWLIBINC)
