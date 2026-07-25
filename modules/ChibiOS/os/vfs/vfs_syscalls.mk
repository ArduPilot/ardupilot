# List of all the ChibiOS/VFS syscall files.
VFSSYSSRC := $(CHIBIOS)/os/vfs/various/syscalls.c \

# Required include directories
VFSSYSINC := $(CHIBIOS)/os/vfs/various

# Shared variables
ALLCSRC += $(VFSSYSSRC)
ALLINC  += $(VFSSYSINC)
