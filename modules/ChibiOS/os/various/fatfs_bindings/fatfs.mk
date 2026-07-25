# FATFS files.
FATFSSRC = $(CHIBIOS)/os/various/fatfs_bindings/fatfs_diskio.c \
           $(CHIBIOS)/os/various/fatfs_bindings/fatfs_syscall.c \
           $(CHIBIOS)/ext/fatfs/source/ff.c \
           $(CHIBIOS)/ext/fatfs/source/ffunicode.c

FATFSINC = $(CHIBIOS)/ext/fatfs/source

# Shared variables
ALLCSRC += $(FATFSSRC)
ALLINC  += $(FATFSINC)
