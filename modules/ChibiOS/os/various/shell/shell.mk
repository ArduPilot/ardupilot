# RT Shell files.
SHELLSRC = $(CHIBIOS)/os/various/shell/shell.c \
           $(CHIBIOS)/os/various/shell/shell_cmd.c

SHELLINC = $(CHIBIOS)/os/various/shell

# Shared variables
ALLCSRC += $(SHELLSRC)
ALLINC  += $(SHELLINC)
