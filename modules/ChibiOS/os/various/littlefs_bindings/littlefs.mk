# LittleFS files.
LITTLEFSSRC = $(CHIBIOS)/os/various/littlefs_bindings/lfs_hal.c \
              $(CHIBIOS)/ext/littlefs/lfs.c \
              $(CHIBIOS)/ext/littlefs/lfs_util.c

LITTLEFSINC = $(CHIBIOS)/os/various/littlefs_bindings \
              $(CHIBIOS)/ext/littlefs

DDEFS      += -DLFS_THREADSAFE=1 -DLFS_NO_DEBUG=0 -DLFS_CONFIG=lfs_config.h

# Shared variables
ALLCSRC += $(LITTLEFSSRC)
ALLINC  += $(LITTLEFSINC)
