# VFS subsystem build.

# Dependencies.
include $(CHIBIOS)/os/common/oop/oop.mk

# Required files.
VFSSRC := $(CHIBIOS)/os/vfs/src/vfspaths.c \
          $(CHIBIOS)/os/vfs/src/vfsparser.c \
          $(CHIBIOS)/os/vfs/src/vfsbuffers.c \
          $(CHIBIOS)/os/vfs/src/vfsdrivers.c \
          $(CHIBIOS)/os/vfs/src/vfsnodes.c \
          $(CHIBIOS)/os/vfs/src/vfs.c \
          $(CHIBIOS)/os/vfs/drivers/tmplfs/drvtmplfs.c \
          $(CHIBIOS)/os/vfs/drivers/chfs/drvchfs.c \
          $(CHIBIOS)/os/vfs/drivers/fatfs/drvfatfs.c \
          $(CHIBIOS)/os/vfs/drivers/littlefs/drvlittlefs.c \
          $(CHIBIOS)/os/vfs/drivers/overlay/drvoverlay.c \
          $(CHIBIOS)/os/vfs/drivers/streams/drvstreams.c

# Required include directories
VFSINC := $(CHIBIOS)/os/common/include \
          $(CHIBIOS)/os/vfs/include \
          $(CHIBIOS)/os/vfs/drivers/tmplfs \
          $(CHIBIOS)/os/vfs/drivers/chfs \
          $(CHIBIOS)/os/vfs/drivers/fatfs \
          $(CHIBIOS)/os/vfs/drivers/littlefs \
          $(CHIBIOS)/os/vfs/drivers/overlay \
          $(CHIBIOS)/os/vfs/drivers/streams

# Shared variables
ALLCSRC += $(VFSSRC)
ALLINC  += $(VFSINC)
