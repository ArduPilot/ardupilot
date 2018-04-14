# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)


# Local flags
CFLAGS_$(d) := -Wall -Werror


MS := AP_HAL_F4Light/hardware/massstorage

# Local rules and targets
cppSRCS_$(d) :=
cppSRCS_$(d) += $(MS)/mass_storage.cpp

cSRCS_$(d) :=
cSRCS_$(d) += $(MS)/StorageMode.c
cSRCS_$(d) += $(MS)/device_desc.c
cSRCS_$(d) += $(MS)/msc/usbd_msc_core.c
cSRCS_$(d) += $(MS)/msc/usbd_msc_bot.c
cSRCS_$(d) += $(MS)/msc/usbd_msc_data.c
cSRCS_$(d) += $(MS)/msc/usbd_msc_scsi.c

sSRCS_$(d) :=

cFILES_$(d)   := $(cSRCS_$(d):%=$(d)/%)
cppFILES_$(d) := $(cppSRCS_$(d):%=$(d)/%)
sFILES_$(d)   := $(sSRCS_$(d):%=$(d)/%)

OBJS_$(d)     := $(cFILES_$(d):%.c=$(LIBRARIES_PATH)/%.o)
OBJS_$(d)     += $(cppFILES_$(d):%.cpp=$(LIBRARIES_PATH)/%.o)
OBJS_$(d)     += $(sFILES_$(d):%.S=$(LIBRARIES_PATH)/%.o)

DEPS_$(d)     := $(OBJS_$(d):%.o=%.d)


$(OBJS_$(d)): TGT_CFLAGS := $(CFLAGS_$(d))

TGT_BIN += $(OBJS_$(d))

# Standard things
-include        $(DEPS_$(d))
d               := $(dirstack_$(sp))
sp              := $(basename $(sp))


