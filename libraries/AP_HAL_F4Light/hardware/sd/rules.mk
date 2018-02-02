# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)

#LIBRARY_INCLUDES += 

# Local flags
# always include board #defines
CFLAGS_$(d) := -Wall -Werror -I$(HARDWARE_PATH)/sd  -I$(HARDWARE_PATH)/sd/FatFs -Wno-trigraphs

DIR := AP_HAL_F4Light/hardware/sd

# Local rules and targets

cppSRCS_$(d) := 
cppSRCS_$(d) += $(DIR)/SD.cpp
cppSRCS_$(d) += $(DIR)/SdFatFs.cpp
cppSRCS_$(d) += $(DIR)/Sd2Card.cpp

cSRCS_$(d)   :=
cSRCS_$(d)   += $(DIR)/FatFs/diskio.c
cSRCS_$(d)   += $(DIR)/FatFs/ff.c
cSRCS_$(d)   += $(DIR)/FatFs/drivers/sd.c

sSRCS_$(d)   := 

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
