# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)

#LIBRARY_INCLUDES += 

OSD := AP_HAL_F4Light/hardware/osd

# Local flags
CFLAGS_$(d) := -Wall -Werror -Wno-missing-declarations -Wno-shadow -Wunused-function -Wno-unused-variable -Wno-missing-declarations -I$(OSD)/osd_core


cppSRCS_$(d) :=  
cppSRCS_$(d) += $(OSD)/osd.cpp
cppSRCS_$(d) += $(OSD)/osd_eeprom.cpp
cppSRCS_$(d) += $(OSD)/osd_core/OSD_Max7456.cpp

# Local rules and targets
cSRCS_$(d)   :=  
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
