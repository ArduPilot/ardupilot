# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)


DIR := AP_HAL_F4Light/hardware/4way

# Local flags
# always include board #defines
CFLAGS_$(d) := -Wall -Werror 


# Local rules and targets
cSRCS_$(d)   :=  

cppSRCS_$(d) := 
cppSRCS_$(d) += $(DIR)/serial_4way.cpp
cppSRCS_$(d) += $(DIR)/serial_4way_avrootloader.cpp
cppSRCS_$(d) += $(DIR)/serial_4way_stk500v2.cpp

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
