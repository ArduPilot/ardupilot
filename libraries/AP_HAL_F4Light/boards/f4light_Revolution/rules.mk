# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)
#BUILDDIRS       += $(BUILD_PATH)/$(d)/comm
BUILDDIRS       += $(BUILD_PATH)/$(d)/../boards/$(BOARD)

WIR    := AP_HAL_F4Light/wirish
BRD    := AP_HAL_F4Light/boards

LIBRARY_INCLUDES += -I$(BRD)/$(BOARD)

# Local flags
# always include board #defines
CFLAGS_$(d) := -Wall -Werror -include $(BRD)/$(BOARD)/board.h


# Local rules and targets
cSRCS_$(d)   :=  
cSRCS_$(d)   += $(BRD)/$(BOARD)/system_stm32f4xx.c # C startup code

cppSRCS_$(d) := 
cppSRCS_$(d) += $(BRD)/$(BOARD)/board.cpp
cppSRCS_$(d) += $(WIR)/boards.cpp

sSRCS_$(d)   := 
sSRCS_$(d)   += $(WIR)/startup.S           # early startup code
sSRCS_$(d)   += $(WIR)/exc.S               # exception handling and task switching code


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

