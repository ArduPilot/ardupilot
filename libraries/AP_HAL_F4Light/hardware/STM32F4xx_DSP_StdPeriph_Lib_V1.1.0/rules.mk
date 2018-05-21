# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)

LIBRARY_INCLUDES += -I$(STM32_PATH)/Libraries/CMSIS/Include
LIBRARY_INCLUDES += -I$(STM32_PATH)/Libraries/CMSIS/Device/ST/STM32F4xx/Include

# Local flags
CFLAGS_$(d) =

# Local rules and targets
cSRCS_$(d) := 

sSRCS_$(d) := 

cFILES_$(d) := $(cSRCS_$(d):%=$(d)/%)
sFILES_$(d) := $(sSRCS_$(d):%=$(d)/%)

OBJS_$(d)	:= $(cFILES_$(d):%.c=$(LIBRARIES_PATH)/%.o)
OBJS_$(d)	+= $(sFILES_$(d):%.s=$(LIBRARIES_PATH)/%.o)
DEPS_$(d) 	:= $(OBJS_$(d):%.o=%.d)

$(OBJS_$(d)): TGT_CFLAGS := $(CFLAGS_$(d))
$(OBJS_$(d)): TGT_ASFLAGS :=

TGT_BIN += $(OBJS_$(d))

# Standard things
-include        $(DEPS_$(d))
d               := $(dirstack_$(sp))
sp              := $(basename $(sp))
