# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)

LIBRARY_INCLUDES += -I$(STM32USB_PATH)

# Local flags
CFLAGS_$(d) =

STM_DIR := AP_HAL_F4Light/hardware/STM32_USB_Driver

# Local rules and targets
cSRCS_$(d) := 
cSRCS_$(d) += $(STM_DIR)/usb_core.c
cSRCS_$(d) += $(STM_DIR)/usb_dcd.c
cSRCS_$(d) += $(STM_DIR)/usb_dcd_int.c
cSRCS_$(d) += $(STM_DIR)/usbd_cdc_core.c
cSRCS_$(d) += $(STM_DIR)/usbd_core.c
cSRCS_$(d) += $(STM_DIR)/usbd_ioreq.c
cSRCS_$(d) += $(STM_DIR)/usbd_req.c

sSRCS_$(d)   := 
cppSRCS_$(d) := 

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
