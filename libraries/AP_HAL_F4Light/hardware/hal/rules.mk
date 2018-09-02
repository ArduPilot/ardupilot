# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)

LIBRARY_INCLUDES += -I$(d)/common -I$(d)/include
WIR := AP_HAL_F4Light/wirish

# Local flags
CFLAGS_$(d) := -Wall 
# -Werror

STM_DIR := AP_HAL_F4Light/hardware/hal

# Local rules and targets
cSRCS_$(d)   := 
cSRCS_$(d)   += $(STM_DIR)/exti.c
cSRCS_$(d)   += $(STM_DIR)/gpio_hal.c
cSRCS_$(d)   += $(STM_DIR)/i2c.c
cSRCS_$(d)   += $(STM_DIR)/spi.c
cSRCS_$(d)   += $(STM_DIR)/syscalls.c
cSRCS_$(d)   += $(STM_DIR)/systick.c
cSRCS_$(d)   += $(STM_DIR)/timer.c
cSRCS_$(d)   += $(STM_DIR)/usart.c
cSRCS_$(d)   += $(STM_DIR)/adc.c
cSRCS_$(d)   += $(STM_DIR)/stopwatch.c
cSRCS_$(d)   += $(STM_DIR)/usb.c
cSRCS_$(d)   += $(STM_DIR)/pwm_in.c
cSRCS_$(d)   += $(STM_DIR)/dma.c
cSRCS_$(d)   += $(STM_DIR)/nvic.c

cppSRCS_$(d) := 

sSRCS_$(d)   := 

cFILES_$(d)   := $(cSRCS_$(d):%=$(d)/%)
cppFILES_$(d) := $(cppSRCS_$(d):%=$(d)/%)
sFILES_$(d)   := $(sSRCS_$(d):%=$(d)/%)

OBJS_$(d)	:= $(cFILES_$(d):%.c=$(LIBRARIES_PATH)/%.o)
OBJS_$(d)	+= $(cppFILES_$(d):%.cpp=$(LIBRARIES_PATH)/%.o)
OBJS_$(d)	+= $(sFILES_$(d):%.s=$(LIBRARIES_PATH)/%.o)

DEPS_$(d) 	:= $(OBJS_$(d):%.o=%.d)

$(OBJS_$(d)): TGT_CFLAGS := $(CFLAGS_$(d))
$(OBJS_$(d)): TGT_ASFLAGS :=

TGT_BIN += $(OBJS_$(d))

# Standard things
-include        $(DEPS_$(d))
d               := $(dirstack_$(sp))
sp              := $(basename $(sp))
