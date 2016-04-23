# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)
BUILDDIRS       += $(BUILD_PATH)/$(d)/comm
BUILDDIRS       += $(BUILD_PATH)/$(d)/boards/$(BOARD)

LIBRARY_INCLUDES += -I$(d)/comm -I$(d)/boards/$(BOARD)

# Local flags
CFLAGS_$(d) := -Wall -Werror

WIR := AP_HAL_REVOMINI/wirish

# Local rules and targets
cSRCS_$(d)   :=  
cSRCS_$(d)   += $(WIR)/boards/$(BOARD)/system_stm32f4xx.c

cppSRCS_$(d) := 
cppSRCS_$(d) += $(WIR)/boards/$(BOARD)/$(BOARD).cpp
#cppSRCS_$(d) += comm/BetterStream.cpp
#cppSRCS_$(d) += comm/FastSerial.cpp
#cppSRCS_$(d) += comm/HardwareI2C.cpp
#cppSRCS_$(d) += comm/HardwareSPI.cpp
cppSRCS_$(d) += $(WIR)/boards.cpp
#cppSRCS_$(d) += cxxabi-compat.cpp
cppSRCS_$(d) += $(WIR)/ext_interrupts.cpp
cppSRCS_$(d) += $(WIR)/HardwareTimer.cpp
#cppSRCS_$(d) += Print.cpp
#cppSRCS_$(d) += pwm.cpp
#cppSRCS_$(d) += wirish_analog.cpp
#cppSRCS_$(d) += wirish_digital.cpp
#cppSRCS_$(d) += wirish_math.cpp
#cppSRCS_$(d) += wirish_time.cpp

sSRCS_$(d)   := 
sSRCS_$(d)   += $(WIR)/boards/$(BOARD)/$(BOARD)_startup.S

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
