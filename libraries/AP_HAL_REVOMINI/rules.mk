# Standard things
sp := $(sp).x
dirstack_$(sp) := $(d)
d := $(dir)
BUILDDIRS += $(BUILD_PATH)/$(d)

# Local flags
CFLAGS_$(d) := 

# Local rules and targets
cSRCS_$(d) :=

cppSRCS_$(d) :=
cppSRCS_$(d) += AnalogIn.cpp
cppSRCS_$(d) += AnalogSource.cpp
cppSRCS_$(d) += GPIO.cpp
cppSRCS_$(d) += HAL_REVOMINI_Class.cpp
cppSRCS_$(d) += I2CDriver.cpp
cppSRCS_$(d) += RCInput.cpp
cppSRCS_$(d) += RCOutput.cpp
cppSRCS_$(d) += Scheduler.cpp
cppSRCS_$(d) += Semaphores.cpp
cppSRCS_$(d) += SPIDevice_SPI1.cpp
cppSRCS_$(d) += SPIDevice_SPI2.cpp
cppSRCS_$(d) += SPIDevice_SPI3.cpp
cppSRCS_$(d) += SPIDeviceManager.cpp
cppSRCS_$(d) += Storage.cpp
cppSRCS_$(d) += UARTDriver.cpp
cppSRCS_$(d) += EEPROM.cpp

cFILES_$(d) := $(cSRCS_$(d):%=$(d)/%)
cppFILES_$(d) := $(cppSRCS_$(d):%=$(d)/%)

OBJS_$(d) := $(cFILES_$(d):%.c=$(BUILD_PATH)/%.o) \
             $(cppFILES_$(d):%.cpp=$(BUILD_PATH)/%.o)
DEPS_$(d) := $(OBJS_$(d):%.o=%.d)

$(OBJS_$(d)): TGT_CFLAGS := $(CFLAGS_$(d))

TGT_BIN += $(OBJS_$(d))

# Standard things
-include $(DEPS_$(d))
d := $(dirstack_$(sp))
sp := $(basename $(sp))