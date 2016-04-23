# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)/Libraries/STM32F4xx_StdPeriph_Driver/src
BUILDDIRS       += $(BUILD_PATH)/$(d)/Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc_ride7

LIBRARY_INCLUDES += -I$(STM32_PATH)/Libraries/STM32F4xx_StdPeriph_Driver/inc
LIBRARY_INCLUDES += -I$(STM32_PATH)/Libraries/CMSIS/Include
LIBRARY_INCLUDES += -I$(STM32_PATH)/Libraries/CMSIS/Device/ST/STM32F4xx/Include

STM_DIR := AP_HAL_REVOMINI/hardware/STM32F4xx_DSP_StdPeriph_Lib_V1.1.0

# Local flags
CFLAGS_$(d) =

# Local rules and targets
cSRCS_$(d) := 
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/misc.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_aes.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_des.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_tdes.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_md5.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_sha1.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c
cSRCS_$(d) += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c

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
