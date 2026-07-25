ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_WSPI TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/STM32/LLD/QUADSPIv2/hal_wspi_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/STM32/LLD/QUADSPIv2/hal_wspi_lld.c
endif

PLATFORMINC += $(CHIBIOS)/os/hal/ports/STM32/LLD/QUADSPIv2
