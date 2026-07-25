ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_SPI TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/SPIv1/hal_spi_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/SPIv1/hal_spi_lld.c
endif

PLATFORMINC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/SPIv1
