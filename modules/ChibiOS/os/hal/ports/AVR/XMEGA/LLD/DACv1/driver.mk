ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_DAC TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DACv1/hal_dac_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DACv1/hal_dac_lld.c
endif

PLATFORMINC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DACv1
