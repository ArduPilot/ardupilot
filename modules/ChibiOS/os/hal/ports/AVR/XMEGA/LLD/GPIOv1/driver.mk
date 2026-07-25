ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_PAL TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/GPIOv1/hal_pal_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/GPIOv1/hal_pal_lld.c
endif

PLATFORMINC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/GPIOv1
