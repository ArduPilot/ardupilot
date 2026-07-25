ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_TRNG TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/STM32/LLD/RNGv1/hal_trng_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/STM32/LLD/RNGv1/hal_trng_lld.c
endif

PLATFORMINC += $(CHIBIOS)/os/hal/ports/STM32/LLD/RNGv1
