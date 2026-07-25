ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_CRY TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/CRYPv1/hal_crypto_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/CRYPv1/hal_crypto_lld.c
endif

PLATFORMINC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/CRYPv1
