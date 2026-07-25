ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_SIO TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/RP/LLD/UARTv1/hal_sio_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/RP/LLD/UARTv1/hal_sio_lld.c
endif

PLATFORMINC += $(CHIBIOS)/os/hal/ports/RP/LLD/UARTv1
