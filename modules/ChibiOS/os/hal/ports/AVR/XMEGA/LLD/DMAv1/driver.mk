
# No need of the smart build for trhis file for the moment.
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DMAv1/xmega_dma_lld.c

PLATFORMINC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DMAv1

#ifeq ($(USE_SMART_BUILD),yes)
#ifneq ($(findstring HAL_USE_PAL TRUE,$(HALCONF)),)
#PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DMAv1/xmega_dma_lld.c
#endif
#else
#PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/GPIOv1/xmega_dma_lld.c
#endif

#PLATFORMINC += $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DMAv1
