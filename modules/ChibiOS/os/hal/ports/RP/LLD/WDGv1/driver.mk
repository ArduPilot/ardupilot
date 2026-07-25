ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_WDG TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/RP/LLD/WDGv1/hal_wdg_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/RP/LLD/WDGv1/hal_wdg_lld.c
endif

PLATFORMINC += $(CHIBIOS)/os/hal/ports/RP/LLD/WDGv1
