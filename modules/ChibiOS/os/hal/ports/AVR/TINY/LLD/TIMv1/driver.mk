PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/TINY/LLD/TIMv1/hal_st_lld.c

ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_GPT TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/TINY/LLD/TIMv1/hal_gpt_lld.c
endif
ifneq ($(findstring HAL_USE_ICU TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/TINY/LLD/TIMv1/hal_icu_lld.c
endif
ifneq ($(findstring HAL_USE_PWM TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/TINY/LLD/TIMv1/hal_pwm_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/TINY/LLD/TIMv1/hal_gpt_lld.c
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/TINY/LLD/TIMv1/hal_icu_lld.c
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/AVR/TINY/LLD/TIMv1/hal_pwm_lld.c
endif

PLATFORMINC += $(CHIBIOS)/os/hal/ports/AVR/TINY/LLD/TIMv1
