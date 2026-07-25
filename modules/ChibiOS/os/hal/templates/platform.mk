# List of all the template platform files.
ifeq ($(USE_SMART_BUILD),yes)

# Configuration files directory
ifeq ($(CONFDIR),)
  CONFDIR = .
endif

HALCONF := $(strip $(shell cat $(CONFDIR)/halconf.h | egrep -e "\#define"))

PLATFORMSRC := ${CHIBIOS}/os/hal/templates/hal_lld.c \
               ${CHIBIOS}/os/hal/templates/hal_st_lld.c
ifneq ($(findstring HAL_USE_ADC TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_adc_lld.c
endif
ifneq ($(findstring HAL_USE_CAN TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_can_lld.c
endif
ifneq ($(findstring HAL_USE_CRY TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_crypto_lld.c
endif
ifneq ($(findstring HAL_USE_DAC TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_dac_lld.c
endif
ifneq ($(findstring HAL_USE_EFL TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_efl_lld.c
endif
ifneq ($(findstring HAL_USE_GPT TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_gpt_lld.c
endif
ifneq ($(findstring HAL_USE_I2C TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_i2c_lld.c
endif
ifneq ($(findstring HAL_USE_I2S TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_i2s_lld.c
endif
ifneq ($(findstring HAL_USE_ICU TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_icu_lld.c
endif
ifneq ($(findstring HAL_USE_MAC TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_mac_lld.c
endif
ifneq ($(findstring HAL_USE_PAL TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_pal_lld.c
endif
ifneq ($(findstring HAL_USE_PWM TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_pwm_lld.c
endif
ifneq ($(findstring HAL_USE_RTC TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_rtc_lld.c
endif
ifneq ($(findstring HAL_USE_SDC TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_sdc_lld.c
endif
ifneq ($(findstring HAL_USE_SERIAL TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_serial_lld.c
endif
ifneq ($(findstring HAL_USE_SIO TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_sio_lld.c
endif
ifneq ($(findstring HAL_USE_SPI TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_spi_v2_lld.c
endif
ifneq ($(findstring HAL_USE_TRNG TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_trng_lld.c
endif
ifneq ($(findstring HAL_USE_UART TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_uart_lld.c
endif
ifneq ($(findstring HAL_USE_USB TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_usb_lld.c
endif
ifneq ($(findstring HAL_USE_WDG TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_wdg_lld.c
endif
ifneq ($(findstring HAL_USE_WSPI TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS}/os/hal/templates/hal_wspi_lld.c
endif
else
PLATFORMSRC = ${CHIBIOS}/os/hal/templates/hal_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_adc_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_can_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_crypto_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_dac_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_efl_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_gpt_lld.c \
 			  ${CHIBIOS}/os/hal/templates/hal_i2c_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_i2s_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_icu_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_mac_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_pal_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_pwm_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_rtc_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_sdc_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_serial_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_sio_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_spi_v2_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_st_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_trng_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_uart_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_usb_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_wdg_lld.c \
              ${CHIBIOS}/os/hal/templates/hal_wspi_lld.c
endif

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/templates

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
