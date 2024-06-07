#pragma once

#include <stdint.h>
/*
  initialise adc
 */
void adc_init(void);

/*
  capture VSERVO
 */
uint16_t adc_vservo(void);

/*
  capture VRSSI
 */
uint16_t adc_vrssi(void);

/* start another update */
void adc_sample_channels(void);

/* capture VRSSI */
void adc_enable_vrssi(void);

/* don't capture VRSSI */
void adc_disable_vrssi(void);
