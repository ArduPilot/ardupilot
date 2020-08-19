#pragma once

#include <stdint.h>
/*
  initialise adc
 */
void adc_init(void);

/*
  capture VSERVO
 */
uint16_t adc_sample_vservo(void);

/*
  capture VRSSI
 */
uint16_t adc_sample_vrssi(void);
