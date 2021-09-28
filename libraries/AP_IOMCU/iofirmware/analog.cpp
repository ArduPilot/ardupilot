/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  analog capture for IOMCU. This uses direct register access to avoid
  using up a DMA channel and to minimise latency. We capture a single
  sample at a time
 */

#include "ch.h"
#include "hal.h"
#include "analog.h"

#if HAL_USE_ADC != TRUE
#error "HAL_USE_ADC must be set"
#endif

/*
  initialise ADC capture
 */
void adc_init(void)
{
    adc_lld_init();
    rccEnableADC1(true);

    /* set channels 4 and 5 for 28.5us sample time */
    ADC1->SMPR2 = ADC_SMPR2_SMP_AN4(ADC_SAMPLE_28P5) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_28P5);

    /* capture a single sample at a time */
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
}

/*
  capture one sample on a channel
 */
static uint16_t adc_sample_channel(uint32_t channel)
{
    // clear EOC
    ADC1->SR = 0;

    /* capture one sample */
    ADC1->SQR3 = channel;
    ADC1->CR2 |= ADC_CR2_ADON;

    /* wait for the conversion to complete */
    uint32_t counter = 16;

    while (!(ADC1->SR & ADC_SR_EOC)) {
        if (--counter == 0) {
            // ensure EOC is clear
            ADC1->SR = 0;
            return 0xffff;
        }
    }

    // return sample (this also clears EOC flag)
    return ADC1->DR;
}

/*
  capture VSERVO in mV
 */
uint16_t adc_sample_vservo(void)
{
    const uint32_t channel = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN4);
    return adc_sample_channel(channel);
}

/*
  capture VRSSI in mV
 */
uint16_t adc_sample_vrssi(void)
{
    const uint32_t channel = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN5);
    return adc_sample_channel(channel);
}
