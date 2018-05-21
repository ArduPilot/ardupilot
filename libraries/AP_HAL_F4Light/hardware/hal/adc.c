/*

(c) 2017 night_ghost@ykoctpa.ru
 

based on: LeafLabs

*/

#pragma GCC optimize ("O2")

#include <hal.h>
#include "adc.h"
#include <stdbool.h>

const adc_dev _adc1 = {
    .regs   = ADC1,
};
/** ADC1 device. */
const adc_dev* const _ADC1 = &_adc1;

const adc_dev _adc2 = {
    .regs   = ADC2,
};
/** ADC2 device. */
const adc_dev* const _ADC2 = &_adc2;

const adc_dev const _adc3 = {
    .regs   = ADC3,
};
/** ADC3 device. */
const adc_dev* const _ADC3 = &_adc3;

__IO uint16_t 	ADC_ConvertedValue;
__IO bool adc_data_ready;

/**
 * @brief Call a function on all ADC devices.
 * @param fn Function to call on each ADC device.
 */
void adc_foreach(void (*fn)(const adc_dev*)) 
{
    fn(_ADC1);
    fn(_ADC2);
    fn(_ADC3);
}

/**
 * @brief Initialize an ADC peripheral.
 *
 * Initializes the RCC clock line for the given peripheral.  Resets
 * ADC device registers.
 *
 * @param dev ADC peripheral to initialize
 */
 
#define CR_CLEAR_MASK             ((uint32_t)0xFFFC30E0)  
#define CR1_CLEAR_MASK            ((uint32_t)0xFCFFFEFF)
#define CR2_CLEAR_MASK            ((uint32_t)0xC0FFF7FD)
 
void adc_init(const adc_dev *dev) {
    RCC_doAPB2_reset(RCC_APB2_bit_ADC); // turn on clock and do reset to all ADCs
      
    uint32_t tmp = ADC->CCR & CR_CLEAR_MASK; // Clear MULTI, DELAY, DMA and ADCPRE bits

    // Multi mode, Delay between two sampling time, ADC prescaler, DMA access mode for multimode 
    ADC->CCR = tmp | ADC_Mode_Independent | ADC_Prescaler_Div4 | ADC_DMAAccessMode_Disabled | ADC_TwoSamplingDelay_5Cycles;

#define ADC_DEFAULT_Nconv 1

    // Get the ADCx CR1 value
    tmp = dev->regs->CR1 & CR1_CLEAR_MASK;  // Clear RES and SCAN bits 
    dev->regs->CR1 = tmp | ADC_Resolution_12b; // scan conversion mode and resolution 

    tmp = dev->regs->CR2 & CR2_CLEAR_MASK; // Clear CONT, ALIGN, EXTEN and EXTSEL bits 
    // external trigger event and edge, data alignment and continuous conversion mode
    dev->regs->CR2 = tmp | ADC_DataAlign_Right | ADC_ExternalTrigConv_T1_CC1 | ADC_ExternalTrigConvEdge_None | DISABLE << 1;

    tmp = dev->regs->SQR1 & SQR1_L_RESET;   // Clear L bits
    dev->regs->SQR1 =tmp | ((uint32_t)(ADC_DEFAULT_Nconv - 1) << 20); // regular channel sequence length 
}

