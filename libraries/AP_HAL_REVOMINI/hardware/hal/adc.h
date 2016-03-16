/**
 *  @file adc.h
 *
 *  @brief Analog-to-Digital Conversion (ADC) header.
 */

#ifndef _ADC_H_
#define _ADC_H_

#include "stm32.h"

/** ADC device type. */
typedef struct adc_dev {
   ADC_TypeDef *adcx;
} adc_dev;

#ifdef __cplusplus
extern "C"{
#endif

extern adc_dev* const _ADC1;
extern adc_dev* const _ADC2;
extern adc_dev* const _ADC3;

void adc_init(const adc_dev *dev);
//void adc_set_extsel(const adc_dev *dev, adc_extsel_event event);
void adc_foreach(void (*fn)(const adc_dev*));
//void adc_set_sample_rate(const adc_dev *dev, adc_smp_rate smp_rate);
uint16_t adc_read(const adc_dev *dev, uint8_t channel);
uint16_t vref_read(void);
uint16_t temp_read(void);

/**
 * @brief Set the regular channel sequence length.
 *
 * Defines the total number of conversions in the regular channel
 * conversion sequence.
 *
 * @param dev ADC device.
 * @param length Regular channel sequence length, from 1 to 16.
 */
static inline void adc_set_reg_seqlen(const adc_dev *dev, uint8_t length) {
	/* ADC L Mask */
	#define SQR1_L_RESET              ((uint32_t)0xFE0FFFFF)

	uint32_t tmpreg1 = 0;
	uint8_t tmpreg2 = 0;

	/* Get the ADCx SQR1 value */
	tmpreg1 = dev->adcx->SQR1;
	/* Clear L bits */
	tmpreg1 &= SQR1_L_RESET;
	/* Configure ADCx: regular channel sequence length */
	/* Set L bits according to ADC_NbrOfConversion value */ 
	tmpreg2 |= (uint8_t)(length - (uint8_t)1);
	tmpreg1 |= ((uint32_t)tmpreg2 << 20);
	/* Write to ADCx SQR1 */
	dev->adcx->SQR1 = tmpreg1;
}

/**
 * @brief Enable an adc peripheral
 * @param dev ADC device to enable
 */
static inline void adc_enable(const adc_dev *dev) {
    /* Enable ADCx */
    ADC_Cmd(dev->adcx, ENABLE);
}

/**
 * @brief Disable an ADC peripheral
 * @param dev ADC device to disable
 */
static inline void adc_disable(const adc_dev *dev) {
	ADC_Cmd(dev->adcx, DISABLE);
}

/**
 * @brief Disable all ADC peripherals.
 */
static inline void adc_disable_all(void) {
    adc_foreach(adc_disable);
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif
