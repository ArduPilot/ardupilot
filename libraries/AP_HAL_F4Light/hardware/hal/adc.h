/**
 *  @file adc.h
 *
 *  @brief Analog-to-Digital Conversion (ADC) header.
 */

#ifndef _ADC_H_
#define _ADC_H_

#include "stm32.h"
#include "hal_types.h"

/** ADC device type. */
typedef struct adc_dev {
   ADC_TypeDef *adcx;
} adc_dev;

#ifdef __cplusplus
extern "C"{
#endif

extern const adc_dev* const _ADC1;
extern const adc_dev* const _ADC2;
extern const adc_dev* const _ADC3;

extern const adc_dev _adc1;
extern const adc_dev _adc2;
extern const adc_dev _adc3;

#define ADC_Channel_0                               ((uint8_t)0x00)
#define ADC_Channel_1                               ((uint8_t)0x01)
#define ADC_Channel_2                               ((uint8_t)0x02)
#define ADC_Channel_3                               ((uint8_t)0x03)
#define ADC_Channel_4                               ((uint8_t)0x04)
#define ADC_Channel_5                               ((uint8_t)0x05)
#define ADC_Channel_6                               ((uint8_t)0x06)
#define ADC_Channel_7                               ((uint8_t)0x07)
#define ADC_Channel_8                               ((uint8_t)0x08)
#define ADC_Channel_9                               ((uint8_t)0x09)
#define ADC_Channel_10                              ((uint8_t)0x0A)
#define ADC_Channel_11                              ((uint8_t)0x0B)
#define ADC_Channel_12                              ((uint8_t)0x0C)
#define ADC_Channel_13                              ((uint8_t)0x0D)
#define ADC_Channel_14                              ((uint8_t)0x0E)
#define ADC_Channel_15                              ((uint8_t)0x0F)
#define ADC_Channel_16                              ((uint8_t)0x10)
#define ADC_Channel_17                              ((uint8_t)0x11)
#define ADC_Channel_18                              ((uint8_t)0x12)

#define ADC_Channel_TempSensor                      ((uint8_t)ADC_Channel_16)
#define ADC_Channel_Vrefint                         ((uint8_t)ADC_Channel_17)
#define ADC_Channel_Vbat                            ((uint8_t)ADC_Channel_18)

#define ADC_SampleTime_3Cycles                    ((uint8_t)0x00)
#define ADC_SampleTime_15Cycles                   ((uint8_t)0x01)
#define ADC_SampleTime_28Cycles                   ((uint8_t)0x02)
#define ADC_SampleTime_56Cycles                   ((uint8_t)0x03)
#define ADC_SampleTime_84Cycles                   ((uint8_t)0x04)
#define ADC_SampleTime_112Cycles                  ((uint8_t)0x05)
#define ADC_SampleTime_144Cycles                  ((uint8_t)0x06)
#define ADC_SampleTime_480Cycles                  ((uint8_t)0x07)

#define SMPR_SMP_SET             ((uint32_t)0x00000007)  
#define SQR_SQ_SET               ((uint32_t)0x0000001F)
#define SQR1_L_RESET             ((uint32_t)0xFE0FFFFF)


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

static inline void adc_channel_config(const adc_dev *dev, uint8_t channel, uint8_t rank, uint8_t sampleTime)
{
  /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
  if (channel > ADC_Channel_9)  {
    uint32_t tmpreg1 = dev->adcx->SMPR1 & ~(SMPR_SMP_SET << (3 * (channel - 10)));
    dev->adcx->SMPR1 = tmpreg1 |    (uint32_t)sampleTime << (3 * (channel - 10));
  }  else  {/* channel include in ADC_Channel_[0..9] */
    uint32_t tmpreg1 = dev->adcx->SMPR2 & ~(SMPR_SMP_SET << (3 * channel));
    dev->adcx->SMPR2 = tmpreg1 |    (uint32_t)sampleTime << (3 * channel);
  }

  if (rank < 7) {
    uint32_t tmpreg1 = dev->adcx->SQR3 & ~(SQR_SQ_SET << (5 * (rank - 1)));
    dev->adcx->SQR3 = tmpreg1 |     (uint32_t)channel << (5 * (rank - 1));
  } else if (rank < 13)  { /* For Rank 7 to 12 */
    uint32_t tmpreg1 = dev->adcx->SQR2 & ~(SQR_SQ_SET << (5 * (rank - 7)));
    dev->adcx->SQR2 = tmpreg1 |     (uint32_t)channel << (5 * (rank - 7));
  }  else  { /* For Rank 13 to 16 */
    uint32_t tmpreg1 = dev->adcx->SQR1 & ~(SQR_SQ_SET << (5 * (rank - 13)));
    dev->adcx->SQR1 = tmpreg1 |     (uint32_t)channel << (5 * (rank - 13));
  }
}
  
/**
 * @brief Enable an adc peripheral
 * @param dev ADC device to enable
 */
static inline void adc_enable(const adc_dev *dev) {
    /* Enable ADCx */
    /* Set the ADON bit to wake up the ADC from power down mode */
    dev->adcx->CR2 |= (uint32_t)ADC_CR2_ADON;
}

/**
 * @brief Disable an ADC peripheral
 * @param dev ADC device to disable
 */
static inline void adc_disable(const adc_dev *dev) {
    dev->adcx->CR2 &= (uint32_t)(~ADC_CR2_ADON);
}

/**
 * @brief Disable all ADC peripherals.
 */
static inline void adc_disable_all(void) {
    adc_foreach(adc_disable);
}

static inline void adc_start_conv(const adc_dev *dev)
{
  /* Enable the selected ADC conversion for regular group */
  dev->adcx->CR2 |= (uint32_t)ADC_CR2_SWSTART;
}

static inline void adc_vref_enable(){
    /* Enable the temperature sensor and Vrefint channel*/
    ADC->CCR |= (uint32_t)ADC_CCR_TSVREFE;
}

static inline void adc_vref_disable(){
    /* Disable the temperature sensor and Vrefint channel*/
    ADC->CCR &= (uint32_t)(~ADC_CCR_TSVREFE);
}


#ifdef __cplusplus
} // extern "C"
#endif

#endif
