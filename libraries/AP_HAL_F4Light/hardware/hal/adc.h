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
   ADC_TypeDef *regs;
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

#define ADC_Channel_0                               (0)
#define ADC_Channel_1                               (1)
#define ADC_Channel_2                               (2)
#define ADC_Channel_3                               (3)
#define ADC_Channel_4                               (4)
#define ADC_Channel_5                               (5)
#define ADC_Channel_6                               (6)
#define ADC_Channel_7                               (7)
#define ADC_Channel_8                               (8)
#define ADC_Channel_9                               (9)
#define ADC_Channel_10                              (10)
#define ADC_Channel_11                              (11)
#define ADC_Channel_12                              (12)
#define ADC_Channel_13                              (13)
#define ADC_Channel_14                              (14)
#define ADC_Channel_15                              (15)
#define ADC_Channel_16                              (16)
#define ADC_Channel_17                              (17)
#define ADC_Channel_18                              (18)

#define ADC_Channel_TempSensor                      (ADC_Channel_16)
#define ADC_Channel_Vrefint                         (ADC_Channel_17)
#define ADC_Channel_Vbat                            (ADC_Channel_18)

#define ADC_SampleTime_3Cycles                    (0)
#define ADC_SampleTime_15Cycles                   (1)
#define ADC_SampleTime_28Cycles                   (2)
#define ADC_SampleTime_56Cycles                   (3)
#define ADC_SampleTime_84Cycles                   (4)
#define ADC_SampleTime_112Cycles                  (5)
#define ADC_SampleTime_144Cycles                  (6)
#define ADC_SampleTime_480Cycles                  (7)

#define SMPR_SMP_SET             (0x00000007)  
#define SQR_SQ_SET               (0x0000001F)
#define SQR1_L_RESET             (0xFE0FFFFF)

#define ADC_Mode_Independent                       (0x00)       
#define ADC_DualMode_RegSimult_InjecSimult         (0x01)
#define ADC_DualMode_RegSimult_AlterTrig           (0x02)
#define ADC_DualMode_InjecSimult                   (0x05)
#define ADC_DualMode_RegSimult                     (0x06)
#define ADC_DualMode_Interl                        (0x07)
#define ADC_DualMode_AlterTrig                     (0x09)
#define ADC_TripleMode_RegSimult_InjecSimult       (0x11)
#define ADC_TripleMode_RegSimult_AlterTrig         (0x12)
#define ADC_TripleMode_InjecSimult                 (0x15)
#define ADC_TripleMode_RegSimult                   (0x16)
#define ADC_TripleMode_Interl                      (0x17)
#define ADC_TripleMode_AlterTrig                   (0x19)

#define ADC_Prescaler_Div2                         (0L<<16)
#define ADC_Prescaler_Div4                         (1L<<16)
#define ADC_Prescaler_Div6                         (2L<<16)
#define ADC_Prescaler_Div8                         (3L<<16)

#define ADC_DMAAccessMode_Disabled              (0L<<14)     // DMA mode disabled
#define ADC_DMAAccessMode_1                     (1L<<14)     // DMA mode 1: 2 / 3 half-words one by one - 1 then 2 then 3
#define ADC_DMAAccessMode_2                     (2L<<14)     // DMA mode 2: 2 / 3 half-words by pairs - 2&1 then 1&3 then 3&2
#define ADC_DMAAccessMode_3                     (3L<<14)     // DMA mode 3: 2 / 3 bytes by pairs - 2&1 then 1&3 then 3&2

#define ADC_TwoSamplingDelay_5Cycles               (0L<<8)
#define ADC_TwoSamplingDelay_6Cycles               (1L<<8)
#define ADC_TwoSamplingDelay_7Cycles               (2L<<8)
#define ADC_TwoSamplingDelay_8Cycles               (3L<<8)
#define ADC_TwoSamplingDelay_9Cycles               (4L<<8)
#define ADC_TwoSamplingDelay_10Cycles              (5L<<8)
#define ADC_TwoSamplingDelay_11Cycles              (6L<<8)
#define ADC_TwoSamplingDelay_12Cycles              (7L<<8)
#define ADC_TwoSamplingDelay_13Cycles              (8L<<8)
#define ADC_TwoSamplingDelay_14Cycles              (9L<<8)
#define ADC_TwoSamplingDelay_15Cycles              (10L<<8)
#define ADC_TwoSamplingDelay_16Cycles              (11L<<8)
#define ADC_TwoSamplingDelay_17Cycles              (12L<<8)
#define ADC_TwoSamplingDelay_18Cycles              (13L<<8)
#define ADC_TwoSamplingDelay_19Cycles              (14L<<8)
#define ADC_TwoSamplingDelay_20Cycles              (15L<<8)

//cr1
#define ADC_Resolution_12b                         (0L<<24)
#define ADC_Resolution_10b                         (1L<<24)
#define ADC_Resolution_8b                          (2L<<24)
#define ADC_Resolution_6b                          (3L<<24)

//cr2
#define ADC_ExternalTrigConvEdge_None              (0L<<28)
#define ADC_ExternalTrigConvEdge_Rising            (1L<<28)
#define ADC_ExternalTrigConvEdge_Falling           (2L<<28)
#define ADC_ExternalTrigConvEdge_RisingFalling     (3L<<28)


#define ADC_ExternalTrigConv_T1_CC1                (0L<<24)
#define ADC_ExternalTrigConv_T1_CC2                (1L<<24)
#define ADC_ExternalTrigConv_T1_CC3                (2L<<24)
#define ADC_ExternalTrigConv_T2_CC2                (3L<<24)
#define ADC_ExternalTrigConv_T2_CC3                (4L<<24)
#define ADC_ExternalTrigConv_T2_CC4                (5L<<24)
#define ADC_ExternalTrigConv_T2_TRGO               (6L<<24)
#define ADC_ExternalTrigConv_T3_CC1                (7L<<24)
#define ADC_ExternalTrigConv_T3_TRGO               (8L<<24)
#define ADC_ExternalTrigConv_T4_CC4                (9L<<24)
#define ADC_ExternalTrigConv_T5_CC1                (10L<<24)
#define ADC_ExternalTrigConv_T5_CC2                (11L<<24)
#define ADC_ExternalTrigConv_T5_CC3                (12L<<24)
#define ADC_ExternalTrigConv_T8_CC1                (13L<<24)
#define ADC_ExternalTrigConv_T8_TRGO               (14L<<24)
#define ADC_ExternalTrigConv_Ext_IT11              (15L<<24)


#define ADC_DataAlign_Right                        (0)
#define ADC_DataAlign_Left                         (1L<<11)

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
    uint32_t tmp = dev->regs->SQR1 & SQR1_L_RESET; // Clear L bits
    dev->regs->SQR1 = tmp | ((uint32_t)(length - 1) << 20); // regular channel sequence length
}

static inline void adc_channel_config(const adc_dev *dev, uint8_t channel, uint8_t rank, uint8_t sampleTime)
{
  /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
  if (channel > ADC_Channel_9)  {
    uint32_t tmp = dev->regs->SMPR1 & ~(SMPR_SMP_SET << (3 * (channel - 10)));
    dev->regs->SMPR1 = tmp |    (uint32_t)sampleTime << (3 * (channel - 10));
  }  else  {/* channel include in ADC_Channel_[0..9] */
    uint32_t tmp = dev->regs->SMPR2 & ~(SMPR_SMP_SET << (3 * channel));
    dev->regs->SMPR2 = tmp |    (uint32_t)sampleTime << (3 * channel);
  }

    uint32_t pos;
    volatile uint32_t *reg;
  
    if (rank < 7) {     // 1..6
        pos = rank - 1;
        reg = &dev->regs->SQR3;
    } else if (rank < 13)  { // 7..12
        pos = rank - 7;
        reg = &dev->regs->SQR2;
    }  else  {            // 13..16
        pos = rank - 13;
        reg = &dev->regs->SQR1;
    }

    uint32_t tmp = *reg & ~(SQR_SQ_SET << (5 * pos));
    *reg = tmp |     (uint32_t)channel << (5 * pos);
}
  
static inline void adc_enable(const adc_dev *dev) {  // enable ADCx device 
    dev->regs->CR2 |= (uint32_t)ADC_CR2_ADON;
}

static inline void adc_disable(const adc_dev *dev) { // disable ADCx device 
    dev->regs->CR2 &= (uint32_t)(~ADC_CR2_ADON);
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
  dev->regs->CR2 |= (uint32_t)ADC_CR2_SWSTART;
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
