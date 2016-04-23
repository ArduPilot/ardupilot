#include <hal.h>
#include "adc.h"
#include "gpiopins.h"
#include <stdbool.h>

static adc_dev adc1 = {
    .adcx   = ADC1,
};
/** ADC1 device. */
adc_dev* const _ADC1 = &adc1;

static adc_dev adc2 = {
    .adcx   = ADC2,
};
/** ADC2 device. */
adc_dev* const _ADC2 = &adc2;

adc_dev adc3 = {
    .adcx   = ADC3,
};
/** ADC3 device. */
adc_dev* const _ADC3 = &adc3;

__IO uint16_t 	ADC_ConvertedValue, T_StartupTimeDelay;
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
void adc_init(const adc_dev *dev) {
	/* Enable The HSI */
	RCC_HSICmd(ENABLE);

	 /* Check that HSI oscillator is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
 
	if (dev->adcx == ADC1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	else if (dev->adcx == ADC2)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	else
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE); 

  ADC_DeInit();
    
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  /* ADC Common Init **********************************************************/
  ADC_CommonStructInit(&ADC_CommonInitStructure);
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  /* ADCx Init ****************************************************************/
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(dev->adcx, &ADC_InitStructure);
     
}

/**
 * @brief Perform a single synchronous software triggered conversion on a
 * channel.
 * @param dev ADC device to use for reading.
 * @param channel channel to convert
 * @return conversion result
 */
uint16_t adc_read(const adc_dev *dev, uint8_t channel)
{
  adc_data_ready = false;
  
  adc_disable(dev);
 
  /* ADC regular channel14 configuration */
  ADC_RegularChannelConfig(dev->adcx, channel, 1, ADC_SampleTime_56Cycles);
  adc_enable(dev);
      
  /* Start ADC Software Conversion */
  ADC_SoftwareStartConv(dev->adcx);
 
  /* Wait until ADC Channel end of conversion */  
  while (ADC_GetFlagStatus(dev->adcx, ADC_FLAG_EOC) == RESET);
  
  /* Read ADC conversion result */
  return ADC_GetConversionValue(dev->adcx);
}

uint16_t temp_read(void)
{
  uint8_t i;
  uint16_t res, T_StartupTimeDelay;
 
  ADC_TempSensorVrefintCmd(ENABLE);
  /* Wait until ADC + Temp sensor start */
  T_StartupTimeDelay = 1024;
  while (T_StartupTimeDelay--);
  
  /* Enable TempSensor and Vrefint channels: channel16 and Channel17 */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_56Cycles);
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 2, ADC_SampleTime_56Cycles);  
                               
  /* initialize result */
  res = 0;
  for(i=4; i>0; i--)
  {
  /* start ADC convertion by software */
    ADC_SoftwareStartConv(ADC1);

    /* wait until end-of-covertion */
    while( ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0 );
  /* read ADC convertion result */
    res += ADC_GetConversionValue(ADC1);
  }
	
  /* de-initialize ADC */
  ADC_TempSensorVrefintCmd(DISABLE);
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
  
  return (res>>2);
}
uint16_t vref_read(void)
{
  uint8_t i;
  uint16_t res, T_StartupTimeDelay;

  ADC_TempSensorVrefintCmd(ENABLE);
  /* Wait until ADC + Temp sensor start */
  T_StartupTimeDelay = 1024;
  while (T_StartupTimeDelay--);

  /* Enable TempSensor and Vrefint channels: channel16 and Channel17 */
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 2, ADC_SampleTime_56Cycles);

  /* initialize result */
  res = 0;
  for(i=4; i>0; i--)
  {
  /* start ADC convertion by software */
    ADC_SoftwareStartConv(ADC1);

    /* wait until end-of-covertion */
    while( ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0 );
  /* read ADC convertion result */
    res += ADC_GetConversionValue(ADC1);
  }

  /* de-initialize ADC */
  ADC_TempSensorVrefintCmd(DISABLE);
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);

  return (res>>2);
}
