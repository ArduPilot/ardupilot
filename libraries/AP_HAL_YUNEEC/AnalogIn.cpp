#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "AnalogIn.h"
#include <stm32f37x.h>
#include <stm32f37x_adc.h>
#include <stm32f37x_rcc.h>
#include <stm32f37x_dma.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

#define ADC1_DR_Address				0x4001244C

#define YUNEEC_VOLTAGE_SCALING		(float)3.3f/4095

extern "C"
{
	static voidFuncPtr dma1_callback = NULL;
	void DMA1_Channel1_IRQHandler(void) {
	    	DMA1->IFCR = DMA1_FLAG_TC1;

		    if(dma1_callback != NULL)
		    	dma1_callback();
	}
}

/*
  scaling table between ADC count and actual input voltage, to account
  for voltage dividers on the board.
 */
//static const struct {
//    uint8_t pin;
//    float scaling;
//} pin_scaling[] = {
//    { PC5,   9 * YUNEEC_VOLTAGE_SCALING }, 	// Battary voltage, 9:1 scaling
//    { PB0,  40 * YUNEEC_VOLTAGE_SCALING }		// Battary current, 20:1 scaling
//};

//const uint8_t YUNEECAnalogSource::_num_pin_scaling = sizeof(pin_scaling) / sizeof(pin_scaling[0]);
volatile uint16_t YUNEECAnalogSource::_ADCConvData_Tab[YUNEEC_INPUT_MAX_CHANNELS] = {0};
uint8_t YUNEECAnalogSource::_ADCChannels_Tab[YUNEEC_INPUT_MAX_CHANNELS] = {0};
uint8_t YUNEECAnalogSource::_num_adc_channels = 0;

YUNEECAnalogSource::YUNEECAnalogSource(uint8_t pin) :
    _sum_count(0),
    _sum(0),
    _last_average(0),
    _latest(0),
    _pin(ANALOG_INPUT_NONE),
    _stop_pin(ANALOG_INPUT_NONE),
    _stop_pin_high(false),
    _settle_time_ms(0),
    _read_start_time_ms(0),
//    _pin_scaling_id(-1),
    _channel_rank(-1)
{
    set_pin(pin);
}

float YUNEECAnalogSource::read_average() {
    uint16_t sum;
    uint8_t sum_count;

    if (_sum_count == 0) {
        // avoid blocking waiting for new samples
        return _last_average;
    }

    /* Read and clear in a critical section */
    __disable_irq();
    sum = _sum;
    sum_count = _sum_count;
    _sum = 0;
    _sum_count = 0;
    __enable_irq();

    float avg = sum / (float) sum_count;
    _last_average = avg;

    return avg;
}

float YUNEECAnalogSource::read_latest() {
    return _latest;
}

/*
  return scaling from ADC count to Volts
 */
//float YUNEECAnalogSource::_pin_scaler(void) {
//	if (_pin_scaling_id != -1)
//		return pin_scaling[_pin_scaling_id].scaling;
//	else {
//        hal.console->println(PSTR("YUNEEC::YUNEECAnalogIn can't find pin scaling message, you need to check the pin_scaling[] definiton\r\n"));
//		return 0;
//	}
//
//}

/*
  return voltage in Volts
 */
float YUNEECAnalogSource::voltage_average(void) {
    return YUNEEC_VOLTAGE_SCALING * read_average();
}

float YUNEECAnalogSource::voltage_latest(void) {
    return YUNEEC_VOLTAGE_SCALING * read_latest();
}

/*
  return voltage from 0.0 to 3.3V, assuming a ratiometric sensor. This
  means the result is really a pseudo-voltage, that assumes the supply
  voltage is exactly 3.3V.
 */
float YUNEECAnalogSource::voltage_average_ratiometric(void) {
    return YUNEEC_VOLTAGE_SCALING * read_average();
}

void YUNEECAnalogSource::set_pin(uint8_t pin) {
	// This pin has been set before
    if (pin == _pin)
    	return;

	// Ensure the pin is marked as an INPUT pin
	if (pin == ANALOG_INPUT_NONE)
		_unregister_adc_channel();
	else if(pin != ANALOG_INPUT_BOARD_VCC) {
		_register_adc_channel(pin);
		// set pin mode to analog input
		hal.gpio->pinMode(pin, HAL_GPIO_INPUT);
	}

	// Clear data
	_pin = pin;
	__disable_irq();
	_sum = 0;
	_sum_count = 0;
	_last_average = 0;
	_latest = 0;
	__enable_irq();

	// Update ADC1 configuration
	_update_adc1_config();
}

uint16_t YUNEECAnalogSource::_get_conv_data(void) {
	if(_channel_rank != -1)
		return _ADCConvData_Tab[_channel_rank];
	else
		return 0;
}

// Register a adc channel for the given pin
void YUNEECAnalogSource::_register_adc_channel(uint8_t pin) {
	// Find the index in _pin_sacling[]
//    for(uint8_t i = 0; i < _num_pin_scaling; i++) {
//    	if(pin_scaling[i].pin == pin)
//    	    _pin_scaling_id = i;
//    }

    // Add this pin into _ADCChannels_Tab[] for ADC1 configuration
    _ADCChannels_Tab[_num_adc_channels] = pin;
    // The rank for conversion and also for find data in _ADCConvData_Tab[]
    _channel_rank = _num_adc_channels;
    // We have one more adc channel set
    _num_adc_channels++;

}

// Unregister a adc channel for the given pin
void YUNEECAnalogSource::_unregister_adc_channel() {
//	_pin_scaling_id = -1;

	for(uint8_t i = _channel_rank; i < _num_adc_channels - 1; i++)
	    _ADCChannels_Tab[i] = _ADCChannels_Tab[i + 1];

    _num_adc_channels--;
    _channel_rank = -1;
}

void YUNEECAnalogSource::set_stop_pin(uint8_t pin) {
    _stop_pin = pin;
}

void YUNEECAnalogSource::set_settle_time(uint16_t settle_time_ms) {
    _settle_time_ms = settle_time_ms;
}

/* new_sample is called from an interrupt. It always has access to
 *  _sum and _sum_count. Lock out the interrupts briefly with
 * __disable_irq()/__enable_irq() to read these variables from outside an interrupt.
 */
void YUNEECAnalogSource::new_sample(uint16_t sample) {
    _sum += sample;
    _latest = sample;
    if (_sum_count >= 15) { // YUNEEC has a 12 bit ADC, so can only sum 16 in a uint16_t
        _sum >>= 1;
        _sum_count = 8;
    } else {
        _sum_count++;
    }
}

void YUNEECAnalogSource::_init_adc1(void) {
	ADC_InitTypeDef			ADC_InitStructure;
	DMA_InitTypeDef 		DMA_InitStructure;
	NVIC_InitTypeDef		NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  		TIM_OCInitStructure;

	// TIM4 clock enable
    RCC->APB1ENR |= RCC_APB1Periph_TIM4;
	// DMA1 clock enable
    RCC->AHBENR |= RCC_AHBPeriph_DMA1;
	// ADCCLK = PCLK2/4
    /* Clear ADCPRE[1:0] bits */
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;
    /* Set ADCPRE[1:0] bits according to RCC_PCLK2 value */
    RCC->CFGR |= RCC_PCLK2_Div4;
	// ADC1 Periph clock enable
    RCC->APB2ENR |= RCC_APB2Periph_ADC1;

	// Time base configuration: 25ms per interrupt
	TIM_TimeBaseStructure.TIM_Period 		= 25000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler 	= SystemCoreClock / 1000000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState 	= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState 	= TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse 			= 0x000000FF;
	TIM_OCInitStructure.TIM_OCPolarity 		= TIM_OCPolarity_Low;
	// PWM Mode configuration: TIM4 Channel4
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	// DMA1 Channel1 Config
	// Reset DMA1
	/* Disable the selected DMA1_Channel1 */
	DMA1_Channel1->CCR &= (uint16_t)(~DMA_CCR_EN);
	/* Reset DMAy Channelx control register */
	DMA1_Channel1->CCR  = 0;
	/* Reset DMAy Channelx remaining bytes register */
	DMA1_Channel1->CNDTR = 0;
	/* Reset DMAy Channelx peripheral address register */
	DMA1_Channel1->CPAR  = 0;
	/* Reset DMAy Channelx memory address register */
	DMA1_Channel1->CMAR = 0;
	/* Reset interrupt pending bits for DMA1 Channel1 */
	DMA1->IFCR |= ((uint32_t)(DMA_ISR_GIF1 | DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1));

	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr 		= (uint32_t)_ADCConvData_Tab;
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize 			= 0;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M 					= DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	// DMA1 Channel1 enable
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	// DMA1 Channel1 interrupt enable
    DMA1_Channel1->CCR |= DMA_IT_TC;

	// Configure two bits for preemption priority
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	// Configure and enable ADC1 interrupt
	NVIC_InitStructure.NVIC_IRQChannel 						= DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// ADC1 DeInit
	/* Enable ADC1 reset state */
    RCC->APB2RSTR |= RCC_APB2Periph_ADC1;
	/* Release ADC1 from reset state */
    RCC->APB2RSTR &= ~RCC_APB2Periph_ADC1;
	// Enable ADC_DMA
    ADC1->CR2 |= ADC_CR2_DMA;
	// Initialize ADC structure
	ADC_InitStructure.ADC_ScanConvMode 			= ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode 	= DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv 		= ADC_ExternalTrigConv_T4_CC4;
	ADC_InitStructure.ADC_DataAlign 			= ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel 			= 0;
	ADC_Init(ADC1, &ADC_InitStructure);

	// Enable ADC1
	ADC1->CR2 |= ADC_CR2_ADON;

	// ADC1 reset calibration register
	ADC1->CR2 |= ADC_CR2_RSTCAL;
	while(ADC1->CR2 & ADC_CR2_RSTCAL);
	// ADC1 calibration start
	ADC1->CR2 |= ADC_CR2_CAL;
	while(ADC1->CR2 & ADC_CR2_RSTCAL);
	// Enable Conversion on Trigger event
	ADC1->CR2 |= ADC_CR2_EXTTRIG;

	// Enable ADC1
	ADC1->CR2 |= ADC_CR2_ADON;
	// Enable Timer to start trigger event
	TIM4->CR1 |= TIM_CR1_CEN;
}


void YUNEECAnalogSource::_update_adc1_config() {
    TIM4->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
    DMA1_Channel1->CCR &= (uint16_t)(~DMA_CCR_EN);
    ADC1->CR2 &= (uint32_t)(~ADC_CR2_ADON);

	// Set DMA buffer size
	DMA1_Channel1->CNDTR = (uint32_t)(_num_adc_channels);

	// Set ADC1 number of regular channel
	uint32_t tmpreg1 = ADC1->SQR1;
	uint8_t tmpreg2 = _num_adc_channels - (uint8_t)1;
	tmpreg1 &= (uint32_t) (~ADC_SQR1_L);
	tmpreg1 |= (uint32_t)tmpreg2 << 20;
	ADC1->SQR1 = tmpreg1;

	for(uint8_t i = 0; i < _num_adc_channels; i++) {
		uint8_t adc_channel = get_adc_channel(_ADCChannels_Tab[i]);
		ADC_RegularChannelConfig(ADC1, adc_channel, (i + 1), ADC_SampleTime_55Cycles5);
	}

    ADC1->CR2 |= ADC_CR2_ADON;
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    TIM4->CR1 |= TIM_CR1_CEN;
}

YUNEECAnalogSource* YUNEECAnalogIn::_channels[YUNEEC_INPUT_MAX_CHANNELS] = {NULL};
uint8_t YUNEECAnalogIn::_num_channels = 0;
uint8_t YUNEECAnalogIn::_current_stop_pin_i = 0;

void YUNEECAnalogIn::init(void* machtnichts) {
	YUNEECAnalogSource::_init_adc1();
	_attach_interrupt(_dma_event);
}

AP_HAL::AnalogSource* YUNEECAnalogIn::channel(int16_t pin)
{
	if (pin == ANALOG_INPUT_NONE) {
	    hal.console->println_P(PSTR("YUNEECAnalogIn: Register a analog channel without a actual adc channel\r\n"));
	}

    if (_num_channels >= YUNEEC_INPUT_MAX_CHANNELS) {
        hal.console->println(PSTR("Error: YUNEEC::YUNEECAnalogIn out of channels\r\n"));
        return NULL;
    }

    YUNEECAnalogSource *ch = new YUNEECAnalogSource(pin);
    _channels[_num_channels++] = ch;
    return ch;
}


float YUNEECAnalogIn::board_voltage(void) {
    return 3.3f;
}

/*
  move to the next stop pin
 */
void YUNEECAnalogIn::_next_stop_pin(void)
{
	// Set current stop pin low to stop device
	if(_channels[_current_stop_pin_i]->_stop_pin != ANALOG_INPUT_NONE) {
		hal.gpio->pinMode(_channels[_current_stop_pin_i]->_stop_pin, HAL_GPIO_OUTPUT);
		hal.gpio->write(_channels[_current_stop_pin_i]->_stop_pin, 0);
		_channels[_current_stop_pin_i]->_stop_pin_high = false;
	}

	for(uint8_t i = 1; i <= _num_channels; i++) {
		uint8_t idx = (_current_stop_pin_i + i) % _num_channels;
		if(_channels[idx]->_stop_pin != ANALOG_INPUT_NONE) {
			_current_stop_pin_i = idx;
			hal.gpio->pinMode(_channels[idx]->_stop_pin, HAL_GPIO_OUTPUT);
			hal.gpio->write(_channels[idx]->_stop_pin, 1);
			_channels[_current_stop_pin_i]->_stop_pin_high = true;
			_channels[_current_stop_pin_i]->_read_start_time_ms = hal.scheduler->millis();
			return;
		}
	}
}


void YUNEECAnalogIn::_dma_event() {
    for (uint8_t i = 0; i < _num_channels; i++) {
    	if(_channels[i]->_pin == ANALOG_INPUT_NONE)
    		continue;
    	else {
			if (_channels[i]->_stop_pin == ANALOG_INPUT_NONE)
				_channels[i]->new_sample(_channels[i]->_get_conv_data());
			else if ( (_channels[i]->_stop_pin_high == true) && ((hal.scheduler->millis() - _channels[i]->_read_start_time_ms) > _channels[i]->_settle_time_ms))
				_channels[i]->new_sample(_channels[i]->_get_conv_data());
    	}
    }

	_next_stop_pin();
}

void YUNEECAnalogIn::_attach_interrupt(voidFuncPtr callback) {
	if(callback != NULL)
		dma1_callback = callback;
}

#endif
