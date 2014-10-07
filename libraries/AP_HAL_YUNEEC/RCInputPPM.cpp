#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "RCInput.h"

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

/* private variables to communicate with input capture isr */
volatile uint16_t YUNEECRCInputPPM::_pulse_capt[YUNEEC_RC_INPUT_NUM_CHANNELS] = {0};
volatile uint8_t  YUNEECRCInputPPM::_valid_channels = 0;
volatile bool YUNEECRCInputPPM::_new_input = false;

extern "C"
{
	static voidFuncPtr timer3_callback = NULL;

	void TIM3_IRQHandler(void){
		if ((TIM3->SR & TIM_IT_CC4) == SET)
		{
			/* Clear TIM2 Capture compare interrupt pending bit */
			TIM3->SR &= (uint16_t)~TIM_IT_CC4;

		    if ((TIM3->SR & TIM_FLAG_CC4OF) == SET)
		    {
		    	TIM3->SR &= ~TIM_FLAG_CC4OF;
				return;
		    }

			if(timer3_callback != NULL)
				timer3_callback();
		}
	}
}

void YUNEECRCInputPPM::init(void* machtnichts) {
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* TIM3 clock enable */
    RCC->APB1ENR |= RCC_APB1Periph_TIM3;

	/* GPIOB clock enable */
    RCC->AHBENR |= RCC_AHBPeriph_GPIOB;

	/* TIM3 channel 4 pin (PB1) configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);

	/* Configure two bits for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM3 configuration: Input Capture mode ---------------------
	 The external signal is connected to TIM3 CH4 pin (PB1)
	 The Rising edge is used as active edge,
	 The TIM3 CCR4 is used to compute the pulse width of each channel
	------------------------------------------------------------ */
	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // Max 65536
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 2000000 - 1; // 2 MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x3;

	TIM_ICInit(TIM3, &TIM_ICInitStructure);

    /* Enable the TIM Counter */
    TIM3->CR1 |= TIM_CR1_CEN;

	_attachInterrupt(_timer_capt_cb);

	/* Enable the CC4 Interrupt Request */
	TIM3->DIER |= TIM_IT_CC4;
}

bool YUNEECRCInputPPM::new_input() {
    return _new_input;
}

uint8_t YUNEECRCInputPPM::num_channels() {
    return _valid_channels;
}

/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

uint16_t YUNEECRCInputPPM::read(uint8_t ch) {
    if (ch >= YUNEEC_RC_INPUT_NUM_CHANNELS) return 0;

    NVIC_DisableIRQ(TIM3_IRQn);
    uint16_t capt = _pulse_capt[ch];
    NVIC_EnableIRQ(TIM3_IRQn);

    _new_input = false;
    /* scale _pulse_capt from 0.5us units to 1us units. */
    uint16_t pulse = constrain_pulse(capt >> 1);
    /* Check for override */
    uint16_t over = _override[ch];
    return (over == 0) ? pulse : over;
}

uint8_t YUNEECRCInputPPM::read(uint16_t* periods, uint8_t len) {
    /* constrain len */
    if (len > YUNEEC_RC_INPUT_NUM_CHANNELS)
    	len = YUNEEC_RC_INPUT_NUM_CHANNELS;
    /* grab channels from isr's memory in critical section */
    NVIC_DisableIRQ(TIM3_IRQn);
    for (uint8_t i = 0; i < len; i++) {
        periods[i] = _pulse_capt[i];
    }
    NVIC_EnableIRQ(TIM3_IRQn);

    /* Outside of critical section, do the math (in place) to scale and
     * constrain the pulse. */
    for (uint8_t i = 0; i < len; i++) {
        /* scale _pulse_capt from 0.5us units to 1us units. */
        periods[i] = constrain_pulse(periods[i] >> 1);
        /* check for override */
        if (_override[i] != 0) {
            periods[i] = _override[i];
        }
    }
    return _valid_channels;
}

bool YUNEECRCInputPPM::set_overrides(int16_t *overrides, uint8_t len) {
    bool res = false;
    for (int i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool YUNEECRCInputPPM::set_override(uint8_t channel, int16_t override) {
    if (override < 0) return false; /* -1: no change. */
    if (channel < YUNEEC_RC_INPUT_NUM_CHANNELS) {
        _override[channel] = override;
        if (override != 0) {
            _new_input = true;
            return true;
        }
    }
    return false;
}

void YUNEECRCInputPPM::clear_overrides() {
    for (int i = 0; i < YUNEEC_RC_INPUT_NUM_CHANNELS; i++) {
        _override[i] = 0;
    }
}

void YUNEECRCInputPPM::_timer_capt_cb(void) {
	uint16_t current_count = TIM3->CCR4;
	static uint16_t previous_count;
	static uint8_t  channel_ctr;
    uint16_t pulse_width;

	if (current_count < previous_count) {
		pulse_width = current_count + 0xFFFF - previous_count;
	} else {
		pulse_width = current_count - previous_count;
	}

	if (pulse_width > 8000) {
		// sync pulse detected.  Pass through values if at least a minimum number of channels received
		if( channel_ctr >= YUNEEC_RC_INPUT_MIN_CHANNELS ) {
			_valid_channels = channel_ctr;
			_new_input = true;
		}
		channel_ctr = 0;
	} else {
		if (channel_ctr < YUNEEC_RC_INPUT_NUM_CHANNELS) {
			_pulse_capt[channel_ctr] = pulse_width;
			channel_ctr++;
			if (channel_ctr == YUNEEC_RC_INPUT_NUM_CHANNELS) {
				_valid_channels = YUNEEC_RC_INPUT_NUM_CHANNELS;
                _new_input = true;
			}
		}
	}
	previous_count = current_count;
}

void YUNEECRCInputPPM::_attachInterrupt(voidFuncPtr callback) {
	if (callback != NULL)
		timer3_callback = callback;
}

#endif
