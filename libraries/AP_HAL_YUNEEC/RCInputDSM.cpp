#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "RCInput.h"

#include <stm32f37x.h>
#include <stm32f37x_tim.h>
#include <stm32f37x_misc.h>
#include <utility/pinmap_typedef.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

#define DSM_BIND_TIME			200 //ms

#define PINPOS					(uint32_t)(DSM_UART_RX_BIT << 1)
#define DSM_GPIO_MODE_OUT		(uint32_t)(GPIO_Mode_OUT << PINPOS)
#define DSM_GPIO_MODE_AF		(uint32_t)(GPIO_Mode_AF << PINPOS)
#define DSM_GPIO_MODER 			(uint32_t)(GPIO_MODER_MODER0 << PINPOS)

/* private variables to communicate with input capture isr */
volatile uint8_t YUNEECRCInputDSM::_dsm_frame[DSM_FRAME_SIZE] = {0};
volatile uint32_t YUNEECRCInputDSM::_dsm_last_rx_time = 0;
volatile uint8_t YUNEECRCInputDSM::_dsm_partial_frame_count = 0;
uint16_t YUNEECRCInputDSM::_dsm_data_mask = 0;
uint8_t YUNEECRCInputDSM::_dsm_channel_shift = 0;
volatile uint16_t YUNEECRCInputDSM::_periods[DSM_RC_INPUT_CHANNELS] = {0};
volatile uint8_t  YUNEECRCInputDSM::_valid_channels = 0;
volatile bool YUNEECRCInputDSM::_new_input = false;
AP_HAL::UARTDriver* YUNEECRCInputDSM::_dsm_uart = NULL;
uint8_t YUNEECRCInputDSM::_pulses = DSM_CONFIG_INT_DSMx_11MS;
volatile uint32_t YUNEECRCInputDSM::_last_time = 0;

//extern "C"
//{
//	static voidFuncPtr timer12_callback = NULL;
//
//	void TIM12_IRQHandler(void) {
//		TIM12->SR &= ~TIM_IT_Update;
//
//		if(timer12_callback != NULL)
//			timer12_callback();
//	}
//}

void YUNEECRCInputDSM::init(void* machtnichts) {
	/* initiate uartC for dsm */
	_dsm_init(hal.uartC);

	/* we check the uart ring buffer every 2ms */
//	_timer12_config();
//	_attachInterrupt(_dsm_input);
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC( &YUNEECRCInputDSM::_dsm_input));

	/* now bind dsm receiver with remote */
	_dsm_bind();
}

bool YUNEECRCInputDSM::new_input() {
    return _new_input;
}

uint8_t YUNEECRCInputDSM::num_channels() {
    return _valid_channels;
}

/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

uint16_t YUNEECRCInputDSM::read(uint8_t ch) {
    if (ch >= YUNEEC_RC_INPUT_NUM_CHANNELS) return 0;

    uint16_t periods = _periods[ch];

    _new_input = false;
    uint16_t pulse = constrain_pulse(periods);
    /* Check for override */
    uint16_t over = _override[ch];
    return (over == 0) ? pulse : over;
}

uint8_t YUNEECRCInputDSM::read(uint16_t* periods, uint8_t len) {
    /* constrain len */
    if (len > YUNEEC_RC_INPUT_NUM_CHANNELS)
    	len = YUNEEC_RC_INPUT_NUM_CHANNELS;

    /* grab channels from isr's memory in critical section */
    for (uint8_t i = 0; i < len; i++) {
        periods[i] = _periods[i];
    }

    /* Outside of critical section, do the math (in place) to scale and
     * constrain the pulse. */
    for (uint8_t i = 0; i < len; i++) {
        periods[i] = constrain_pulse(periods[i]);
        /* check for override */
        if (_override[i] != 0) {
            periods[i] = _override[i];
        }
    }
    return _valid_channels;
}

bool YUNEECRCInputDSM::set_overrides(int16_t *overrides, uint8_t len) {
    bool res = false;
    for (int i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool YUNEECRCInputDSM::set_override(uint8_t channel, int16_t override) {
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

void YUNEECRCInputDSM::clear_overrides() {
    for (int i = 0; i < YUNEEC_RC_INPUT_NUM_CHANNELS; i++) {
        _override[i] = 0;
    }
}

/**
 * Initialize the DSM receive functionality
 *
 * Open the uartA for receiving DSM frames and configure it appropriately
 *
 * Notice that on YUNEEC platform, we can use uartA, which is USART2 of stm32f372
 * and its RX pin as DSM input port.
 */
void YUNEECRCInputDSM::_dsm_init(AP_HAL::UARTDriver* uartX) {

	if (uartX == NULL) {
		hal.scheduler->panic("No UART Rx port for DSM, Please specify one!");
		return ; // never reach
	}

	_dsm_uart = uartX;
	_dsm_uart->begin(115200);
}

/**
 * Check if the configuration is correct
 */
bool YUNEECRCInputDSM::_dsm_check_binded(void) {
	uint8_t dsm_config = 0;

	switch (_pulses) {
	case DSM_CONFIG_INT_DSM2_22MS:
		dsm_config = DSM2_1024_22MS;
		_dsm_channel_shift = 10;
		break;
	case DSM_CONFIG_INT_DSM2_11MS:
		dsm_config = DSM2_2048_11MS;
		_dsm_channel_shift = 11;
		break;
	case DSM_CONFIG_INT_DSMx_22MS:
		dsm_config = DSM2_2048_11MS;
		_dsm_channel_shift = 11;
		break;
	case DSM_CONFIG_INT_DSMx_11MS:
		dsm_config = DSMx_2048_11MS;
		_dsm_channel_shift = 11;
		break;
	default:
		hal.scheduler->panic(PSTR("YUNEECRCInputDSM: uncertain dsm configuration parameter!"));
		break;
	}

	/* DSM is configured incorrectly */
	if ((_dsm_frame[1] & ~dsm_config) != 0)
		return false;


	/* DSM is configured correctly */
	if (_dsm_channel_shift == 10)
		_dsm_data_mask = 0x3ff;
	else
		_dsm_data_mask = 0x7ff;

	return true;
}

/**
 * Set UART RX pin to active output mode
 */
void YUNEECRCInputDSM::_dsm_output_pulses(void) {
	_dsm_uart->end();

	/* Change gpio mode */
	hal.gpio->pinMode(PA3, HAL_GPIO_OUTPUT);
	/* Pulse RX pin a number of times */
	for (uint8_t i = 0; i < _pulses; i++) {
		hal.scheduler->delay_microseconds(25);
		hal.gpio->write(PA3, 0);
		hal.scheduler->delay_microseconds(25);
		hal.gpio->write(PA3, 1);
	}

	_dsm_uart->begin(115200);
}

/**
 * Handle DSM satellite receiver bind mode handler
 */
void YUNEECRCInputDSM::_dsm_bind(void) {
	hal.console->printf_P(PSTR("DSM is in BINDING mode...\n"));

	/* Wait for tx task complete */
	while (_dsm_uart->tx_pending())
		;

	uint32_t start = hal.scheduler->millis();
	/* Check if dsm receiver is already binded */
	while (_new_input == false) {
		uint32_t now = hal.scheduler->millis();
		if (now - start > DSM_BIND_TIME)
			break;
		hal.scheduler->delay(1);
	}

	if ((_new_input == true) && (_dsm_check_binded() == true)) {
		hal.console->printf_P(PSTR("DSM BINDED already\n"));
		return;
	}

	/* It is not binded, output pulses to configure dsm receiver */
	_dsm_output_pulses();

	/* Confirm dsm receiver is configured correctly */
	while (_new_input == false)
		;

	if (_dsm_check_binded() == true) {
		hal.console->printf_P(PSTR("DSM BINDING successfully\n"));
		return;
	}
	else
		hal.scheduler->panic(PSTR("YUNEECRCInputDSM: DSM receiver is configured incorrectly"));
}

/**
 * Called periodically to check for input data from the DSM UART
 */
void YUNEECRCInputDSM::_dsm_input(void) {
	int16_t	rx_num;
	uint32_t now;

	now = hal.scheduler->millis();

	/* We check input dsm data every 3ms to reduce scheduler timer load */
	if (now - _last_time < 3)
		return;

	_last_time = now;

	if ((now - _dsm_last_rx_time) > 5) {
		if (_dsm_partial_frame_count > 0)
			_dsm_partial_frame_count = 0;
	}

	/*
	 * Fetch bytes, but no more than we would need to complete
	 * the current dsm frame.
	 */
	for (rx_num = 0; rx_num < DSM_FRAME_SIZE - _dsm_partial_frame_count; rx_num++) {
		int16_t ret = _dsm_uart->read();
		if (ret < 0)
			break;
		else
			_dsm_frame[rx_num + _dsm_partial_frame_count] = ret;
	}

	/* if the read failed for any reason, just give up here */
	if (rx_num < 1)
		return;

	_dsm_last_rx_time = now;

	/*
	 * Add bytes to the current dsm frame
	 */
	_dsm_partial_frame_count += rx_num;

	/*
	 * If we don't have a full dsm frame, return
	 */
	if (_dsm_partial_frame_count < DSM_FRAME_SIZE)
		return;

	/*
	 * Great, it looks like we might have a dsm frame.  Go ahead and
	 * decode it.
	 */
	_dsm_partial_frame_count = 0;

	/*
	 * The encoding of the first two bytes contains dsm configuration
	 * info.
	 *
	 * Each channel is a 16-bit unsigned value containing either a 10-
	 * or 11-bit channel value and a 4-bit channel number, shifted
	 * either 10 or 11 bits. The MSB may also be set to indicate the
	 * second dsm_frame in variants of the protocol where more than
	 * seven channels are being transmitted.
	 */
	for (uint8_t i = 0; i < DSM_FRAME_CHANNELS; i++) {
		uint16_t raw = (_dsm_frame[2 + 2*i] << 8) | _dsm_frame[3 + 2*i];
		uint8_t channel = (raw >> _dsm_channel_shift) & 0xf;
		uint16_t value = raw & _dsm_data_mask;

		/* ignore channels out of range */
		if (channel >= DSM_RC_INPUT_CHANNELS)
			continue;

		/* update the decoded channel count */
		if (channel >= _valid_channels)
			_valid_channels = channel + 1;

		/* convert 0-1024 / 0-2048 values to 900-2100 ppm encoding. */
		if (_dsm_channel_shift == 10)
			value = value * 1200 / 1024 + 900;
		else // if(_dsm_channel_shift == 11)
			value = value * 1200 / 2048 + 900;

		/*
		 * Store the decoded channel into the R/C input buffer, taking into
		 * account the different ideas about channel assignment that we have.
		 *
		 * Specifically, the first four channels in rc_channel_data are roll, pitch, throttle, yaw,
		 * but the first four channels from the DSM receiver are throttle, roll, pitch, yaw.
		 */
		switch (channel) {
		case 0:
			channel = 2;
			break;

		case 1:
			channel = 0;
			break;

		case 2:
			channel = 1;

		default:
			break;
		}

		_periods[channel] = value;
	}

	/*
	 * Spektrum likes to send junk in higher channel numbers to fill
	 * their packets. We don't know about a 13 channel model in their TX
	 * lines, so if we get a channel count of 13, we'll return 12 (the last
	 * data index that is stable).
	 */
	if (_valid_channels == 13)
		_valid_channels = 12;

	/*
	 * XXX Note that we may be in failsafe here; we need to work out how to detect that.
	 */

	_new_input = true;

}

//void YUNEECRCInputDSM::_timer12_config() {
//	NVIC_InitTypeDef NVIC_InitStructure;
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//
//	/* TIM12 clock enable */
//    RCC->APB1ENR |= RCC_APB1Periph_TIM12;
//
//	/* Configure two bits for preemption priority */
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	/* Enable the TIM3 global Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM12_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	/* Time base configuration */
//	TIM_TimeBaseStructure.TIM_Period = 2000 - 1; // 2ms per interrupt
//	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1; // 1 MHz
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);
//
//	/* Enable the CC4 Interrupt Request */
//    TIM12->DIER |= TIM_IT_Update;
//
//    /* Enable the TIM Counter */
//    TIM12->CR1 |= TIM_CR1_CEN;
//}
//
//void YUNEECRCInputDSM::_attachInterrupt(voidFuncPtr callback) {
//	if (callback != NULL)
//		timer12_callback = callback;
//}

#endif
