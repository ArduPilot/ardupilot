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
  Flymaple port by Mike McCauley
  Monitor a PPM-SUM input pin, and decode the channels based on pulse widths
  Uses a timer to capture the time between negative transitions of the PPM-SUM pin
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

// Flymaple RCInput
// PPM input from a single pin

#include "RCInput.h"
#include "FlymapleWirish.h"

using namespace AP_HAL_FLYMAPLE_NS;

extern const AP_HAL::HAL& hal;


/* private variables to communicate with input capture isr */
volatile uint16_t FLYMAPLERCInput::_pulse_capt[FLYMAPLE_RC_INPUT_NUM_CHANNELS] = {0};  
volatile uint8_t  FLYMAPLERCInput::_valid_channels = 0;
volatile uint32_t FLYMAPLERCInput::_last_input_interrupt_time = 0; // Last time the input interrupt ran

// Pin 6 is connected to timer 1 channel 1
#define FLYMAPLE_RC_INPUT_PIN 6

// This is the rollover count of the timer
// each count is 0.5us, so 600000 = 30ms
// We cant reliably measure intervals that exceed this time.
#define FLYMAPLE_TIMER_RELOAD 60000

FLYMAPLERCInput::FLYMAPLERCInput()
{}

// This interrupt triggers on a negative transiution of the PPM-SIM pin
void FLYMAPLERCInput::_timer_capt_cb(void)
{
    _last_input_interrupt_time = AP_HAL::millis();

    static uint16 previous_count;
    static uint8  channel_ctr;

    // Read the CCR register, where the time count since the last input pin transition will be
    timer_dev *tdev = PIN_MAP[FLYMAPLE_RC_INPUT_PIN].timer_device;
    uint8 timer_channel = PIN_MAP[FLYMAPLE_RC_INPUT_PIN].timer_channel;
    uint16 current_count = timer_get_compare(tdev, timer_channel);
    uint32 sr = (tdev->regs).gen->SR;
    uint32 overcapture_mask = (1 << (TIMER_SR_CC1OF_BIT + timer_channel - 1));

    if (sr & overcapture_mask)
    {
	// Hmmm, lost an interrupt somewhere? Ignore this sample
	(tdev->regs).gen->SR &= ~overcapture_mask; // Clear overcapture flag
	return;
    }

    uint16_t pulse_width;
    if (current_count < previous_count) {
        pulse_width = current_count + FLYMAPLE_TIMER_RELOAD - previous_count;
    } else {
        pulse_width = current_count - previous_count;
    }

    // Pulse sequence repetition rate is about 22ms.
    // Longest servo pulse is about 1.8ms
    // Shortest servo pulse is about 0.5ms
    // Shortest possible PPM sync pulse with 10 channels is about 4ms = 22 - (10 channels * 1.8)
    if (pulse_width > 8000) { // 4ms
        // sync pulse detected.  Pass through values if at least a minimum number of channels received
        if( channel_ctr >= FLYMAPLE_RC_INPUT_MIN_CHANNELS ) {
            _valid_channels = channel_ctr;
	    // Clear any remaining channels, in case they were corrupted during a connect or something
	    while (channel_ctr < FLYMAPLE_RC_INPUT_NUM_CHANNELS)
		_pulse_capt[channel_ctr++] = 0;
        }
        channel_ctr = 0;
    } else {

//	if (channel_ctr == 0)
//	    hal.uartA->printf("ch 0 %d\n", pulse_width);

        if (channel_ctr < FLYMAPLE_RC_INPUT_NUM_CHANNELS) {
            _pulse_capt[channel_ctr] = pulse_width;
            channel_ctr++;
            if (channel_ctr == FLYMAPLE_RC_INPUT_NUM_CHANNELS) {
                _valid_channels = FLYMAPLE_RC_INPUT_NUM_CHANNELS;
            }
        }
    }
    previous_count = current_count;
}

void FLYMAPLERCInput::init()
{
    /* initialize overrides */
    clear_overrides();

    // Configure pin 6 input to timer 1 CH1 bRin Input Capture mode
    pinMode(FLYMAPLE_RC_INPUT_PIN, INPUT_PULLDOWN);

    timer_dev *tdev = PIN_MAP[FLYMAPLE_RC_INPUT_PIN].timer_device;
    uint8 timer_channel = PIN_MAP[FLYMAPLE_RC_INPUT_PIN].timer_channel;
    timer_pause(tdev); // disabled
    timer_set_prescaler(tdev, (CYCLES_PER_MICROSECOND/2) - 1); // 2MHz = 0.5us timer ticks
    timer_set_reload(tdev, FLYMAPLE_TIMER_RELOAD-1);
    // Without a filter, can get triggering on the wrong edge and other problems.
    (tdev->regs).gen->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1 | (3 << 4); // no prescaler, input from T1, filter internal clock, N=8
    (tdev->regs).gen->CCER  = TIMER_CCER_CC1P | TIMER_CCER_CC1E; // falling edge, enable capture
    timer_attach_interrupt(tdev, timer_channel, _timer_capt_cb);
    timer_generate_update(tdev);
    timer_resume(tdev); // reenabled
}

bool FLYMAPLERCInput::new_input() {
    if ((AP_HAL::millis() - _last_input_interrupt_time) > 50)
	_valid_channels = 0; // Lost RC Input?
    return _valid_channels != 0;
}

uint8_t FLYMAPLERCInput::num_channels() {
    return _valid_channels;
}

/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

uint16_t FLYMAPLERCInput::read(uint8_t ch) {
    timer_dev *tdev = PIN_MAP[FLYMAPLE_RC_INPUT_PIN].timer_device;
    uint8 timer_channel = PIN_MAP[FLYMAPLE_RC_INPUT_PIN].timer_channel;

    /* constrain ch */
    if (ch >= FLYMAPLE_RC_INPUT_NUM_CHANNELS) 
	return 0;
    /* grab channel from isr's memory in critical section*/
    timer_disable_irq(tdev, timer_channel);
    uint16_t capt = _pulse_capt[ch];
    timer_enable_irq(tdev, timer_channel);
    /* scale _pulse_capt from 0.5us units to 1us units. */
    uint16_t pulse = constrain_pulse(capt >> 1);
    /* Check for override */
    uint16_t over = _override[ch];
    return (over == 0) ? pulse : over;
}

uint8_t FLYMAPLERCInput::read(uint16_t* periods, uint8_t len) {
    timer_dev *tdev = PIN_MAP[FLYMAPLE_RC_INPUT_PIN].timer_device;
    uint8 timer_channel = PIN_MAP[FLYMAPLE_RC_INPUT_PIN].timer_channel;

    /* constrain len */
    if (len > FLYMAPLE_RC_INPUT_NUM_CHANNELS) 
	len = FLYMAPLE_RC_INPUT_NUM_CHANNELS;
    /* grab channels from isr's memory in critical section */
    timer_disable_irq(tdev, timer_channel);
    for (uint8_t i = 0; i < len; i++) {
        periods[i] = _pulse_capt[i];
    }
    timer_enable_irq(tdev, timer_channel);
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

bool FLYMAPLERCInput::set_overrides(int16_t *overrides, uint8_t len) {
    bool res = false;
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool FLYMAPLERCInput::set_override(uint8_t channel, int16_t override) {
    if (override < 0) return false; /* -1: no change. */
    if (channel < FLYMAPLE_RC_INPUT_NUM_CHANNELS) {
        _override[channel] = override;
        if (override != 0) {
            _valid_channels = 1;
            return true;
        }
    }
    return false;
}

void FLYMAPLERCInput::clear_overrides()
{
    for (uint8_t i = 0; i < FLYMAPLE_RC_INPUT_NUM_CHANNELS; i++) {
        _override[i] = 0;
    }
}

#endif
