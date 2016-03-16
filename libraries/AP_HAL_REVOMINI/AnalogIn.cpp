/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  Copied from: Flymaple port by Mike McCauley
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI

#include <AP_HAL/AP_HAL.h>
#include "AnalogIn.h"
#include <adc.h>
#include <boards.h>
#include <gpio_hal.h>
#include "GPIO.h"

extern const AP_HAL::HAL& hal;

using namespace REVOMINI;

/* CHANNEL_READ_REPEAT: how many reads on a channel before using the value.
 * This seems to be determined empirically */
#define CHANNEL_READ_REPEAT 1

REVOMINIAnalogIn::REVOMINIAnalogIn():
	_vcc(REVOMINIAnalogSource(ANALOG_INPUT_BOARD_VCC))
{}

void REVOMINIAnalogIn::init() {

    /* Register REVOMINIAnalogIn::_timer_event with the scheduler. */
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&REVOMINIAnalogIn::_timer_event,void));
    /* Register each private channel with REVOMINIAnalogIn. */
    _register_channel(&_vcc);
}

REVOMINIAnalogSource* REVOMINIAnalogIn::_create_channel(uint8_t chnum) {

    REVOMINIAnalogSource *ch = new REVOMINIAnalogSource(chnum);
    _register_channel(ch);
    return ch;
}

void REVOMINIAnalogIn::_register_channel(REVOMINIAnalogSource* ch) {
    if (_num_channels >= REVOMINI_INPUT_MAX_CHANNELS) {
        for(;;) {
            hal.console->print("Error: AP_HAL_REVOMINI::REVOMINIAnalogIn out of channels\r\n");
            hal.scheduler->delay(1000);
        }
    }
    _channels[_num_channels] = ch;

    hal.console->printf("Register Channel:%u on pin:%u \n", _num_channels, ch->_pin );

    /* Need to lock to increment _num_channels as it is used
     * by the interrupt to access _channels */
    noInterrupts();
    _num_channels++;
    interrupts();
}


void REVOMINIAnalogIn::_timer_event(void)
{


    if (_num_channels == 0)
    {
        /* No channels are registered - nothing to be done. */
            return;
        }


    //adc_reg_map *regs = ADC1->regs;
    const adc_dev *dev = _channels[_active_channel]->_find_device();

    if (_channels[_active_channel]->_pin == ANALOG_INPUT_NONE) {
        _channels[_active_channel]->new_sample(0);
        goto next_channel;
    }

    if (!(dev->adcx->SR & ADC_SR_EOC))
	{
	    /* ADC Conversion is still running - this should not happen, as we
	     * are called at 1khz. */
	    return;
        }

    _channel_repeat_count++;
    if (_channel_repeat_count < CHANNEL_READ_REPEAT ||
        !_channels[_active_channel]->reading_settled()) 
    {
        /* Start a new conversion, throw away the current conversion */
        dev->adcx->CR2 |= (uint32_t)ADC_CR2_SWSTART;
        return;
    }

    _channel_repeat_count = 0;
    {
        // Need this block because of declared variabled and goto
        uint16_t sample = (uint16)(dev->adcx->DR & ADC_DR_DATA);
        /* Give the active channel a new sample */
        _channels[_active_channel]->new_sample( sample );
    }
next_channel:
    /* stop the previous channel, if a stop pin is defined */
    _channels[_active_channel]->stop_read();
    /* Move to the next channel */
    _active_channel = (_active_channel + 1) % _num_channels;
    /* Setup the next channel's conversion */
    _channels[_active_channel]->setup_read();

    dev = _channels[_active_channel]->_find_device();

    if(dev != NULL)
    /* Start conversion */
    dev->adcx->CR2 |= (uint32_t)ADC_CR2_SWSTART;
}


AP_HAL::AnalogSource* REVOMINIAnalogIn::channel(int16_t ch)
{
    if ((uint8_t)ch == ANALOG_INPUT_BOARD_VCC) {
            return &_vcc;
    } else {
        return _create_channel((uint8_t)ch);
    }
}

#endif
