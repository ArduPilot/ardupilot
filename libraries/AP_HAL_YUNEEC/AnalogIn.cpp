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
  Flymaple port by Mike McCauley
 */

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include "FlymapleWirish.h"
#include <AP_HAL.h>
#include "AnalogIn.h"
using namespace AP_HAL_FLYMAPLE_NS;

extern const AP_HAL::HAL& hal;

/* CHANNEL_READ_REPEAT: how many reads on a channel before using the value.
 * This seems to be determined empirically */
#define CHANNEL_READ_REPEAT 2

FLYMAPLEAnalogIn::FLYMAPLEAnalogIn() :
    _vcc(FLYMAPLEAnalogSource(ANALOG_INPUT_BOARD_VCC))
{}

void FLYMAPLEAnalogIn::init(void* machtnichts) {
    /* Register FLYMAPLEAnalogIn::_timer_event with the scheduler. */
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&FLYMAPLEAnalogIn::_timer_event));
    /* Register each private channel with FLYMAPLEAnalogIn. */
    _register_channel(&_vcc);
}

FLYMAPLEAnalogSource* FLYMAPLEAnalogIn::_create_channel(int16_t chnum) {
    FLYMAPLEAnalogSource *ch = new FLYMAPLEAnalogSource(chnum);
    _register_channel(ch);
    return ch;
}

void FLYMAPLEAnalogIn::_register_channel(FLYMAPLEAnalogSource* ch) {
    if (_num_channels >= FLYMAPLE_INPUT_MAX_CHANNELS) {
        for(;;) {
            hal.console->print_P(PSTR(
                "Error: AP_HAL_FLYMAPLE::FLYMAPLEAnalogIn out of channels\r\n"));
            hal.scheduler->delay(1000);
        }
    }
    _channels[_num_channels] = ch;
    /* Need to lock to increment _num_channels as it is used
     * by the interrupt to access _channels */
    noInterrupts();
    _num_channels++;
    interrupts();
    // Start conversions:
    adc_reg_map *regs = ADC1->regs;
    regs->CR2 |= ADC_CR2_SWSTART;
}

void FLYMAPLEAnalogIn::_timer_event(void) 
{
    adc_reg_map *regs = ADC1->regs;
    if (_channels[_active_channel]->_pin == ANALOG_INPUT_NONE) {
        _channels[_active_channel]->new_sample(0);
        goto next_channel;
    }

    if (!(regs->SR & ADC_SR_EOC))
    {
        /* ADC Conversion is still running - this should not happen, as we
         * are called at 1khz. */
        return;
    }

    if (_num_channels == 0) 
    {
        /* No channels are registered - nothing to be done. */
        return;
    }

    _channel_repeat_count++;
    if (_channel_repeat_count < CHANNEL_READ_REPEAT ||
        !_channels[_active_channel]->reading_settled()) 
    {
        /* Start a new conversion, throw away the current conversion */
        regs->CR2 |= ADC_CR2_SWSTART;
        return;
    }

    _channel_repeat_count = 0;
    {
        // Need this block because of declared variabled and goto
        uint16_t sample = (uint16)(regs->DR & ADC_DR_DATA);
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
    /* Start conversion */
    regs->CR2 |= ADC_CR2_SWSTART;
}


AP_HAL::AnalogSource* FLYMAPLEAnalogIn::channel(int16_t ch) 
{
    if (ch == ANALOG_INPUT_BOARD_VCC) {
            return &_vcc;
    } else {
        return _create_channel(ch);
    }
}

float FLYMAPLEAnalogIn::board_voltage(void)
{
    return _vcc.voltage_latest();
}

#endif
