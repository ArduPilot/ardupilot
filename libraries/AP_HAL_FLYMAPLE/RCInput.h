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

#ifndef __AP_HAL_FLYMAPLE_RCINPUT_H__
#define __AP_HAL_FLYMAPLE_RCINPUT_H__

#include "AP_HAL_FLYMAPLE.h"

#define FLYMAPLE_RC_INPUT_NUM_CHANNELS 8

#define FLYMAPLE_RC_INPUT_MIN_CHANNELS 5     // for ppm sum we allow less than 8 channels to make up a valid packet

class AP_HAL_FLYMAPLE_NS::FLYMAPLERCInput : public AP_HAL::RCInput {
public:
    FLYMAPLERCInput();
    void init(void* machtnichts);
    bool new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

private:
    /* private callback for input capture ISR */
    static void _timer_capt_cb(void);

    /* private variables to communicate with input capture isr */
    static volatile uint16_t _pulse_capt[FLYMAPLE_RC_INPUT_NUM_CHANNELS];
    static volatile uint8_t  _valid_channels;
    static volatile uint32_t _last_input_interrupt_time;

    /* override state */
    uint16_t _override[FLYMAPLE_RC_INPUT_NUM_CHANNELS]; 
};

#endif // __AP_HAL_FLYMAPLE_RCINPUT_H__
