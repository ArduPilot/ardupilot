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
  this is a driver for multiple RCInput methods on one board
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
#include "RCInput_Multi.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

// constructor
RCInput_Multi::RCInput_Multi(uint8_t _num_inputs, ...) :
    num_inputs(_num_inputs)
{
    va_list ap;
    inputs = new RCInput*[num_inputs];
    if (inputs == nullptr) {
        AP_HAL::panic("failed to allocated RCInput array");
    }
    va_start(ap, _num_inputs);
    for (uint8_t i=0; i<num_inputs; i++) {
        inputs[i] = va_arg(ap, RCInput *);
        if (inputs[i] == nullptr) {
            AP_HAL::panic("Bad RCInput object");
        }
    }
    va_end(ap);
}

void RCInput_Multi::init()
{
    for (uint8_t i=0; i<num_inputs; i++) {
        inputs[i]->init();
    }
}

void RCInput_Multi::_timer_tick(void)
{
    for (uint8_t i=0; i<num_inputs; i++) {
        inputs[i]->_timer_tick();
        if (inputs[i]->new_input()) {
            inputs[i]->read(_pwm_values, inputs[i]->num_channels());
            _num_channels = inputs[i]->num_channels();
            rc_input_count++;
        }        
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE

