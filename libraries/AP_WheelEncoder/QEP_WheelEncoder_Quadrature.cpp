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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_BoardConfig/AP_BoardConfig.h>
//#include <board_config.h>
#include "QEP_WheelEncoder_Quadrature.h"
#include <stdio.h>
#include "roboticscape/roboticscape.h"
extern const AP_HAL::HAL& hal;
QEP_WheelEncoder_Quadrature::IrqState QEP_WheelEncoder_Quadrature::irq_state[WHEELENCODER_MAX_INSTANCES];
int flag=0;
int32_t previous_read=0;
// constructor
QEP_WheelEncoder_Quadrature::QEP_WheelEncoder_Quadrature(AP_WheelEncoder &frontend, uint8_t instance, AP_WheelEncoder::WheelEncoder_State &state) :
	AP_WheelEncoder_Backend(frontend, instance, state)
{
}

void QEP_WheelEncoder_Quadrature::update(void)
{
    uint8_t instance = _state.instance;

    irq_state[instance].distance_count = rc_get_encoder_pos(instance+2);
    _state.distance_count =irq_state[instance].distance_count;
      irq_state[instance].last_reading_ms = AP_HAL::millis();
    _state.last_reading_ms = irq_state[instance].last_reading_ms;

}



#endif // CONFIG_HAL_BOARD
