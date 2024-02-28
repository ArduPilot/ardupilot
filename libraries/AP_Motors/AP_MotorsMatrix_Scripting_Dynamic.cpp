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

#include <AP_Scripting/AP_Scripting_config.h>

#if AP_SCRIPTING_ENABLED

// This allows motor roll, pitch, yaw and throttle factors to be changed in flight, allowing vehicle geometry to be changed

#include "AP_MotorsMatrix_Scripting_Dynamic.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define debug_print 0

// add a motor and give its testing order
bool AP_MotorsMatrix_Scripting_Dynamic::add_motor(uint8_t motor_num, uint8_t testing_order)
{
    if (initialised_ok()) {
        // no adding motors after init
        return false;
    }
    if (motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        _test_order[motor_num] = testing_order;
        motor_enabled[motor_num] = true;
        return true;
    }
    return false;
}

void AP_MotorsMatrix_Scripting_Dynamic::load_factors(const factor_table &new_table)
{
    WITH_SEMAPHORE(_sem);
    had_table = true;

    memcpy(_roll_factor,new_table.roll,sizeof(_roll_factor));
    memcpy(_pitch_factor,new_table.pitch,sizeof(_pitch_factor));
    memcpy(_yaw_factor,new_table.yaw,sizeof(_yaw_factor));
    memcpy(_throttle_factor,new_table.throttle,sizeof(_throttle_factor));

#if debug_print
    hal.console->printf("Got new factors:\n");
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            hal.console->printf("%i - Roll: %0.2f, Pitch %0.2f, Yaw: %0.2f, throttle %0.2f\n",i,_roll_factor[i],_pitch_factor[i],_yaw_factor[i],_throttle_factor[i]);
        }
    }
#endif

}

bool AP_MotorsMatrix_Scripting_Dynamic::init(uint8_t expected_num_motors)
{
    WITH_SEMAPHORE(_sem);

    // Check factors have been set
    if (!had_table) {
        return false;
    }

    // Make sure the correct number of motors have been added
    uint8_t num_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }

    set_initialised_ok(expected_num_motors == num_motors);

    if (!initialised_ok()) {
        _mav_type = MAV_TYPE_GENERIC;
        return false;
    }

    switch (num_motors) {
        case 3:
            _mav_type = MAV_TYPE_TRICOPTER;
            break;
        case 4:
            _mav_type = MAV_TYPE_QUADROTOR;
            break;
        case 6:
            _mav_type = MAV_TYPE_HEXAROTOR;
            break;
        case 8:
            _mav_type = MAV_TYPE_OCTOROTOR;
            break;
        case 10:
            _mav_type = MAV_TYPE_DECAROTOR;
            break;
        case 12:
            _mav_type = MAV_TYPE_DODECAROTOR;
            break;
        default:
            _mav_type = MAV_TYPE_GENERIC;
    }

    set_update_rate(_speed_hz);

    return true;
}

// output - sends commands to the motors, 
// Need to take the semaphore to enasure the motor factors are not changed during the mixer calculation
void AP_MotorsMatrix_Scripting_Dynamic::output_to_motors()
{
    WITH_SEMAPHORE(_sem);

    // call the base class ouput function
    AP_MotorsMatrix::output_to_motors();
}

// singleton instance
AP_MotorsMatrix_Scripting_Dynamic *AP_MotorsMatrix_Scripting_Dynamic::_singleton;

#endif // AP_SCRIPTING_ENABLED
