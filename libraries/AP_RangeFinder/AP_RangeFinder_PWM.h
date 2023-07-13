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
#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_PWM_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

class AP_RangeFinder_PWM : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_PWM(RangeFinder::RangeFinder_State &_state,
                       AP_RangeFinder_Params &_params,
                       float &_estimated_terrain_height);

    // destructor
    ~AP_RangeFinder_PWM(void) {};

    // static detection function
    static bool detect();

    // update state
    void update(void) override;

protected:

    bool get_reading(float &reading_m);

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:

    bool check_pin();
    void check_stop_pin();
    bool check_pins();
    uint8_t last_stop_pin = -1;

    AP_HAL::PWMSource pwm_source;

    float &estimated_terrain_height;

    // return true if we are beyond the power saving range
    bool out_of_range(void) const;
    bool was_out_of_range = -1; // this odd initialisation ensures we transition to new state

};

#endif  // AP_RANGEFINDER_PWM_ENABLED
