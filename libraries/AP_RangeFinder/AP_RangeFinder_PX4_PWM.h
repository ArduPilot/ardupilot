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

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_PX4_PWM : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_PX4_PWM(RangeFinder::RangeFinder_State &_state, AP_Int16 &powersave_range, float &_estimated_terrain_height);

    // destructor
    ~AP_RangeFinder_PX4_PWM(void);
    
    // static detection function
    static bool detect();

    // update state
    void update(void);

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:
    int _fd;
    uint64_t _last_timestamp;
    uint64_t _last_pulse_time_ms;
    uint32_t _disable_time_ms;
    uint32_t _good_sample_count;
    float _last_sample_distance_cm;

    AP_Int16 &_powersave_range;
    float &estimated_terrain_height;

    // return true if we are beyond the power saving range
    bool out_of_range(void) const {
        return _powersave_range > 0 && estimated_terrain_height > _powersave_range;
    }

};
