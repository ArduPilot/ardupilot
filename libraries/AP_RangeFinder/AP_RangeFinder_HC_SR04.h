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

class AP_RangeFinder_HC_SR04 : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_HC_SR04(RangeFinder::RangeFinder_State &_state,
                       AP_Int16 &_powersave_range,
                       float &_estimated_terrain_height);

    // destructor
    ~AP_RangeFinder_HC_SR04(void) {};

    // static detection function
    static bool detect();

    // update state
    void update(void) override;

protected:

    bool get_reading(uint16_t &reading_cm);

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:
    uint32_t irq_value_us;          // some of calculated pwm values (irq copy)
    uint32_t irq_pulse_start_us;   // system time of start of pulse
	float max_response_delay_ms;   // max delay for comming back signal
	float distance_m;			  // max distance in meter
    void irq_handler(uint8_t pin, bool pin_high, uint32_t timestamp_us);
    AP_Int16 &powersave_range;
    float &estimated_terrain_height;
	bool is_in_range;

    // return true if we are beyond the power saving range
    bool out_of_range(void) const;
    bool was_out_of_range = -1; // this odd initialisation ensures we transition to new state

};