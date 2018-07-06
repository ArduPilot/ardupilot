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

class AP_RangeFinder_PWM : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_PWM(RangeFinder::RangeFinder_State &_state,
                       AP_RangeFinder_Params &_params,
                       AP_Int16 &_powersave_range,
                       float &_estimated_terrain_height);

    // destructor
    ~AP_RangeFinder_PWM(void) {};

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

    int8_t last_pin; // last pin used for reading pwm (used to recognise change in pin assignment)

    // the following three members are updated by the interrupt handler
    uint32_t irq_value_us;         // some of calculated pwm values (irq copy)
    uint16_t irq_sample_count;     // number of pwm values in irq_value_us (irq copy)
    uint32_t irq_pulse_start_us;   // system time of start of pulse

    void irq_handler(uint8_t pin, bool pin_high, uint32_t timestamp_us);

    void check_pin();
    void check_stop_pin();
    void check_pins();
    uint8_t last_stop_pin = -1;

    AP_Int16 &powersave_range;
    float &estimated_terrain_height;

    // return true if we are beyond the power saving range
    bool out_of_range(void) const;
    bool was_out_of_range = -1; // this odd initialisation ensures we transition to new state

};
