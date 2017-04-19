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
  simple Gripper (EPM) simulation class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

class Gripper_EPM {
public:
    const uint8_t gripper_servo;

    Gripper_EPM(const uint8_t _gripper_servo) :
        gripper_servo(_gripper_servo)
    {}

    // update field stength
    void update(const struct Aircraft::sitl_input &input);

private:

    const uint32_t report_interval = 100000; // microseconds
    uint64_t last_report_us;

    bool servo_based = true;

    double field_strength; // percentage
    double reported_field_strength = -1; // unlikely

    // I've a feeling these are probably a higher order than this:
    const float field_strength_slew_rate = 400; // (percentage of delta between field strength and 100)/second
    const float field_decay_rate = 2; // percent of field strength/second
    const float field_degauss_rate = 300; // percent of field strength/second

    uint64_t last_update_us;

    bool should_report();

    void update_from_demand(const Aircraft::sitl_input &input);
    void update_servobased(const struct Aircraft::sitl_input &input);

    float tesla();

    float demand;
};

}
