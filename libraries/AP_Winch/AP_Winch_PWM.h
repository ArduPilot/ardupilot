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

#include <AP_Winch/AP_Winch_Backend.h>
#include <SRV_Channel/SRV_Channel.h>

class AP_Winch_PWM : public AP_Winch_Backend {
public:

    using AP_Winch_Backend::AP_Winch_Backend;

    // true if winch is healthy
    bool healthy() const override;

    // control the winch
    void update() override;

    // returns current length of line deployed
    float get_current_length() const override { return line_length; }

    // send status to ground station
    void send_status(const GCS_MAVLINK &channel) override;

    // write log
    void write_log() override;

private:

    // update pwm outputs to control winch
    void control_winch();

    uint32_t control_update_ms; // last time control_winch was called
    float line_length;          // estimated length of line in meters
};
