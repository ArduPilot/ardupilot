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

#include <AP_Winch/AP_Winch.h>

class AP_Winch_Backend {
public:
    AP_Winch_Backend(struct AP_Winch::Backend_Config &_config) :
        config(_config) { }

    // true if winch is healthy
    virtual bool healthy() const = 0;

    // initialise the backend
    virtual void init();

    // update - should be called at at least 10hz
    virtual void update() = 0;

    // returns current length of line deployed
    virtual float get_current_length() const = 0;

    // send status to ground station
    virtual void send_status(const GCS_MAVLINK &channel) = 0;

    // write log
    virtual void write_log() = 0;

protected:

    // calculate the pilot desired rate (+ve deploys line, -ve retracts line, 0 stops) from rc input
    // may change the state to RATE and update config.rate_desired
    void read_pilot_desired_rate();

    // returns the rate limited by the maximum acceleration
    // this also updates the previous rate so should not be called more than once per loop
    float get_rate_limited_by_accel(float rate, float dt) WARN_IF_UNUSED;

    // set the rate used for acceleration limiting
    void set_previous_rate(float rate) { previous_rate = rate; }

    struct AP_Winch::Backend_Config &config;

    int16_t previous_radio_in = -1; // previous RC input from pilot, used to ignore small changes
    float previous_rate;            // previous rate used for acceleration limiting
};
