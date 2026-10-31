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

#include "AP_WheelEncoder_config.h"

#if AP_WHEELENCODER_MAV_ENABLED

#include "WheelEncoder_Backend.h"

/*
  Wheel encoder fed by the WHEEL_DISTANCE MAVLink message, for vehicles
  whose encoders are read by an external board rather than by the
  autopilot's own pins.

  Each instance takes its distance from the matching index of the
  message's distance array, so the first instance reads distance[0] and
  so on. Only messages from components on this vehicle are accepted.
*/
class AP_WheelEncoder_MAV : public AP_WheelEncoder_Backend
{
public:
    using AP_WheelEncoder_Backend::AP_WheelEncoder_Backend;

    // readings arrive through handle_msg, so there is nothing to poll
    void update() override {}

    // true if a reading has arrived recently
    bool healthy() const override;

    // readings arrive at the sender's rate
    bool streamed() const override { return true; }

    void handle_msg(const mavlink_message_t &msg) override;
};

#endif  // AP_WHEELENCODER_MAV_ENABLED
