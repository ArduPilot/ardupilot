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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_VisualOdom.h"

class AP_VisualOdom_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_VisualOdom_Backend(AP_VisualOdom &frontend);

    // consume VISION_POSITION_DELTA MAVLink message
	virtual void handle_msg(mavlink_message_t *msg) {};

protected:

    // set deltas (used by backend to update state)
    void set_deltas(const Vector3f &angle_delta, const Vector3f& position_delta, uint64_t time_delta_usec, float confidence);

private:

    // references
    AP_VisualOdom &_frontend;
};
