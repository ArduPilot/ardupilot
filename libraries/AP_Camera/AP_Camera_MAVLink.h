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
  Camera driver for cameras included in Mount
 */
#pragma once

#include "AP_Camera_Backend.h"

#if AP_CAMERA_MAVLINK_ENABLED

class AP_Camera_MAVLink : public AP_Camera_Backend
{
public:

    // Constructor
    using AP_Camera_Backend::AP_Camera_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_MAVLink);

    // entry point to actually take a picture
    bool trigger_pic() override;

    // configure camera
    void configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time) override;

    // handle camera control message
    void control(float session, float zoom_pos, float zoom_step, float focus_lock, float shooting_cmd, float cmd_id) override;
};

#endif // AP_CAMERA_MAVLINK_ENABLED
