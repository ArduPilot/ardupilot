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
  Camera servo driver backend class
 */
#pragma once

#include "AP_Camera_Backend.h"

#if AP_CAMERA_SERVO_ENABLED

class AP_Camera_Servo : public AP_Camera_Backend
{
public:

    // Constructor
    using AP_Camera_Backend::AP_Camera_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_Servo);

    // update - should be called at 50hz
    void update() override;

    // entry point to actually take a picture.  returns true on success
    bool trigger_pic() override;

    // configure camera
    void configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time) override;

private:

    uint16_t trigger_counter;   // count of number of cycles shutter should be held open
    uint16_t iso_counter;       // count of number of cycles iso output should be held open
};

#endif // AP_CAMERA_SERVO_ENABLED
