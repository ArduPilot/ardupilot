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

#if AP_CAMERA_MOUNT_ENABLED

class AP_Camera_Mount : public AP_Camera_Backend
{
public:

    // Constructor
    using AP_Camera_Backend::AP_Camera_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_Mount);

    // entry point to actually take a picture.  returns true on success
    bool trigger_pic() override;

    // start or stop video recording.  returns true on success
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set camera zoom step.  returns true on success
    // zoom out = -1, hold = 0, zoom in = 1
    bool set_zoom_step(int8_t zoom_step) override;

    // set focus in, out or hold.  returns true on success
    // focus in = -1, focus hold = 0, focus out = 1
    bool set_manual_focus_step(int8_t focus_step) override;

    // auto focus.  returns true on success
    bool set_auto_focus() override;
};

#endif // AP_CAMERA_MOUNT_ENABLED
