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
  Camera Tracking Library
 */
#pragma once

#include "AP_Camera_config.h"
#if AP_CAMERA_TRACKING_ENABLED
#include <GCS_MAVLink/GCS.h>
#include "AP_Camera_shareddefs.h"
#include "AP_Camera_config.h"

// #include "AP_Camera.h"

class AP_Camera_Tracking
{
public:

    // Constructor
    AP_Camera_Tracking() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_Tracking);

    // initialize the camera tracking library
    void init();

    // set tracking to non      nj klbhnjk nm,e, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    bool set_tracking(TrackingType tracking_type, const Vector2f& top_left, const Vector2f& bottom_right, uint8_t tracking_device_sysid, uint8_t tracking_device_compid, mavlink_camera_information_t _cam_info);

    // handle MAVLink messages from the camera
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);

private:
    GCS_MAVLINK* _link;
};

#endif // AP_CAMERA_TRACKING_ENABLED
