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
#if AP_CAMERA_OFFBOARD_TRACKING_ENABLED
#include <GCS_MAVLink/GCS.h>
#include "AP_Camera_shareddefs.h"

class AP_Camera_Tracking
{
public:

    // Constructor
    AP_Camera_Tracking() {
      init();
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_Tracking);

    // initialize the camera tracking library
    void init();

    // set tracking to none, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    bool set_tracking(TrackingType tracking_type, const Vector2f& top_left, const Vector2f& bottom_right, uint8_t tracking_device_sysid, uint8_t tracking_device_compid, mavlink_camera_information_t _cam_info);

    // handle MAVLink messages from the camera
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);

    void convert_poi_to_follow_target_message_thread();

private:
    GCS_MAVLINK* _link;

    // Generate synthetic target messages for follow system
    void generate_global_position_int_message(const Location &poi_loc, uint32_t timestamp_ms);
    void generate_follow_target_message(const Location &poi_loc, uint32_t timestamp_ms);

    // Follow integration state
    bool _generate_follow_messages{false};      // True when generating messages for follow system
    bool _tracking_active{false};               // True when tracking is active
    TrackingType _tracking_type{TrackingType::TRK_NONE}; // Current tracking type
    uint32_t _last_tracking_update_ms{0};       // Last tracking command timestamp
    uint32_t _last_follow_message_ms{0};        // Last follow message generation timestamp
    
    // POI location tracking for velocity estimation
    Location _last_poi_location;                // Previous POI location
    uint32_t _last_poi_timestamp_ms{0};         // Previous POI timestamp
    bool _last_poi_location_valid{false};       // True if last POI location is valid
};

#endif // AP_CAMERA_OFFBOARD_TRACKING_ENABLED
