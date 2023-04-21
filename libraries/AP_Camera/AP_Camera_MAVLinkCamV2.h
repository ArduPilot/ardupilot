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
  Camera driver for cameras that implement the newer MAVLink camera v2 protocol
  see https://mavlink.io/en/services/camera.html
 */
#pragma once

#include "AP_Camera_Backend.h"

#if AP_CAMERA_MAVLINKCAMV2_ENABLED

class AP_Camera_MAVLinkCamV2 : public AP_Camera_Backend
{
public:

    // Constructor
    using AP_Camera_Backend::AP_Camera_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_MAVLinkCamV2);

    // update - should be called at 50hz
    void update() override;

    // entry point to actually take a picture
    bool trigger_pic() override;

    // start or stop video recording.  returns true on success
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set zoom specified as a rate or percentage
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    // set focus in, out or hold.  returns true on success
    // focus in = -1, focus hold = 0, focus out = 1
    bool set_manual_focus_step(int8_t focus_step) override;

    // auto focus.  returns true on success
    bool set_auto_focus() override;

    // handle incoming mavlink message
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg) override;

private:

    // search for camera in GCS_MAVLink routing table
    void find_camera();

    // request CAMERA_INFORMATION (holds vendor and model name)
    void request_camera_information() const;

    // internal members
    bool _initialised;          // true once the camera has provided a CAMERA_INFORMATION
    bool _got_camera_info;      // true once camera has provided CAMERA_INFORMATION
    uint32_t _last_caminfo_req_ms;  // system time that CAMERA_INFORMATION was last requested (used to throttle requests)
    class GCS_MAVLINK *_link;   // link we have found the camera on. nullptr if not seen yet
    uint8_t _sysid;             // sysid of camera
    uint8_t _compid;            // component id of gimbal
    uint32_t _cap_flags;        // capability flags from CAMERA_INFORMATION msg, see MAVLink CAMERA_CAP_FLAGS enum
};

#endif // AP_CAMERA_MAVLINKCAMV2_ENABLED
