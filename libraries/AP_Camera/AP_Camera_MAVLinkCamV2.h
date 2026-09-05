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

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    // handle MAVLink messages from the camera
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg) override;

    // send camera information message to GCS
    void send_camera_information(mavlink_channel_t chan) const override;

    // send cached remote camera capture/recording status to GCS
    void send_camera_capture_status(mavlink_channel_t chan) const override;

#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
    // send cached video stream information messages to GCS
    void send_video_stream_information(mavlink_channel_t chan) const override;
#endif

private:

    // search for camera in GCS_MAVLink routing table
    void find_camera();

    // request CAMERA_INFORMATION (holds vendor and model name)
    void request_camera_information() const;

    // request CAMERA_CAPTURE_STATUS from the remote camera
    void request_camera_capture_status();

#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
    // request and cache VIDEO_STREAM_INFORMATION from the remote camera
    void request_video_stream_information();
    bool video_stream_information_complete() const;
    void reset_video_stream_information(uint8_t stream_count);

    // Bound RAM use while supporting multi-sensor cameras.
    mavlink_video_stream_information_t
    *_video_stream_info[AP_CAMERA_MAVLINKCAMV2_MAX_VIDEO_STREAMS];
    uint8_t _video_stream_count;
    bool _video_stream_info_empty;
    uint32_t _last_stream_info_req_ms;
#endif

    // internal members
    bool _initialised;          // true once the camera has provided a CAMERA_INFORMATION
    bool _got_camera_info;      // true once camera has provided CAMERA_INFORMATION
    mavlink_camera_information_t _cam_info {}; // latest camera information received from camera
    mavlink_camera_capture_status_t _capture_status; // latest recording/capture status
    bool _got_capture_status;    // true once camera has provided CAMERA_CAPTURE_STATUS
    uint32_t _last_capture_status_ms; // receipt time of the cached status
    uint32_t _last_capture_status_req_ms; // last remote status request
    uint32_t _last_caminfo_req_ms;  // system time that CAMERA_INFORMATION was last requested (used to throttle requests)
    class GCS_MAVLINK *_link;   // link we have found the camera on. nullptr if not seen yet
    uint8_t _sysid;             // sysid of camera
    uint8_t _compid;            // component id of gimbal
};

#endif // AP_CAMERA_MAVLINKCAMV2_ENABLED
