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
  Camera driver backend class. Each supported camera type
  needs to have an object derived from this class.
 */
#pragma once

#include "AP_Camera_config.h"

#if AP_CAMERA_ENABLED
#include "AP_Camera.h"
#include <AP_Common/Location.h>
#include <AP_Logger/LogStructure.h>

class AP_Camera_Backend
{
public:

    // Constructor
    AP_Camera_Backend(AP_Camera &frontend, AP_Camera_Params &params, uint8_t instance);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_Backend);

    // camera options parameter values
    enum class Options : int8_t {
        RecordWhileArmed = (1 << 0U)
    };

    // init - performs any required initialisation
    virtual void init() {};

    // update - should be called at 50hz
    virtual void update();

    // return true if healthy
    virtual bool healthy() const { return true; }

    // momentary switch to change camera between picture and video modes
    virtual void cam_mode_toggle() {}

    // take a picture.  returns true on success
    bool take_picture();

    // take multiple pictures, time_interval between two consecutive pictures is in miliseconds
    // total_num is number of pictures to be taken, -1 means capture forever
    void take_multiple_pictures(uint32_t time_interval_ms, int16_t total_num);

    // stop capturing multiple image sequence
    void stop_capture();

    // entry point to actually take a picture.  returns true on success
    virtual bool trigger_pic() = 0;

    // start or stop video recording.  returns true on success
    // set start_recording = true to start record, false to stop recording
    virtual bool record_video(bool start_recording) { return false; }

    // set zoom specified as a rate or percentage
    virtual bool set_zoom(ZoomType zoom_type, float zoom_value) { return false; }

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    virtual SetFocusResult set_focus(FocusType focus_type, float focus_value) { return SetFocusResult::UNSUPPORTED; }

    // set tracking to none, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    virtual bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2) { return false; }

    // set camera lens as a value from 0 to 5
    virtual bool set_lens(uint8_t lens) { return false; }

#if AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
    // set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
    virtual bool set_camera_source(AP_Camera::CameraSource primary_source, AP_Camera::CameraSource secondary_source) { return false; }
#endif

    // get camera image horizontal or vertical field of view in degrees.  returns 0 if unknown
    float horizontal_fov() const { return MAX(0, _params.hfov); }
    float vertical_fov() const { return MAX(0, _params.vfov); }

    // handle MAVLink messages from the camera
    virtual void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg) {}

    // configure camera
    virtual void configure(float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time) {}

    // handle camera control
    virtual void control(float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id);

    // set camera trigger distance in meters
    void set_trigger_distance(float distance_m) { _params.trigg_dist.set(distance_m); }

    // send camera feedback message to GCS
    void send_camera_feedback(mavlink_channel_t chan);

    // send camera information message to GCS
    virtual void send_camera_information(mavlink_channel_t chan) const;

    // send camera settings message to GCS
    virtual void send_camera_settings(mavlink_channel_t chan) const;

#if AP_CAMERA_SEND_FOV_STATUS_ENABLED
    // send camera field of view status
    void send_camera_fov_status(mavlink_channel_t chan) const;
#endif

    // send camera capture status message to GCS
    virtual void send_camera_capture_status(mavlink_channel_t chan) const;

#if AP_CAMERA_SCRIPTING_ENABLED
    // accessor to allow scripting backend to retrieve state
    // returns true on success and cam_state is filled in
    virtual bool get_state(AP_Camera::camera_state_t& cam_state) { return false; }
#endif

protected:

    // references
    AP_Camera &_frontend;       // reference to the front end which holds parameters
    AP_Camera_Params &_params;  // parameters for this backend

    // feedback pin related methods
    void setup_feedback_callback();
    void feedback_pin_isr(uint8_t pin, bool high, uint32_t timestamp_us);
    void feedback_pin_timer();
    void check_feedback();

    // store vehicle location and attitude for use in camera_feedback message to GCS
    void prep_mavlink_msg_camera_feedback(uint64_t timestamp_us);
    struct {
        uint64_t timestamp_us;      // system time of most recent image
        Location location;          // location where most recent image was taken
        int32_t roll_sensor;        // vehicle roll in centi-degrees
        int32_t pitch_sensor;       // vehicle pitch in centi-degrees
        int32_t yaw_sensor;         // vehicle yaw in centi-degrees
        uint32_t feedback_trigger_logged_count; // ID sequence number
    } camera_feedback;

    // Picture settings
    struct {
        uint32_t time_interval_ms;     // time interval (in miliseconds) between two consecutive pictures
        int16_t num_remaining;      // number of pictures still to be taken, -1 means take unlimited pictures
    } time_interval_settings;

    // Logging Function
    void log_picture();
    void Write_Camera(uint64_t timestamp_us=0);
    void Write_Trigger();
    void Write_CameraInfo(enum LogMessages msg, uint64_t timestamp_us=0);

    // get corresponding mount instance for the camera
    uint8_t get_mount_instance() const;

    // get mavlink gimbal device id which is normally mount_instance+1
    uint8_t get_gimbal_device_id() const;

    // internal members
    uint8_t _instance;      // this instance's number
    bool timer_installed;   // true if feedback pin change detected using timer
    bool isr_installed;     // true if feedback pin change is detected with an interrupt
    uint8_t last_pin_state; // last pin state.  used by timer based detection
    uint32_t feedback_trigger_count;        // number of times the interrupt detected the feedback pin changed
    uint32_t feedback_trigger_timestamp_us; // system time (in microseconds) that timer detected the feedback pin changed
    uint32_t feedback_trigger_logged_count; // number of times the feedback has been logged
    bool trigger_pending;           // true if a call to take_pic() was delayed due to the minimum time interval time
    uint32_t last_picture_time_ms;    // system time that photo was last taken
    Location last_location;         // Location that last picture was taken at (used for trigg_dist calculation)
    uint16_t image_index;           // number of pictures taken since boot
    bool last_is_armed;             // stores last arm/disarm state. true if it was armed lastly
};

#endif // AP_CAMERA_ENABLED
