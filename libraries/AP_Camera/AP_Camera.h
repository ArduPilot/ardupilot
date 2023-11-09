/// @file	AP_Camera.h
/// @brief	Photo or video camera manager, with EEPROM-backed storage of constants.
#pragma once

#include "AP_Camera_config.h"

#if AP_CAMERA_ENABLED

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_Camera_Params.h"
#include "AP_Camera_shareddefs.h"

#define AP_CAMERA_MAX_INSTANCES             2       // maximum number of camera backends

// declare backend classes
class AP_Camera_Backend;
class AP_Camera_Servo;
class AP_Camera_Relay;
class AP_Camera_SoloGimbal;
class AP_Camera_Mount;
class AP_Camera_MAVLink;
class AP_Camera_MAVLinkCamV2;
class AP_Camera_Scripting;

/// @class	Camera
/// @brief	Object managing a Photo or video camera
class AP_Camera {

    // declare backends as friends
    friend class AP_Camera_Backend;
    friend class AP_Camera_Servo;
    friend class AP_Camera_Relay;
    friend class AP_Camera_SoloGimbal;
    friend class AP_Camera_Mount;
    friend class AP_Camera_MAVLink;
    friend class AP_Camera_MAVLinkCamV2;
    friend class AP_Camera_Scripting;

public:

    // constructor
    AP_Camera(uint32_t _log_camera_bit);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera);

    // get singleton instance
    static AP_Camera *get_singleton() { return _singleton; }

    // enums
    enum class CameraType {
        NONE = 0,           // None
#if AP_CAMERA_SERVO_ENABLED
        SERVO = 1,          // Servo/PWM controlled camera
#endif
#if AP_CAMERA_RELAY_ENABLED
        RELAY = 2,          // Relay controlled camera
#endif
#if AP_CAMERA_SOLOGIMBAL_ENABLED
        SOLOGIMBAL = 3,     // GoPro in Solo gimbal
#endif
#if AP_CAMERA_MOUNT_ENABLED
        MOUNT = 4,          // Mount library implements camera
#endif
#if AP_CAMERA_MAVLINK_ENABLED
        MAVLINK = 5,        // MAVLink enabled camera
#endif
#if AP_CAMERA_MAVLINKCAMV2_ENABLED
        MAVLINK_CAMV2 = 6,  // MAVLink camera v2
#endif
#if AP_CAMERA_SCRIPTING_ENABLED
        SCRIPTING = 7,  // Scripting backend
#endif
    };

    // detect and initialise backends
    void init();

    // update - to be called periodically at 50Hz
    void update();

    // handle MAVLink messages from the camera
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);

    // handle MAVLink command from GCS to control the camera
    MAV_RESULT handle_command(const mavlink_command_int_t &packet);

    // send camera feedback message to GCS
    void send_feedback(mavlink_channel_t chan);

    // send camera information message to GCS
    void send_camera_information(mavlink_channel_t chan);

    // send camera settings message to GCS
    void send_camera_settings(mavlink_channel_t chan);

#if AP_CAMERA_SEND_FOV_STATUS_ENABLED
    // send camera field of view status
    void send_camera_fov_status(mavlink_channel_t chan);
#endif

    // send camera capture status message to GCS
    void send_camera_capture_status(mavlink_channel_t chan);

    // configure camera
    void configure(float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time);
    void configure(uint8_t instance, float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time);

    // handle camera control
    void control(float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id);
    void control(uint8_t instance, float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id);

    // set camera trigger distance in a mission
    void set_trigger_distance(float distance_m);
    void set_trigger_distance(uint8_t instance, float distance_m);

    // momentary switch to change camera between picture and video modes
    void cam_mode_toggle();
    void cam_mode_toggle(uint8_t instance);

    // take a picture.  If instance is not provided, all available cameras affected
    // returns true if at least one camera took a picture
    bool take_picture();
    bool take_picture(uint8_t instance);

    // take multiple pictures, time_interval between two consecutive pictures is in miliseconds
    // if instance is not provided, all available cameras affected
    // time_interval_ms must be positive
    // total_num is number of pictures to be taken, -1 means capture forever
    // returns true if at least one camera is successful
    bool take_multiple_pictures(uint32_t time_interval_ms, int16_t total_num);
    bool take_multiple_pictures(uint8_t instance, uint32_t time_interval_ms, int16_t total_num);

    // stop capturing multiple image sequence
    void stop_capture();
    bool stop_capture(uint8_t instance);

    // start/stop recording video
    // start_recording should be true to start recording, false to stop recording
    bool record_video(bool start_recording);
    bool record_video(uint8_t instance, bool start_recording);

    // set zoom specified as a rate or percentage
    bool set_zoom(ZoomType zoom_type, float zoom_value);
    bool set_zoom(uint8_t instance, ZoomType zoom_type, float zoom_value);

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(FocusType focus_type, float focus_value);
    SetFocusResult set_focus(uint8_t instance, FocusType focus_type, float focus_value);

    // set tracking to none, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2);
    bool set_tracking(uint8_t instance, TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2);

    // set camera lens as a value from 0 to 5
    bool set_lens(uint8_t lens);
    bool set_lens(uint8_t instance, uint8_t lens);

    // set if vehicle is in AUTO mode
    void set_is_auto_mode(bool enable) { _is_in_auto_mode = enable; }

#if AP_CAMERA_SCRIPTING_ENABLED
    // structure and accessors for use by scripting backends
    typedef struct {
        uint16_t take_pic_incr; // incremented each time camera is requested to take a picture
        bool recording_video;   // true when recording video
        uint8_t zoom_type;      // see ZoomType enum (1:Rate or 2:Pct)
        float zoom_value;       // percentage or zoom out = -1, hold = 0, zoom in = 1
        uint8_t focus_type;     // see FocusType enum (1:Rate, 2:Pct, 4:Auto)
        float focus_value;      // If Rate, focus in = -1, focus hold = 0, focus out = 1.  If PCT 0 to 100
        uint8_t tracking_type;  // see TrackingType enum (0:NONE, 1:POINT, 2:RECTANGLE)
        Vector2f tracking_p1;   // center or top-left tracking point. x left-right, y is top-bottom. range is 0 to 1
        Vector2f tracking_p2;   // bottom-right tracking point. x left-right, y is top-bottom. range is 0 to 1
    } camera_state_t;

    // accessor to allow scripting backend to retrieve state
    // returns true on success and cam_state is filled in
    bool get_state(uint8_t instance, camera_state_t& cam_state);
#endif

    // allow threads to lock against AHRS update
    HAL_Semaphore &get_semaphore() { return _rsem; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // return true if vehicle mode allows trigg dist
    bool vehicle_mode_ok_for_trigg_dist() const { return (_auto_mode_only == 0) || _is_in_auto_mode; }

    // return maximum acceptable vehicle roll angle (in degrees)
    int16_t get_roll_max() const { return _max_roll; }

    // return log bit
    uint32_t get_log_camera_bit() const { return log_camera_bit; }

    // parameters for backends
    AP_Camera_Params _params[AP_CAMERA_MAX_INSTANCES];

private:

    static AP_Camera *_singleton;

    // parameters
    AP_Int8 _auto_mode_only;    // if 1: trigger by distance only if in AUTO mode.
    AP_Int16 _max_roll;         // Maximum acceptable roll angle when trigging camera

    // check instance number is valid
    AP_Camera_Backend *get_instance(uint8_t instance) const;

    // perform any required parameter conversion
    void convert_params();

    HAL_Semaphore _rsem;                // semaphore for multi-thread access
    AP_Camera_Backend *primary;         // primary camera backed
    bool _is_in_auto_mode;              // true if in AUTO mode
    uint32_t log_camera_bit;            // logging bit (from LOG_BITMASK) to enable camera logging
    AP_Camera_Backend *_backends[AP_CAMERA_MAX_INSTANCES];  // pointers to instantiated backends
};

namespace AP {
AP_Camera *camera();
};

#endif
