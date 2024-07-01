#include "AP_Mount_Topotek.h"

#if HAL_MOUNT_TOPOTEK_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Filesystem/posix_compat.h>
#include <ctime>

extern const AP_HAL::HAL& hal;

#define ANGULAR_VELOCITY_CONVERSION         1.220740379         // gimbal angular velocity conversion ratio
#define TRACK_TOTAL_WIDTH                   1920                // track the maximum width of the image range
#define TRACK_TOTAL_HEIGHT                  1080                // track the maximum height of the image range
#define TRACK_RANGE                         60                  // the size of the image at point tracking
#define AP_MOUNT_TOPOTEK_UPDATE_INTERVAL_MS 100                 // resend angle or rate targets to gimbal at this interval
#define AP_MOUNT_TOPOTEK_HEALTH_TIMEOUT_MS  1000                // timeout for health and rangefinder readings
#define AP_MOUNT_TOPOTEK_ROTATION_SPEED     99                  // the speed of the gimbal when controlling angles

#define AP_MOUNT_TOPOTEK_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_TOPOTEK_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Topotek: " fmt, ## args); } } while (0)


const char* AP_Mount_Topotek::send_message_prefix = "Mount Topotek:";

// control gimbal command
int8_t AP_Mount_Topotek::_zoom_cmd[15] = "#TPUM2wZMC00";                        // zoom command
int8_t AP_Mount_Topotek::_start_record_video[15] = "#TPUD2wREC00";              // record command
int8_t AP_Mount_Topotek::_take_pic[15] = "#TPUD2wCAP01";                        // photo command
int8_t AP_Mount_Topotek::_focus_cmd[15] = "#TPUM2wFCC00";                       // focus command
int8_t AP_Mount_Topotek::_stop_track_cmd[15] = "#TPUD2wTRC01";                  // stop track command
int8_t AP_Mount_Topotek::_begin_track_cmd[25] = "#tpUDAwLOC";                   // begin track command
int8_t AP_Mount_Topotek::_next_pip_mode[15] = "#TPUD2wPIP0A";                   // picture-in-picture command
int8_t AP_Mount_Topotek::_get_gimbal_attitude[15] = "#TPUG2wGIA01";             // get gimbal attitude command
int8_t AP_Mount_Topotek::_get_gimbal_sdcard_info[15] = "#TPUD2rSDC00";          // get gimbal memory card information command
int8_t AP_Mount_Topotek::_get_gimbal_track_status[15] = "#TPUD2rTRC00";         // get gimbal tracking status command
int8_t AP_Mount_Topotek::_get_gimbal_basic_info[15] = "#tpUD2rVSN00";           // get gimbal basic information command
int8_t AP_Mount_Topotek::_set_yaw_angle_cmd[20] = "#tpUG6wGIY";                 // set gimbal yaw angle command
int8_t AP_Mount_Topotek::_set_pitch_angle_cmd[20] = "#tpUG6wGIP";               // set gimbal pitch angle command
int8_t AP_Mount_Topotek::_set_roll_angle_cmd[20] = "#tpUG6wGIR";                // set gimbal roll angle command
int8_t AP_Mount_Topotek::_set_yaw_pitch_roll_speed_cmd[20] = "#tpUG6wYPR";      // set the speed of gimbal yaw, pitch and roll command
int8_t AP_Mount_Topotek::_set_gimbal_time[45] = "#tpUDCwUTC";                   // set the gimbal time command
int8_t AP_Mount_Topotek::_set_gimbal_lat[25] = "#tpUDAwLAT";                    // set the gimbal's latitude
int8_t AP_Mount_Topotek::_set_gimbal_lng[25] = "#tpUDBwLON";                    // set the gimbal's longitude
int8_t AP_Mount_Topotek::_set_gimbal_alt[25] = "#tpUD8wALT";                    // set the gimbal's altitude
int8_t AP_Mount_Topotek::_set_gimbal_azi[20] = "#tpUD5wAZI";                    // send heading information to the gimbal
int8_t AP_Mount_Topotek::_set_gimbal_range_enable[15] = "#TPUM2wLRF00";         // open and close ranging command
int8_t AP_Mount_Topotek::_gimbal_control_stop[15] = "#TPUG2wPTZ00";             // stop control of the gimbal command
int8_t AP_Mount_Topotek::_gimbal_lock[15] = "#TPUG2wPTZ06";                     // set whether the gimbal is locked or followed command

// init - performs any required initialisation for this instance
void AP_Mount_Topotek::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Gimbal, 0);
    if (_uart != nullptr) {
        return;
    }

    _initialised = true;
    AP_Mount_Backend::init();
}

// update mount position - should be called periodically
void AP_Mount_Topotek::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }
    // reading incoming packets from gimbal
    read_incoming_packets();

    uint32_t now_ms = AP_HAL::millis();

    // send relevant messages to the gimbal within the specified time frame.
    if ((now_ms - _last_update_ms) < AP_MOUNT_TOPOTEK_UPDATE_INTERVAL_MS) {
        return;
    }

    _last_update_ms = now_ms;
    // send the stop zoom command a second time to prevent data transmission errors.
    if (_last_zoom_stop) {
        _last_zoom_stop = false;
        send_packet(_zoom_cmd, 12);
    }

    // send the stop focus command a second time to prevent data transmission errors.
    if (_last_focus_stop) {
        _last_focus_stop = false;
        send_packet(_focus_cmd, 12);
    }

    // get gimbal basic information
    if (!_got_gimbal_basic_info) {
        request_gimbal_basic_info();
    }

    if (_last_req_count % 3 == 0) {
        // request memory card information within the specified time
        request_gimbal_sdcard_info();

        // send GPS-related information to the gimbal.
        send_location_info();

        // if trace mode is enabled, trace status information is requested within a specified period of time
        if (_is_tracking) {
            request_track_status();
        }
    }

    // the gimbal attitude is obtained every second, and the gimbal will continue to send attitude information during the next period
    if (_last_req_count % 11 == 0) {
        request_gimbal_attitude();
        if (!_last_req_count) {
            _last_req_count = 0;
        }
    }
    _last_req_count++;

    // update based on mount mode
    switch (get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &angle_bf_target = _params.retract_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &angle_bf_target = _params.neutral_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // mavlink targets are stored while handling the incoming message
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's RC inputs
            MountTarget rc_target;
            get_rc_target(mnt_target.target_type, rc_target);
            switch (mnt_target.target_type) {
            case MountTargetType::ANGLE:
                mnt_target.angle_rad = rc_target;
                break;
            case MountTargetType::RATE:
                mnt_target.rate_rads = rc_target;
                break;
            }
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to Home location
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to another vehicle
        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        default:
            // we do not know this mode so raise internal error
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }

    // send target angles or rates depending on the target type
    switch (mnt_target.target_type) {
        case MountTargetType::ANGLE:
            send_angle_target(mnt_target.angle_rad.roll, mnt_target.angle_rad.pitch, mnt_target.angle_rad.yaw, mnt_target.angle_rad.yaw_is_ef);
            break;
        case MountTargetType::RATE:
            send_rate_target(mnt_target.rate_rads.roll, mnt_target.rate_rads.pitch, mnt_target.rate_rads.yaw, mnt_target.rate_rads.yaw_is_ef);
            break;
    }

}

// return true if healthy
bool AP_Mount_Topotek::healthy() const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // unhealthy if attitude information not received recently
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_current_angle_ms > AP_MOUNT_TOPOTEK_HEALTH_TIMEOUT_MS) {
        return false;
    }

    return true;
}

// take a picture.  returns true on success
bool AP_Mount_Topotek::take_picture()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // exit immediately if the memory card is abnormal
    if (!_sdcard_status) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s sd card error", send_message_prefix);
        return false;
    }

    return send_packet(_take_pic, 12);
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_Topotek::record_video(bool start_recording)
{

    // exit immediately if not initialised to reduce mismatch
    // between internal and actual state of recording
    if (!_initialised) {
        return false;
    }

    // exit immediately if the memory card is abnormal
    if (!_sdcard_status) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s sd card error", send_message_prefix);
        return false;
    }

    if (start_recording) {
        _start_record_video[11] = '1';
    } else {
        _start_record_video[11] = '0';
    }

    return send_packet(_start_record_video, 12);
}

// set zoom specified as a rate
bool AP_Mount_Topotek::set_zoom(ZoomType zoom_type, float zoom_value)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    _zoom_type = zoom_type;
    _zoom_value = zoom_value;
    // zoom rate
    if (zoom_type == ZoomType::RATE) {
        // zoom stop
        _zoom_cmd[11] = '0';
        if (zoom_value < 0) {
            // zoom out
            _zoom_cmd[11] = '1';
        } else if (zoom_value > 0) {
            // zoom in
            _zoom_cmd[11] = '2';
        }

        // record when the zooming stops, either zooming in or out.
        if (_zoom_cmd[11] == '0') {
            _last_zoom_stop = true;
        }

        return send_packet(_zoom_cmd, 12);
    }

    // unsupported zoom type
    return false;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Mount_Topotek::set_focus(FocusType focus_type, float focus_value)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return SetFocusResult::FAILED;
    }

    switch (focus_type) {
    case FocusType::RATE: {
        // focus stop
        _focus_cmd[10] = '0';
        _focus_cmd[11] = '0';
        if (focus_value < 0) {
            // focus-
            _focus_cmd[11] = '2';
        } else if (focus_value > 0) {
            // focus+
            _focus_cmd[11] = '1';
        }
        // record when the focus is stopped.
        if (_focus_cmd[10] == '0' && _focus_cmd[11] == '0') {
            _last_focus_stop = true;
        }

        if (send_packet(_focus_cmd, 12)) {
            // manual focus
            _focus_cmd[10] = '1';
            _focus_cmd[11] = '1';
            if (send_packet(_focus_cmd, 12)) {
                return SetFocusResult::ACCEPTED;
            }
        }
        return SetFocusResult::FAILED;
    }
    case FocusType::PCT:
        // not supported
        return SetFocusResult::INVALID_PARAMETERS;
    case FocusType::AUTO:
        // auto focus
        _focus_cmd[10] = '1';
        _focus_cmd[11] = '0';
        if (send_packet(_focus_cmd, 12)) {
            return SetFocusResult::ACCEPTED;
        }
        return SetFocusResult::FAILED;
    }

    // unsupported focus type
    return SetFocusResult::INVALID_PARAMETERS;
}

// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
// p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Mount_Topotek::set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    switch (tracking_type) {
    case TrackingType::TRK_NONE:
        _is_tracking = false;
        return send_packet(_stop_track_cmd, 12);
    case TrackingType::TRK_POINT: {
        // the converted tracking center
        int16_t track_center_x = (int16_t)((p1.x*TRACK_TOTAL_WIDTH - 960) /  0.96);
        int16_t track_center_y = (int16_t)((p1.y*TRACK_TOTAL_HEIGHT - 540) /  0.54);
        // tracking range after conversion
        int16_t track_width = (int16_t)(TRACK_RANGE / 0.96);
        int16_t track_height = (int16_t)(TRACK_RANGE / 0.54);

        _begin_track_cmd[10] = (track_center_x >> 8) & 0xff;
        _begin_track_cmd[11] = track_center_x & 0xff;

        _begin_track_cmd[12] = (track_center_y >> 8) & 0xff;
        _begin_track_cmd[13] =  track_center_y & 0xff;

        _begin_track_cmd[14] = (track_width >> 8) & 0xff;
        _begin_track_cmd[15] =  track_width & 0xff;

        _begin_track_cmd[16] = (track_height >> 8) & 0xff;
        _begin_track_cmd[17] =  track_height & 0xff;

        _begin_track_cmd[18] = 0;
        _begin_track_cmd[19] = 1;

        _is_tracking = true;
        return send_packet(_begin_track_cmd, 20);
    }
    case TrackingType::TRK_RECTANGLE:
        // upper left point
        int16_t upper_leftx;
        int16_t upper_lefty;
        // bottom right point
        int16_t down_rightx;
        int16_t down_righty;

        // frame selection range
        int16_t frame_selection_width;
        int16_t frame_selection_height;

        if (p1.x < p2.x) {
            upper_leftx = (int16_t)(p1.x*TRACK_TOTAL_WIDTH);
            down_rightx = (int16_t)(p2.x*TRACK_TOTAL_WIDTH);
        } else {
            upper_leftx = (int16_t)(p2.x*TRACK_TOTAL_WIDTH);
            down_rightx = (int16_t)(p1.x*TRACK_TOTAL_WIDTH);
        }

        if (p1.y < p2.y) {
            upper_lefty = (int16_t)(p1.y*TRACK_TOTAL_HEIGHT);
            down_righty = (int16_t)(p2.y*TRACK_TOTAL_HEIGHT);
        } else {
            upper_lefty = (int16_t)(p2.y*TRACK_TOTAL_HEIGHT);
            down_righty = (int16_t)(p1.y*TRACK_TOTAL_HEIGHT);
        }

        // calculated tracking range
        frame_selection_width = down_rightx - upper_leftx;
        frame_selection_height = down_righty - upper_lefty;

        if (frame_selection_width <= 0 || frame_selection_height <= 0) {
            return false;
        }

        // trace the central point calculation
        upper_leftx = upper_leftx + frame_selection_width/2;
        upper_lefty = upper_lefty + frame_selection_height/2;

        // the converted tracking center
        int16_t track_center_x = (int16_t)((upper_leftx - 960) /  0.96);
        int16_t track_center_y = (int16_t)((upper_lefty - 540) /  0.54);
        // tracking range after conversion
        int16_t track_width = (int16_t)(frame_selection_width / 0.96);
        int16_t track_height = (int16_t)(frame_selection_height / 0.54);

        _begin_track_cmd[10] = (track_center_x >> 8) & 0xff;
        _begin_track_cmd[11] = track_center_x & 0xff;

        _begin_track_cmd[12] = (track_center_y >> 8) & 0xff;
        _begin_track_cmd[13] =  track_center_y & 0xff;

        _begin_track_cmd[14] = (track_width >> 8) & 0xff;
        _begin_track_cmd[15] =  track_width & 0xff;

        _begin_track_cmd[16] = (track_height >> 8) & 0xff;
        _begin_track_cmd[17] =  track_height & 0xff;

        _begin_track_cmd[18] = 0;
        _begin_track_cmd[19] = 1;

        _is_tracking = true;
        return send_packet(_begin_track_cmd, 20);

    }

    // should never reach here
    return false;
}

// set camera picture-in-picture mode
bool AP_Mount_Topotek::set_lens(uint8_t lens)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }
    if (lens > 0) {
        _next_pip_mode[11] = 'A';
        return send_packet(_next_pip_mode, 12);
    }
    return false;
}

// set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
// primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
bool AP_Mount_Topotek::set_camera_source(uint8_t primary_source, uint8_t secondary_source)
{
    // maps primary and secondary source to topotek image sensor
    switch (primary_source) {
    case 0: // Default (RGB)
        _next_pip_mode[11] = '0';
        break;
    case 1: // RGB
        switch (secondary_source) {
        case 0: // RGB + Default (None)
            _next_pip_mode[11] = '0';
            break;
        case 2: // PIP RGB+IR
            _next_pip_mode[11] = '1';
            break;
        default:
            return false;
        }
        break;
    case 2: // IR
        switch (secondary_source) {
        case 0: // IR + Default (None)
            _next_pip_mode[11] = '3';
            break;
        case 1: // PIP IR+RGB
            _next_pip_mode[11] = '2';
            break;
        default:
            return false;
        }
        break;
    default:
        return false;
    }

    // send desired image type to camera
    return send_packet(_next_pip_mode, 12);
}

// send camera information message to GCS
void AP_Mount_Topotek::send_camera_information(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    static const uint8_t vendor_name[32] = "Topotek";
    static uint8_t model_name[32] {};
    const char cam_definition_uri[140] {};

    if (_got_gimbal_basic_info) {
        strcpy((char*)model_name, (const char*)_model_name);
    }
    // capability flags
    const uint32_t flags = CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                           CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                           CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM |
                           CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS |
                           CAMERA_CAP_FLAGS_HAS_TRACKING_POINT |
                           CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE;

    // send CAMERA_INFORMATION message
    mavlink_msg_camera_information_send(
        chan,
        AP_HAL::millis(),       // time_boot_ms
        vendor_name,            // vendor_name uint8_t[32]
        model_name,             // model_name uint8_t[32]
        _firmware_ver,          // firmware version uint32_t
        0,                      // focal_length float (mm)
        0,                      // sensor_size_h float (mm)
        0,                      // sensor_size_v float (mm)
        0,                      // resolution_h uint16_t (pix)
        0,                      // resolution_v uint16_t (pix)
        0,                      // lens_id uint8_t
        flags,                  // flags uint32_t (CAMERA_CAP_FLAGS)
        0,                      // cam_definition_version uint16_t
        cam_definition_uri,     // cam_definition_uri char[140]
        _instance + 1);         // gimbal_device_id uint8_t
}

// send camera settings message to GCS
void AP_Mount_Topotek::send_camera_settings(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    const float NaN = nanf("0x4152");

    // send CAMERA_SETTINGS message
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        _recording ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE, // camera mode (0:image, 1:video, 2:image survey)
        NaN,                // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaN);               // focusLevel float, percentage from 0 to 100, NaN if unknown
}

// get rangefinder distance.  Returns true on success
bool AP_Mount_Topotek::get_rangefinder_distance(float& distance_m) const
{
    // if not healthy or zero distance return false
    // healthy() checks attitude timeout which is in same message as rangefinder distance
    if (!healthy() && _measure_dist_m < 0) {
        return false;
    }

    distance_m = _measure_dist_m;
    return true;
}

// enable/disable rangefinder.  Returns true on success
bool AP_Mount_Topotek::set_rangefinder_enable(bool enable)
{
    if (enable) {
        _set_gimbal_range_enable[11] = '3';
    } else {
        _set_gimbal_range_enable[11] = '0';
    }
    return send_packet(_set_gimbal_range_enable, 12);
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Topotek::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(_current_angle_rad.x, _current_angle_rad.y, _current_angle_rad.z);
    return true;
}

// reading incoming packets from gimbal and confirm they are of the correct format
void AP_Mount_Topotek::read_incoming_packets()
{
    // check for bytes on the serial port
    int16_t nbytes = MIN(_uart->available(), 1024U);
    if (nbytes <= 0 ) {
        return;
    }

    // process bytes received
    for (int16_t i = 0; i < nbytes; i++) {
        uint8_t b;
        if (!_uart->read(b)) {
            continue;
        }

        if (_msg_buff_len != 0 || b == '#') {
            if (b == '#' && _msg_buff_len != 0) {
                analyse_packet_data();
            }
            _msg_buff[_msg_buff_len++] = b;
        }
    }
    analyse_packet_data();
}

// request gimbal attitude
void AP_Mount_Topotek::request_gimbal_attitude()
{
    send_packet(_get_gimbal_attitude, 12);
}

// request gimbal memory card information
void AP_Mount_Topotek::request_gimbal_sdcard_info(void)
{
    send_packet(_get_gimbal_sdcard_info, 12);
}

// request gimbal tracking status
void AP_Mount_Topotek::request_track_status(void)
{
    send_packet(_get_gimbal_track_status, 12);
}

// request gimbal basic information
void AP_Mount_Topotek::request_gimbal_basic_info(void)
{
    send_packet(_get_gimbal_basic_info, 12);
}

// set angle target in degrees
void AP_Mount_Topotek::send_angle_target(float roll_rad, float pitch_rad, float yaw_rad, bool yaw_is_ef)
{
    if (!set_gimbal_lock(yaw_is_ef)) {
        return;
    }

    uint8_t speed = AP_MOUNT_TOPOTEK_ROTATION_SPEED;
    int16_t roll_angle_hund = (int16_t)(RAD_TO_DEG*roll_rad*100);
    int16_t pitch_angle_hund = (int16_t)(RAD_TO_DEG*pitch_rad*100);
    int16_t yaw_angle_hund = (int16_t)(RAD_TO_DEG*yaw_rad*100);

    _set_yaw_angle_cmd[10] = hex2char((yaw_angle_hund >> 12) & 0x0F);
    _set_yaw_angle_cmd[11] = hex2char((yaw_angle_hund >> 8) & 0x0F);
    _set_yaw_angle_cmd[12] = hex2char((yaw_angle_hund >> 4) & 0x0F);
    _set_yaw_angle_cmd[13] = hex2char((yaw_angle_hund) & 0x0F);

    _set_yaw_angle_cmd[14] = hex2char((speed >> 4) & 0x0F);
    _set_yaw_angle_cmd[15] = hex2char((speed) & 0x0F);

    if (yaw_is_ef) {
        // control yaw using a gyroscope
        _set_yaw_angle_cmd[8] = 'I';
    } else {
        // control yaw using magnetic encoding
        _set_yaw_angle_cmd[8] = 'A';
    }
    send_packet(_set_yaw_angle_cmd, 16);

    _set_pitch_angle_cmd[10] = hex2char((pitch_angle_hund >> 12) & 0x0F);
    _set_pitch_angle_cmd[11] = hex2char((pitch_angle_hund >> 8) & 0x0F);
    _set_pitch_angle_cmd[12] = hex2char((pitch_angle_hund >> 4) & 0x0F);
    _set_pitch_angle_cmd[13] = hex2char((pitch_angle_hund) & 0x0F);

    _set_pitch_angle_cmd[14] = hex2char((speed >> 4) & 0x0F);
    _set_pitch_angle_cmd[15] = hex2char((speed) & 0x0F);

    send_packet(_set_pitch_angle_cmd, 16);

    _set_roll_angle_cmd[10] = hex2char((roll_angle_hund >> 12) & 0x0F);
    _set_roll_angle_cmd[11] = hex2char((roll_angle_hund >> 8) & 0x0F);
    _set_roll_angle_cmd[12] = hex2char((roll_angle_hund >> 4) & 0x0F);
    _set_roll_angle_cmd[13] = hex2char((roll_angle_hund) & 0x0F);

    _set_roll_angle_cmd[14] = hex2char((speed >> 4) & 0x0F);
    _set_roll_angle_cmd[15] = hex2char((speed) & 0x0F);

    send_packet(_set_roll_angle_cmd, 16);

    return;
}

// sets rate target in deg/s
void AP_Mount_Topotek::send_rate_target(float roll_rads, float pitch_rads, float yaw_rads, bool yaw_is_ef)
{
    if (!set_gimbal_lock(yaw_is_ef)) {
        return;
    }

    // send stop rotation command three times if target roll, pitch and yaw are near zero
    if (roll_rads >= -0.0001 && roll_rads <= 0.0001 && pitch_rads >= -0.0001 && pitch_rads <= 0.0001 && yaw_rads >= -0.0001 && yaw_rads <= 0.0001) {

        if (_stop_order_count < 3) {
            send_packet(_gimbal_control_stop, 12);
            _stop_order_count++;
        }
        return;
    }
    _stop_order_count = 0;

    // keep the speed within a reasonable range.
    uint8_t roll_angle_speed;
    if (roll_rads >= 0) {
        roll_angle_speed = RAD_TO_DEG*roll_rads*ANGULAR_VELOCITY_CONVERSION> 99 ? 99 : (uint8_t)(RAD_TO_DEG*roll_rads*ANGULAR_VELOCITY_CONVERSION);
    } else {
        roll_angle_speed = RAD_TO_DEG*roll_rads*ANGULAR_VELOCITY_CONVERSION < -99 ? -99 : (uint8_t)(RAD_TO_DEG*roll_rads*ANGULAR_VELOCITY_CONVERSION);
    }

    uint8_t pitch_angle_speed;
    if (pitch_rads >= 0) {
        pitch_angle_speed = RAD_TO_DEG*pitch_rads*ANGULAR_VELOCITY_CONVERSION > 99 ? 99 : (uint8_t)(RAD_TO_DEG*pitch_rads*ANGULAR_VELOCITY_CONVERSION);
    } else {
        pitch_angle_speed = RAD_TO_DEG*pitch_rads*ANGULAR_VELOCITY_CONVERSION < -99 ? -99 : (uint8_t)(RAD_TO_DEG*pitch_rads*ANGULAR_VELOCITY_CONVERSION);
    }

    uint8_t yaw_angle_speed;
    if (yaw_rads >= 0) {
        yaw_angle_speed = RAD_TO_DEG*yaw_rads*ANGULAR_VELOCITY_CONVERSION > 99 ? 99 : (uint8_t)(RAD_TO_DEG*yaw_rads*ANGULAR_VELOCITY_CONVERSION);
    } else {
        yaw_angle_speed = RAD_TO_DEG*yaw_rads*ANGULAR_VELOCITY_CONVERSION < -99 ? -99 : (uint8_t)(RAD_TO_DEG*yaw_rads*ANGULAR_VELOCITY_CONVERSION);
    }

    _set_yaw_pitch_roll_speed_cmd[10] = hex2char((yaw_angle_speed >> 4) & 0x0F);
    _set_yaw_pitch_roll_speed_cmd[11] = hex2char((yaw_angle_speed) & 0x0F);

    _set_yaw_pitch_roll_speed_cmd[12] = hex2char((pitch_angle_speed >> 4) & 0x0F);
    _set_yaw_pitch_roll_speed_cmd[13] = hex2char((pitch_angle_speed) & 0x0F);

    _set_yaw_pitch_roll_speed_cmd[14] = hex2char((roll_angle_speed >> 4) & 0x0F);
    _set_yaw_pitch_roll_speed_cmd[15] = hex2char((roll_angle_speed) & 0x0F);

    send_packet(_set_yaw_pitch_roll_speed_cmd, 16);

    return;
}

// send time to gimbal
bool AP_Mount_Topotek::send_time_to_gimbal(uint64_t &utc_time)
{
    time_t now_second = utc_time / 1000000;

    struct tm* now_time = localtime(&now_second);

    sprintf((char*)_set_gimbal_time + 10, "%02d%02d%02d%02d%02d%02d",
            now_time->tm_hour,
            now_time->tm_min,
            now_time->tm_sec,
            now_time->tm_mday,
            now_time->tm_mon + 1,
            now_time->tm_year - 100);
    return send_packet(_set_gimbal_time, 22);
}

// send GPS-related information to the gimbal
bool AP_Mount_Topotek::send_location_info(void)
{
    // get current location
    Location loc;
    int32_t alt_amsl_cm = 0;
    if (!AP::ahrs().get_location(loc) || !loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm)) {
        return false;
    }
    int32_t lat = loc.lat;
    int32_t lng = loc.lng;

    double latitude = lat / 10000000.0;
    double longitude = lng / 10000000.0;

    // ensure it is a positive number
    if (latitude < 0) {
        latitude = -latitude;
    }
    if (longitude < 0) {
        longitude = -longitude;
    }

    // get the degree part of the latitude
    int lat_deg = (int)latitude;

    // get the minute part of the latitude
    double lat_min = (latitude - lat_deg) * 60.0;
    sprintf((char*)_set_gimbal_lat + 10, "%c%02d%07.4f", lat > 0 ? 'N':'S', lat_deg, lat_min);
    send_packet(_set_gimbal_lat, 20);

    // get the degree part of the longitude
    int lng_deg = (int)longitude;

    // get the minute part of the longitude
    double lng_min = (longitude - lng_deg) * 60.0;
    sprintf((char*)_set_gimbal_lng + 10, "%c%03d%07.4f", lng > 0 ? 'E':'W',lng_deg, lng_min);
    send_packet(_set_gimbal_lng, 21);

    // get the height
    float alt_amsl_m = alt_amsl_cm / 100;
    sprintf((char*)_set_gimbal_alt + 10, "%08.1f", alt_amsl_m);
    send_packet(_set_gimbal_alt, 18);

    // get vehicle yaw in the range 0 to 360
    int32_t veh_yaw_deg = wrap_360_cd(degrees(AP::ahrs().get_yaw() * 100));
    float veh_yaw = veh_yaw_deg / 100.0f;
    sprintf((char*)_set_gimbal_azi + 10, "%05.1f", veh_yaw);
    send_packet(_set_gimbal_azi, 15);

    return true;
}

// analyze the data information received
void AP_Mount_Topotek::analyse_packet_data(void)
{
    // computed check bit
    int8_t crc = calculate_crc(_msg_buff, _msg_buff_len - 2);

    if (hex2char((crc >> 4) & 0x0f) == _msg_buff[_msg_buff_len-2] && hex2char((crc) & 0x0f) == _msg_buff[_msg_buff_len-1]) {
        for (uint8_t count = 0; count < AP_MOUNT_RECV_GIMBAL_CMD_CATEGORIES_NUM; count++) {
            // compare the commands received from the gimbal and perform the related operations
            if (0 == strncmp((const char*)_msg_buff + 7, (const char*)(uart_recv_cmd_compare_list[count].uart_cmd_key), 3)) {
                (this->*(uart_recv_cmd_compare_list[count].func))();
                break;
            }
        }
    }

    // clear the accept cache
    memset(_msg_buff, 0, AP_MOUNT_TOPOTEK_PACKETLEN_MAX);
    _msg_buff_len = 0;
}


// attitude information analysis of gimbal
void AP_Mount_Topotek::gimbal_angle_analyse(void)
{
    _last_current_angle_ms = AP_HAL::millis();

    // angle[0] represents the gimbal yaw angle multiplied by 100
    // angle[1] represents the gimbal pitch angle multiplied by 100
    // angle[2] represents the gimbal roll angle multiplied by 100
    int16_t angle[3];

    angle[0] = char_to_number(_msg_buff[10])<<12 | char_to_number(_msg_buff[11])<<8 | char_to_number(_msg_buff[12])<<4 | char_to_number(_msg_buff[13]);
    angle[1] = char_to_number(_msg_buff[14])<<12 | char_to_number(_msg_buff[15])<<8 | char_to_number(_msg_buff[16])<<4 | char_to_number(_msg_buff[17]);
    angle[2] = char_to_number(_msg_buff[18])<<12 | char_to_number(_msg_buff[19])<<8 | char_to_number(_msg_buff[20])<<4 | char_to_number(_msg_buff[21]);

    // handle when the yaw angle is too large
    angle[0] = wrap_180_cd(angle[0]);

    _current_angle_rad.x = (float)angle[2] / 100 * DEG_TO_RAD;
    _current_angle_rad.y = (float)angle[1] / 100 * DEG_TO_RAD;
    _current_angle_rad.z = (float)angle[0] / 100 * DEG_TO_RAD;

    return;
}

// gimbal video information analysis
void AP_Mount_Topotek::gimbal_record_analyse(void)
{
    if (_msg_buff[10] == '1' || _msg_buff[11] == '1') {
        _recording = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s recording ON", send_message_prefix);
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s recording OFF", send_message_prefix);

    _recording = false;
    return;
}

// information analysis of gimbal storage card
void AP_Mount_Topotek::gimbal_sdcard_analyse(void)
{
    if (('N' == _msg_buff[10]) && ('N' == _msg_buff[11]) && ('N' == _msg_buff[12]) && ('N' == _msg_buff[13])) {
        // memory card exception
        _sdcard_status = false;
        return;
    }
    _sdcard_status = true;

#if AP_RTC_ENABLED
    // send UTC time to the camera
     if (_sent_time_count < 7) {
        uint64_t utc_usec ;
        if (AP::rtc().get_utc_usec(utc_usec) && send_time_to_gimbal(utc_usec)) {
            _sent_time_count++;
        }
    }
#endif

    return;
}

// gimbal tracking information analysis
void AP_Mount_Topotek::gimbal_track_analyse(void)
{
    switch (_msg_buff[11]) {
    case '0':
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s trace exception", send_message_prefix);
        break;
    case '1':
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s stop tracking", send_message_prefix);
        break;
    case '2':
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s be tracking", send_message_prefix);
        break;
    }
}

// gimbal distance information analysis
void AP_Mount_Topotek::gimbal_dist_info_analyse(void)
{
    if ('E' == _msg_buff[10] && 'R'== _msg_buff[11] && 'R'==_msg_buff[12]) {
        _measure_dist_m = -1.0f;
        return;
    }
    long lrf = (char_to_number(_msg_buff[10])*100000 +
                char_to_number(_msg_buff[11])*10000 +
                char_to_number(_msg_buff[12])*1000 +
                char_to_number(_msg_buff[13])*100 +
                char_to_number(_msg_buff[14])*10 +
                char_to_number(_msg_buff[16]));

    _measure_dist_m = lrf/10 + lrf%10 * 0.1;
}

// gimbal basic information analysis
void AP_Mount_Topotek::gimbal_basic_info_analyse(void)
{
    uint8_t count;
    uint8_t major_ver = 0;
    uint8_t minor_ver = 0;
    uint8_t patch_ver = 0;

    // analyze gimbal model and firmware version
    for (count = 0; count < char_to_number(_msg_buff[5]); count++) {
        if (count == 0) {
            major_ver = char_to_number(_msg_buff[10 + count]);
        } else if (count == 1) {
            minor_ver = char_to_number(_msg_buff[10 + count]);
        } else if (count == 2) {
            patch_ver = char_to_number(_msg_buff[10 + count]);
        } else {
            strncpy(( char*)_model_name, (const char*)(_msg_buff+13), 10);
            break;
        }
    }
    _firmware_ver = (patch_ver << 16) | (minor_ver << 8) | (major_ver);

    if (strlen((const char*)_model_name) == 0) {
        strcpy((char*)_model_name, "Unknown");
    }

    _got_gimbal_basic_info = true;
}

// add check
void AP_Mount_Topotek::add_check(int8_t *cmd, uint8_t len)
{
    int8_t crc = 0;
    crc = calculate_crc(cmd, len);
    cmd[len] = hex2char((crc >> 4) & 0x0f);
    cmd[len+1] = hex2char(crc & 0x0f);
}

// calculate checksum
int8_t AP_Mount_Topotek::calculate_crc(int8_t *cmd, uint8_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i<len; i++) {
        crc += cmd[i];
    }
    return(crc);
}

// hexadecimal to character conversion
int8_t AP_Mount_Topotek::hex2char(int8_t data)
{
    if ((9 >= data)) {
        return (data + '0');
    } else {
        return (data - 10 + 'A');
    }
}

// character to number conversion
uint8_t AP_Mount_Topotek::char_to_number(int8_t data)
{
    if (('0' <= data && '9' >= data)) {
        return (data - '0');
    }
    else if (('a' <= data) && ('z' >= data)) {
        return (data - 'a' + 10);
    }
    else if (('A' <= data) && ('Z' >= data)) {
        return (data - 'A' + 10);
    }
    return 0;
}

// send packet to gimbal.  databuff includes everything after the length-and-frame-counter, does not include crc
// returns true on success, false if serial port initialization failed
bool AP_Mount_Topotek::send_packet(int8_t* data_packet, uint8_t data_packet_len)
{
    if (!_initialised) {
        return false;
    }

    // add check
    add_check(data_packet, data_packet_len);

    return _uart->write((const uint8_t*)data_packet, data_packet_len + 2);
}

// set gimbal's lock vs follow mode
// lock should be true if gimbal should maintain an earth-frame target
// lock is false to follow / maintain a body-frame target
bool AP_Mount_Topotek::set_gimbal_lock(bool lock)
{
    if (_last_lock == lock) {
        return true;
    }
    if (lock) {
        // send lock command
        _gimbal_lock[11] = '6';
        if (send_packet(_gimbal_lock, 12)) {
            _last_lock = lock;
            return true;
        }
    } else {
        // send follow command
        _gimbal_lock[11] = '7';
        if (send_packet(_gimbal_lock, 12)) {
            _last_lock = lock;
            return true;
        }
    }
    return false;
}

#endif // HAL_MOUNT_TOPOTEK_ENABLED
