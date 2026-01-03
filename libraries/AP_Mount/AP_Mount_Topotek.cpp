#include "AP_Mount_config.h"

#if HAL_MOUNT_TOPOTEK_ENABLED

#include "AP_Mount_Topotek.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RTC/AP_RTC.h>

extern const AP_HAL::HAL& hal;

#define ANGULAR_VELOCITY_CONVERSION         1.220740379         // gimbal angular velocity conversion ratio
#define TRACK_TOTAL_WIDTH                   1920                // track the maximum width of the image range
#define TRACK_TOTAL_HEIGHT                  1080                // track the maximum height of the image range
#define TRACK_RANGE                         60                  // the size of the image at point tracking
#define AP_MOUNT_TOPOTEK_UPDATE_INTERVAL_MS 100                 // resend angle or rate targets to gimbal at this interval
#define AP_MOUNT_TOPOTEK_HEALTH_TIMEOUT_MS  1000                // timeout for health and rangefinder readings
#define AP_MOUNT_TOPOTEK_PACKETLEN_MIN      12                  // packet length not including the data segment
#define AP_MOUNT_TOPOTEK_DATALEN_MAX        (AP_MOUNT_TOPOTEK_PACKETLEN_MAX - AP_MOUNT_TOPOTEK_PACKETLEN_MIN) // data segment lens can be no more tha this

// 3 character identifiers
# define AP_MOUNT_TOPOTEK_ID3CHAR_CAPTURE       "CAP"           // take picture, data bytes: 01:RGB + thermal, 02:RGB, 03:thermal, 05:RGB + thermal (with temp measurement)
# define AP_MOUNT_TOPOTEK_ID3CHAR_RECORD_VIDEO  "REC"           // record video, data bytes: 00:stop, 01:start, 0A:toggle start/stop
# define AP_MOUNT_TOPOTEK_ID3CHAR_CONTROL_ZOOM  "ZMC"           // control zoom, data bytes: 00:stop, 01:zoom out, 02:zoom in
# define AP_MOUNT_TOPOTEK_ID3CHAR_GET_ZOOM      "ZOM"           // get zoom, no data bytes
# define AP_MOUNT_TOPOTEK_ID3CHAR_CONTROL_FOCUS "FCC"           // control focus, data bytes: 00:stop, 01:focus+, 02:focus-, 0x10:auto focus, 0x11:manual focus, 0x12:manu focus (save), 0x13:auto focus (save)
# define AP_MOUNT_TOPOTEK_ID3CHAR_GET_FOCUS     "FOC"           // get focus, no data bytes
# define AP_MOUNT_TOPOTEK_ID3CHAR_SET_ZOOM_AND_FOCU "ZFP"       // set zoom and focus
# define AP_MOUNT_TOPOTEK_ID3CHAR_DAY_NIGHT_SWITCHING "IRC"     // set day/night setting, data bytes: 00:day, 01:night, 0A:toggle state
# define AP_MOUNT_TOPOTEK_ID3CHAR_TRACKING      "TRC"           // get/set image tracking, data bytes: 00:get status (use with "r"), 01:stop (use with "w")
# define AP_MOUNT_TOPOTEK_ID3CHAR_START_TRACKING "LOC"          // start image tracking
# define AP_MOUNT_TOPOTEK_ID3CHAR_LRF           "LRF"           // laser rangefinder control, data bytes: 00:ranging stop, 01:ranging start, 02:single measurement, 03:continuous measurement
# define AP_MOUNT_TOPOTEK_ID3CHAR_PIP           "PIP"           // set picture-in-picture setting, data bytes: // 00:main only, 01:main+sub, 02:sub+main, 03:sub only, 0A:next
# define AP_MOUNT_TOPOTEK_ID3CHAR_GIMBAL_ATT    "GIA"           // get gimbal attitude, data bytes: 00:stop, 01:start
# define AP_MOUNT_TOPOTEK_ID3CHAR_SD_CARD       "SDC"           // get SD card state, data bytes: 00:get remaining capacity, 01:get total capacity
# define AP_MOUNT_TOPOTEK_ID3CHAR_TIME          "UTC"           // set time and date, data bytes: HHMMSSDDMMYY
# define AP_MOUNT_TOPOTEK_ID3CHAR_GET_VERSION   "VSN"           // get firmware version, data bytes always 00
# define AP_MOUNT_TOPOTEK_ID3CHAR_GET_MODEL_NAME "PA2"          // get model name, data bytes always 00
# define AP_MOUNT_TOPOTEK_ID3CHAR_GIMBAL_MODE   "PTZ"           // set gimbal mode, data bytes: 00:stop, 01:up, 02:down, 03:left, 04:right, 05:home position, 06:lock, 07:follow, 08:lock/follow toggle, 09:calibration, 0A:one button down
# define AP_MOUNT_TOPOTEK_ID3CHAR_YPR_RATE      "YPR"           // set the rate yaw, pitch and roll targets of the gimbal yaw in range -99 ~ +99
# define AP_MOUNT_TOPOTEK_ID3CHAR_YAW_ANGLE     "GIY"           // set the yaw angle target in the range -150 ~ 150, speed 0 ~ 99 (0.1deg/sec)
# define AP_MOUNT_TOPOTEK_ID3CHAR_YAW_ANGLE_BF  "GAY"           // set the yaw angle target in body-frame in the range -150 ~ 150, speed 0 ~ 99 (0.1deg/sec)
# define AP_MOUNT_TOPOTEK_ID3CHAR_PITCH_ANGLE   "GIP"           // set the pitch angle target in the range -90 ~ 90, speed 0 ~ 99 (0.1deg/sec)
# define AP_MOUNT_TOPOTEK_ID3CHAR_ROLL_ANGLE    "GIR"           // set the roll angle target in the range -90 ~ 90, speed 0 ~ 99 (0.1deg/sec)
# define AP_MOUNT_TOPOTEK_ID3CHAR_SET_LAT       "LAT"           // set the gimbal's latitude
# define AP_MOUNT_TOPOTEK_ID3CHAR_SET_LON       "LON"           // set the gimbal's longitude
# define AP_MOUNT_TOPOTEK_ID3CHAR_SET_ALT       "ALT"           // set the gimbal's altitude
# define AP_MOUNT_TOPOTEK_ID3CHAR_SET_AZIMUTH   "AZI"           // set the gimbal's yaw (aka azimuth)

#define AP_MOUNT_TOPOTEK_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_TOPOTEK_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Topotek: " fmt, ## args); } } while (0)

const char* AP_Mount_Topotek::send_message_prefix = "Mount: Topotek";

// update mount position - should be called periodically
void AP_Mount_Topotek::update()
{
    AP_Mount_Backend::update();

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // reading incoming packets from gimbal
    read_incoming_packets();

    // everything below updates at 10hz
    uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_req_current_info_ms) < 100) {
        return;
    }
    _last_req_current_info_ms = now_ms;

    // re-send the stop zoom command a second time to prevent data transmission errors.
    if (_last_zoom_stop) {
        _last_zoom_stop = false;
        send_fixedlen_packet(AddressByte::LENS, AP_MOUNT_TOPOTEK_ID3CHAR_CONTROL_ZOOM, true, 0);
    }

    // re-send the stop focus command a second time to prevent data transmission errors.
    if (_last_focus_stop) {
        _last_focus_stop = false;
        send_fixedlen_packet(AddressByte::LENS, AP_MOUNT_TOPOTEK_ID3CHAR_CONTROL_FOCUS, true, 0);
    }

    // send GPS-related information to the gimbal
    send_location_info();

    // calls below here called at 1hz
    _last_req_step++;
    if (_last_req_step >= 10) {
        _last_req_step = 0;
    }
    switch (_last_req_step) {
    case 0:
        // get gimbal version
        if (!_got_gimbal_version) {
            request_gimbal_version();
        }
        break;
    case 2:
        // request gimbal attitude at 1hz
        // gimbal will continue to send attitude information during the next period
        request_gimbal_attitude();
        break;
    case 4:
        // request memory card information
        request_gimbal_sdcard_info();
        break;
    case 6:
        // request tracking info
        request_track_status();
        break;
    case 8:
        // get gimbal model name
        if (!_got_gimbal_model_name) {
            request_gimbal_model_name();
        }
        break;
    }

    // handle tracking state
    if (_is_tracking) {
        // cancel tracking if mode has changed
        if (_last_mode != _mode) {
            _last_mode = _mode;
            cancel_tracking();
        }
        return;
    }
    _last_mode = _mode;

    // update based on mount mode
    update_mnt_target();

    // send target angles or rates depending on the target type
    send_target_to_gimbal();
}

// return true if healthy
bool AP_Mount_Topotek::healthy() const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // unhealthy if attitude information not received recently
    const uint32_t last_current_angle_ms = _last_current_angle_ms;
    return (AP_HAL::millis() - last_current_angle_ms < AP_MOUNT_TOPOTEK_HEALTH_TIMEOUT_MS);
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
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s SD card error", send_message_prefix);
        return false;
    }

    // sample command: #TPUD2wCAP01
    return send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_CAPTURE, true, 1);
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_Topotek::record_video(bool start_recording)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // exit immediately if the memory card is abnormal
    if (!_sdcard_status) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s SD card error", send_message_prefix);
        return false;
    }

    // sample command: #TPUD2wREC01
    return send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_RECORD_VIDEO, true, start_recording ? 1 : 0);
}

// set zoom specified as a rate
bool AP_Mount_Topotek::set_zoom(ZoomType zoom_type, float zoom_value)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // zoom rate
    if (zoom_type == ZoomType::RATE) {
        uint8_t zoom_cmd;
        if (is_zero(zoom_value)) {
            // stop zoom
            zoom_cmd = 0;
            _last_zoom_stop = true;
        } else if (zoom_value < 0) {
            // zoom out
            zoom_cmd = 1;
        } else {
            // zoom in
            zoom_cmd = 2;
        }
        // sample command: #TPUM2wZMC00
        return send_fixedlen_packet(AddressByte::LENS, AP_MOUNT_TOPOTEK_ID3CHAR_CONTROL_ZOOM, true, zoom_cmd);
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
        uint8_t focus_cmd;
        if (is_zero(focus_value)) {
            focus_cmd = 0;
            _last_focus_stop = true;
        } else if (focus_value < 0) {
            // focus-
            focus_cmd = 2;
        } else {
            // focus+
            focus_cmd = 1;
        }
        // send focus command and switch to manual focus
        // sample command: #TPUM2wFCC00
        if (send_fixedlen_packet(AddressByte::LENS, AP_MOUNT_TOPOTEK_ID3CHAR_CONTROL_FOCUS, true, focus_cmd) &&
            send_fixedlen_packet(AddressByte::LENS, AP_MOUNT_TOPOTEK_ID3CHAR_CONTROL_FOCUS, true, 0x11)) {
                return SetFocusResult::ACCEPTED;
        }
        return SetFocusResult::FAILED;
    }
    case FocusType::PCT:
        // not supported
        return SetFocusResult::INVALID_PARAMETERS;
    case FocusType::AUTO:
        // auto focus
        if (send_fixedlen_packet(AddressByte::LENS, AP_MOUNT_TOPOTEK_ID3CHAR_CONTROL_FOCUS, true, 0x10)) {
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

    // local variables holding tracker center and width
    int16_t track_center_x, track_center_y, track_width, track_height;
    bool send_tracking_cmd = false;

    switch (tracking_type) {

    case TrackingType::TRK_NONE:
        return cancel_tracking();

    case TrackingType::TRK_POINT: {
        // calculate tracking center, width and height
        track_center_x = (int16_t)((p1.x*TRACK_TOTAL_WIDTH - 960) /  0.96);
        track_center_y = (int16_t)((p1.y*TRACK_TOTAL_HEIGHT - 540) /  0.54);
        track_width = (int16_t)(TRACK_RANGE / 0.96);
        track_height = (int16_t)(TRACK_RANGE / 0.54);
        send_tracking_cmd = true;
        break;
    }

    case TrackingType::TRK_RECTANGLE:
        // calculate upper left and bottom right points
        // handle case where p1 and p2 are in an unexpected order
        int16_t upper_leftx = (int16_t)(MIN(p1.x, p2.x)*TRACK_TOTAL_WIDTH);
        int16_t upper_lefty = (int16_t)(MIN(p1.y, p2.y)*TRACK_TOTAL_HEIGHT);
        int16_t bottom_rightx = (int16_t)(MAX(p1.x, p2.x)*TRACK_TOTAL_WIDTH);
        int16_t bottom_righty = (int16_t)(MAX(p1.y, p2.y)*TRACK_TOTAL_HEIGHT);

        // calculated width and height and sanity check 
        const int16_t frame_selection_width = bottom_rightx - upper_leftx;
        const int16_t frame_selection_height = bottom_righty - upper_lefty;
        if (frame_selection_width <= 0 || frame_selection_height <= 0) {
            return false;
        }

        // calculate tracking center
        track_center_x = (int16_t)((((upper_leftx + bottom_rightx) * 0.5) - 960) / 0.96);
        track_center_y = (int16_t)((((upper_lefty + bottom_righty) * 0.5) - 540) / 0.54);

        // tracking range after conversion
        track_width = (int16_t)(frame_selection_width / 0.96);
        track_height = (int16_t)(frame_selection_height / 0.54);

        send_tracking_cmd = true;
        break;
    }

    if (send_tracking_cmd) {
        // set the gimbal to the ready-to-track state when the gimbal tracking status is stopped
        if (_last_tracking_state == TrackingStatus::STOPPED_TRACKING) {
            send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_TRACKING, true, 2);
        }

        // prepare data bytes
        uint8_t databuff[10];
        databuff[0] = HIGHBYTE(track_center_x);
        databuff[1] = LOWBYTE(track_center_x);
        databuff[2] = HIGHBYTE(track_center_y);
        databuff[3] = LOWBYTE(track_center_y);
        databuff[4] = HIGHBYTE(track_width);
        databuff[5] = LOWBYTE(track_width);
        databuff[6] = HIGHBYTE(track_height);
        databuff[7] = LOWBYTE(track_height);
        databuff[8] = 0;
        databuff[9] = (tracking_type == TrackingType::TRK_POINT) ? 9 : 1;   // when tracking point, enable fuzzy click function

        // send tracking command
        bool res = send_variablelen_packet(HeaderType::VARIABLE_LEN,
                                           AddressByte::SYSTEM_AND_IMAGE,
                                           AP_MOUNT_TOPOTEK_ID3CHAR_START_TRACKING,
                                           true,
                                           (uint8_t*)databuff, ARRAY_SIZE(databuff));

        // display error message on failure
        if (!res) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s tracking failed", send_message_prefix);
        }

        return res;
    }

    // should never reach here
    return false;
}

// send command to gimbal to cancel tracking (if necessary)
// returns true on success, false on failure to send message
bool AP_Mount_Topotek::cancel_tracking()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // if gimbal is tracking-in-progress change to waiting state, otherwise stop
    const uint8_t track_set = _last_tracking_state == TrackingStatus::TRACKING_IN_PROGRESS ? 1 : 0;

    // send tracking command
    return send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_TRACKING, true, track_set);
}

// set camera picture-in-picture mode
bool AP_Mount_Topotek::set_lens(uint8_t lens)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // sanity check lens number
    // 00:main only, 01:main+sub, 02:sub+main, 03:sub only, 0A:next
    // sample command: #TPUD2wPIP0A
    if (lens > 3) {
        return false;
    }

    // send pip command
    return send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_PIP, true, lens);
}

#if HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED
// set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
// primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
bool AP_Mount_Topotek::set_camera_source(uint8_t primary_source, uint8_t secondary_source)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // maps primary and secondary source to pip setting
    // pip settings 00:main only, 01:main+sub, 02:sub+main, 03:sub only, 0A:next
    // sample command: #TPUD2wPIP0A
    uint8_t pip_setting = 0;
    switch (primary_source) {
    case 0: // Default (RGB)
        FALLTHROUGH;
    case 1: // RGB
        switch (secondary_source) {
        case 0: // RGB + Default (None)
            pip_setting = 0;    // main only
            break;
        case 2: // PIP RGB+IR
            pip_setting = 1;    // main+sub
            break;
        default:
            return false;
        }
        break;
    case 2: // IR
        switch (secondary_source) {
        case 0: // IR + Default (None)
            pip_setting = 3;    // sub only
            break;
        case 1: // IR+RGB
            pip_setting = 2;    // sub+main
            break;
        default:
            return false;
        }
        break;
    default:
        return false;
    }

    // send pip command
    return send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_PIP, true, pip_setting);
}
#endif  // HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED

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

    // copy model name if available
    if (_got_gimbal_model_name) {
        strncpy((char*)model_name, (const char*)_model_name, ARRAY_SIZE(model_name));
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
        NaNf,                   // sensor_size_h float (mm)
        NaNf,                   // sensor_size_v float (mm)
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

    // send CAMERA_SETTINGS message
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        _recording ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE, // camera mode (0:image, 1:video, 2:image survey)
        NaNf,               // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaNf);              // focusLevel float, percentage from 0 to 100, NaN if unknown
}

// get rangefinder distance.  Returns true on success
bool AP_Mount_Topotek::get_rangefinder_distance(float& distance_m) const
{
    // if not healthy or negative distance return false
    // healthy() checks attitude timeout which is in same message as rangefinder distance
    if (!healthy() || (_measure_dist_m < 0)) {
        return false;
    }

    distance_m = _measure_dist_m;
    return true;
}

// enable/disable rangefinder.  Returns true on success
bool AP_Mount_Topotek::set_rangefinder_enable(bool enable)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // 00:ranging stop, 01:ranging start, 02:single measurement, 03:continuous measurement
    // sample command: #TPUM2wLRF00
    return send_fixedlen_packet(AddressByte::LENS, AP_MOUNT_TOPOTEK_ID3CHAR_LRF, true, enable ? 3 : 0);
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

    // flag to allow cases below to reset parser state
    bool reset_parser = false;

    // process bytes received
    for (int16_t i = 0; i < nbytes; i++) {
        uint8_t b;
        if (!_uart->read(b)) {
            continue;
        }

        // add latest byte to buffer
        _msg_buff[_msg_buff_len++] = b;

        // protect against overly long messages
        if (_msg_buff_len >= AP_MOUNT_TOPOTEK_PACKETLEN_MAX) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_parser.state) {

        case ParseState::WAITING_FOR_HEADER1:
            if (b == '#') {
                _parser.state = ParseState::WAITING_FOR_HEADER2;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_HEADER2:
            if (b == 't' || b == 'T') {
                _parser.state = ParseState::WAITING_FOR_HEADER3;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_HEADER3:
            if (b == 'p' || b == 'P') {
                _parser.state = ParseState::WAITING_FOR_ADDR1;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_ADDR1:
        case ParseState::WAITING_FOR_ADDR2:
            if (b == 'U' || b =='M' || b == 'D' || b =='E' || b =='P' || b =='G') {
                // advance to next state
                _parser.state = (ParseState)((uint8_t)_parser.state+1);
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_DATALEN:
            // sanity check data length
            _parser.data_len = (uint8_t)char_to_hex(b);
            if (_parser.data_len <= AP_MOUNT_TOPOTEK_DATALEN_MAX) {
                _parser.state = ParseState::WAITING_FOR_CONTROL;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_CONTROL:
            // r or w
            if (b == 'r' || b == 'w') {
                _parser.state = ParseState::WAITING_FOR_ID1;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_ID1:
        case ParseState::WAITING_FOR_ID2:
        case ParseState::WAITING_FOR_ID3:
            // check all uppercase letters and numbers.  eg 'GAC'
            if ((b >= 'A' && b <= 'Z') || (b >= '0' && b <= '9')) {
                // advance to next state
                _parser.state = (ParseState)((uint8_t)_parser.state+1);
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_DATA: {
            // normally hex numbers in char form (e.g. '0A')
            const uint8_t data_bytes_received = _msg_buff_len - (AP_MOUNT_TOPOTEK_PACKETLEN_MIN - 2);

            // sanity check to protect against programming errors
            if (data_bytes_received > AP_MOUNT_TOPOTEK_DATALEN_MAX) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                reset_parser = true;
                break;
            }
            
            // advance parser state once expected number of bytes have been received
            if (data_bytes_received == _parser.data_len) {
                _parser.state = ParseState::WAITING_FOR_CRC_LOW;
            }
            break;
        }

        case ParseState::WAITING_FOR_CRC_LOW:
            _parser.state = ParseState::WAITING_FOR_CRC_HIGH;
            break;

        case ParseState::WAITING_FOR_CRC_HIGH:
            // this is the last byte in the message so reset the parser
            reset_parser = true;

            // sanity check to protect against programming errors
            if (_msg_buff_len < AP_MOUNT_TOPOTEK_PACKETLEN_MIN) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                break;
            }

            // calculate and check CRC
            const uint8_t crc_value = calculate_crc(_msg_buff, _msg_buff_len - 2);
            const char crc_char1 = hex2char((crc_value >> 4) & 0x0f);
            const char crc_char2 = hex2char((crc_value) & 0x0f);
            if (crc_char1 != _msg_buff[_msg_buff_len - 2] || crc_char2 != _msg_buff[_msg_buff_len-1]) {
                debug("CRC expected:%x got:%c%c", (int)crc_value, crc_char1, crc_char2);
                break;
            }

            // CRC is OK, call function to process the message
            for (uint8_t count = 0; count < AP_MOUNT_RECV_GIMBAL_CMD_CATEGORIES_NUM; count++) {
                if (strncmp((const char*)_msg_buff + 7, (const char*)(uart_recv_cmd_compare_list[count].uart_cmd_key), 3) == 0) {
                    (this->*(uart_recv_cmd_compare_list[count].func))();
                    break;
                }
            }
        }

        // handle reset of parser
        if (reset_parser) {
            _parser.state = ParseState::WAITING_FOR_HEADER1;
            _msg_buff_len = 0;
            reset_parser = false;
        }
    }
}

// request gimbal attitude
void AP_Mount_Topotek::request_gimbal_attitude()
{
    // sample command: #TPUG2wGIA01
    send_fixedlen_packet(AddressByte::GIMBAL, AP_MOUNT_TOPOTEK_ID3CHAR_GIMBAL_ATT, true, 1);
}

// request gimbal memory card information
void AP_Mount_Topotek::request_gimbal_sdcard_info()
{
    // request remaining capacity
    // sample command including CRC: #TPUD2rSDC003E
    // 00:get remaining capacity, 01:get total capacity
    send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_SD_CARD, false, 0);
}

// request gimbal tracking status
void AP_Mount_Topotek::request_track_status()
{
    // 00:get status (use with "r"), 01:stop (use with "w")
    // sample command: #TPUD2rTRC00
    send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_TRACKING, false, 0);
}

// request gimbal version
void AP_Mount_Topotek::request_gimbal_version()
{
    // sample command: #TPUD2rVSN00
    send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_GET_VERSION, false, 0);
}

// request gimbal model name
void AP_Mount_Topotek::request_gimbal_model_name()
{
    // sample command: #TPUG2rPA200
    send_fixedlen_packet(AddressByte::GIMBAL, AP_MOUNT_TOPOTEK_ID3CHAR_GET_MODEL_NAME, false, 0);
}

// send angle target in radians to gimbal
void AP_Mount_Topotek::send_target_angles(const MountAngleTarget& angle_rad)
{
    // gimbal's earth-frame angle control drifts so always use body frame
    // set gimbal's lock state if it has changed
    if (!set_gimbal_lock(false)) {
        return;
    }

    // calculate and send yaw target
    // sample command #tpUG6wGIY
    const char* format_str = "%04X%02X";
    const uint8_t speed = 99;
    const uint16_t yaw_angle_cd = (uint16_t)constrain_int16(degrees(angle_rad.get_bf_yaw()) * 100, MAX(-18000, _params.yaw_angle_min * 100), MIN(18000, _params.yaw_angle_max * 100));

    uint8_t databuff[7];
    hal.util->snprintf((char *)databuff, ARRAY_SIZE(databuff), format_str, yaw_angle_cd, speed);
    send_variablelen_packet(HeaderType::VARIABLE_LEN,
                            AddressByte::GIMBAL,
                            AP_MOUNT_TOPOTEK_ID3CHAR_YAW_ANGLE_BF,
                            true,
                            (uint8_t*)databuff, ARRAY_SIZE(databuff)-1);

    // send pitch target
    // sample command: #tpUG6wGIP
    const uint16_t pitch_angle_cd = (uint16_t)constrain_int16(-degrees(angle_rad.pitch) * 100, -9000, 9000);
    hal.util->snprintf((char *)databuff, ARRAY_SIZE(databuff), format_str, pitch_angle_cd, speed);
    send_variablelen_packet(HeaderType::VARIABLE_LEN,
                            AddressByte::GIMBAL,
                            AP_MOUNT_TOPOTEK_ID3CHAR_PITCH_ANGLE,
                            true,
                            (uint8_t*)databuff, ARRAY_SIZE(databuff)-1);

    // send roll target
    // sample command: #tpUG6wGIR
    const uint16_t roll_angle_cd = (uint16_t)constrain_int16(degrees(angle_rad.roll) * 100, -18000, 18000);
    hal.util->snprintf((char *)databuff, ARRAY_SIZE(databuff), format_str, roll_angle_cd, speed);
    send_variablelen_packet(HeaderType::VARIABLE_LEN,
                            AddressByte::GIMBAL,
                            AP_MOUNT_TOPOTEK_ID3CHAR_ROLL_ANGLE,
                            true,
                            (uint8_t*)databuff, ARRAY_SIZE(databuff)-1);
}

// send rate target in rad/s to gimbal
void AP_Mount_Topotek::send_target_rates(const MountRateTarget& rate_rads)
{
    // set gimbal's lock state if it has changed
    if (!set_gimbal_lock(rate_rads.yaw_is_ef)) {
        return;
    }

    // convert and constrain rates
    const uint8_t roll_angle_speed = constrain_int16(degrees(rate_rads.roll) * ANGULAR_VELOCITY_CONVERSION, -99, 99);
    const uint8_t pitch_angle_speed = constrain_int16(-degrees(rate_rads.pitch) * ANGULAR_VELOCITY_CONVERSION, -99, 99);
    const uint8_t yaw_angle_speed = constrain_int16(degrees(rate_rads.yaw) * ANGULAR_VELOCITY_CONVERSION, -99, 99);

    // send stop rotation command three times if target roll, pitch and yaw are zero
    if (roll_angle_speed == 0 && pitch_angle_speed == 0 && yaw_angle_speed == 0) {
        if (_stop_order_count < 3) {
            // sample command: #TPUG2wPTZ00
            if (send_fixedlen_packet(AddressByte::GIMBAL, AP_MOUNT_TOPOTEK_ID3CHAR_GIMBAL_MODE, true, 0)) {
                _stop_order_count++;
            }
        }
        return;
    }
    _stop_order_count = 0;

    // prepare and send command
    // sample command: #tpUG6wYPR
    uint8_t databuff[7];
    hal.util->snprintf((char *)databuff, ARRAY_SIZE(databuff), "%02X%02X%02X", yaw_angle_speed, pitch_angle_speed, roll_angle_speed);
    send_variablelen_packet(HeaderType::VARIABLE_LEN, AddressByte::GIMBAL, AP_MOUNT_TOPOTEK_ID3CHAR_YPR_RATE, true, databuff, ARRAY_SIZE(databuff)-1);
}

// send time and date to gimbal
bool AP_Mount_Topotek::send_time_to_gimbal()
{
#if AP_RTC_ENABLED
    // get date and time
    // year is the regular Gregorian year, month is 0~11, day is 1~31, hour is 0~23, minute is 0~59, second is 0~60 (1 leap second), ms is 0~999
    uint16_t year, ms;
    uint8_t month, day, hour, min, sec;
    if (!AP::rtc().get_date_and_time_utc(year, month, day, hour, min, sec, ms)) {
        return false;
    }

    // sample command: #tpUDCwUTCHHMMSSDDMMYY
    uint8_t databuff[13];
    hal.util->snprintf((char*)databuff, ARRAY_SIZE(databuff), "%02d%02d%02d%02d%02d%02d", hour, min, sec, day, month + 1, year - 2000);
    return send_variablelen_packet(HeaderType::VARIABLE_LEN, AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_TIME, true, (uint8_t*)databuff, ARRAY_SIZE(databuff)-1);
#else
    return false;
#endif
}

// send GPS-related information to the gimbal
bool AP_Mount_Topotek::send_location_info()
{
    // get current location
    Location loc;
    int32_t alt_amsl_cm = 0;
    if (!AP::ahrs().get_location(loc) || !loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm)) {
        return false;
    }

    // convert latitude and longitude to positive angles in degrees
    const double latitude = labs(loc.lat) * 1e-7;
    const double longitude = labs(loc.lng) * 1e-7;

    // get the degree part
    const int16_t lat_deg = (int16_t)latitude;
    const int16_t lng_deg = (int16_t)longitude;

    // get the minute part
    const double lat_min = (latitude - lat_deg) * 60.0;
    const double lng_min = (longitude - lng_deg) * 60.0;

    // prepare and send latitude
    // first byte is N or S, followed by GPS coordinates in degree division format, in the format of ddmm.mmmm
    // first byte is zero and will also be transmitted.  same as the data format in $GPGGA
    // sample command: #tpUDAwLATNddmm.mmmm
    uint8_t databuff_lat[11];
    hal.util->snprintf((char*)databuff_lat, ARRAY_SIZE(databuff_lat), "%c%02d%07.4f", loc.lat > 0 ? 'N':'S', lat_deg, lat_min);
    if (!send_variablelen_packet(HeaderType::VARIABLE_LEN, AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_SET_LAT, true, (uint8_t*)databuff_lat, ARRAY_SIZE(databuff_lat)-1)) {
        return false;
    }

    // prepare and send longitude
    // sample command: #tpUDBwLONEdddmm.mmmm
    uint8_t databuff_lon[12];
    hal.util->snprintf((char*)databuff_lon, ARRAY_SIZE(databuff_lon), "%c%03d%07.4f", loc.lng > 0 ? 'E':'W', lng_deg, lng_min);
    if (!send_variablelen_packet(HeaderType::VARIABLE_LEN, AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_SET_LON, true, (uint8_t*)databuff_lon, ARRAY_SIZE(databuff_lon)-1)) {
        return false;
    }

    // get the height in meters
    float alt_amsl_m = alt_amsl_cm * 0.01;

    // prepare and send vehicle altitude
    // sample command: #tpUD8wALT000000.0, similar format to $GPGGA
    uint8_t databuff_alt[9];
    hal.util->snprintf((char*)databuff_alt, ARRAY_SIZE(databuff_alt), "%08.1f", alt_amsl_m);
    if (!send_variablelen_packet(HeaderType::VARIABLE_LEN, AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_SET_ALT, true, (uint8_t*)databuff_alt, ARRAY_SIZE(databuff_alt)-1)) {
        return false;
    }

    // prepare and send vehicle yaw
    // sample command: #tpUD5wAZI359.9, similar format to $GPRMC
    const float veh_yaw_deg = AP::ahrs().get_yaw_deg();
    uint8_t databuff_azimuth[6];
    hal.util->snprintf((char*)databuff_azimuth, ARRAY_SIZE(databuff_azimuth), "%05.1f", veh_yaw_deg);
    if (!send_variablelen_packet(HeaderType::VARIABLE_LEN, AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_TOPOTEK_ID3CHAR_SET_AZIMUTH, true, (uint8_t*)databuff_azimuth, ARRAY_SIZE(databuff_azimuth)-1)) {
        return false;
    }

    return true;
}

// attitude information analysis of gimbal
void AP_Mount_Topotek::gimbal_angle_analyse()
{
    // consume current angles
    int16_t yaw_angle_cd = wrap_180_cd(hexchar4_to_int16(_msg_buff[10], _msg_buff[11], _msg_buff[12], _msg_buff[13]));
    int16_t pitch_angle_cd = -hexchar4_to_int16(_msg_buff[14], _msg_buff[15], _msg_buff[16], _msg_buff[17]);
    int16_t roll_angle_cd = hexchar4_to_int16(_msg_buff[18], _msg_buff[19], _msg_buff[20], _msg_buff[21]);

    // convert cd to radians
    _current_angle_rad.x = cd_to_rad(roll_angle_cd);
    _current_angle_rad.y = cd_to_rad(pitch_angle_cd);
    _current_angle_rad.z = cd_to_rad(yaw_angle_cd);
    _last_current_angle_ms = AP_HAL::millis();

    return;
}

// gimbal video information analysis
void AP_Mount_Topotek::gimbal_record_analyse()
{
    _recording = (_msg_buff[10] == '1' || _msg_buff[11] == '1');
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s recording %s", send_message_prefix, _recording ? "ON" : "OFF");
}

// information analysis of gimbal storage card
void AP_Mount_Topotek::gimbal_sdcard_analyse()
{
    if (('N' == _msg_buff[10]) && ('N' == _msg_buff[11]) && ('N' == _msg_buff[12]) && ('N' == _msg_buff[13])) {
        // memory card exception
        _sdcard_status = false;
        return;
    }
    _sdcard_status = true;

    // send UTC time to the camera
    if (_sent_time_count < 7) {
        if (send_time_to_gimbal()) {
            _sent_time_count++;
        }
    }

    return;
}

// gimbal tracking information analysis
void AP_Mount_Topotek::gimbal_track_analyse()
{
    // ignore tracking state if unchanged
    TrackingStatus tracking_state = (TrackingStatus)_msg_buff[11];
    if (tracking_state == _last_tracking_state) {
        return;
    }
    _last_tracking_state = tracking_state;

    // inform user
    const char* tracking_str = "tracking";
    switch (tracking_state) {
    case TrackingStatus::STOPPED_TRACKING:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %s stopped", send_message_prefix, tracking_str);
        _is_tracking = false;
        break;
    case TrackingStatus::WAITING_FOR_TRACKING:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %s waiting", send_message_prefix, tracking_str);
        _is_tracking = false;
        break;
    case TrackingStatus::TRACKING_IN_PROGRESS:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %s started", send_message_prefix, tracking_str);
        _is_tracking = true;
        break;
    case TrackingStatus::LENS_UNSUPPORT_TRACK:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %s unsupported lens", send_message_prefix, tracking_str);
        break;
    }
}

// gimbal distance information analysis
void AP_Mount_Topotek::gimbal_dist_info_analyse()
{
    if ('E' == _msg_buff[10] && 'R' == _msg_buff[11] && 'R' ==_msg_buff[12]) {
        _measure_dist_m = -1.0f;
        return;
    }

    // distance is in meters in the format, "12345.6" where each digit is in decimal
    _measure_dist_m = char_to_hex(_msg_buff[10]) * 10000.0 +
                      char_to_hex(_msg_buff[11]) * 1000.0 +
                      char_to_hex(_msg_buff[12]) * 100.0 +
                      char_to_hex(_msg_buff[13]) * 10.0 +
                      char_to_hex(_msg_buff[14]) +
                      char_to_hex(_msg_buff[16]) * 0.1;
}

// gimbal basic information analysis
void AP_Mount_Topotek::gimbal_version_analyse()
{
    // version array with index 0=major, 1=minor, 2=patch
    uint8_t version[3] {};

    // extract firmware version
    // the version can be in the format "1.2.3" or "123"
    const uint8_t data_buf_len = char_to_hex(_msg_buff[5]);

    // check for "."
    bool contains_period = false;
    for (uint8_t i = 0; i < data_buf_len; i++) {
        contains_period |= _msg_buff[10 + i] == '.';
    }

    // if contains period, extract version number
    uint32_t ver_num = 0;
    uint8_t ver_count = 0;
    if (contains_period) {
        for (uint8_t i = 0; i < data_buf_len; i++) {
            if (_msg_buff[10 + i] != '.') {
                ver_num = ver_num * 10 + char_to_hex(_msg_buff[10 + i]);
            } else {
                version[ver_count++] = ver_num;
                ver_num = 0;
            }
            if (ver_count >= ARRAY_SIZE(version)) {
                break;
            }
        }
    } else {
        if (data_buf_len >= 1) {
            version[0] = char_to_hex(_msg_buff[10]);
        }
        if (data_buf_len >= 2) {
            version[1] = char_to_hex(_msg_buff[11]);
        }
        if (data_buf_len >= 3) {
            version[2] = char_to_hex(_msg_buff[12]);
        }
    }
    _firmware_ver = (version[2] << 16) | (version[1] << 8) | (version[0]);

    // display gimbal model and firmware version to user
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s v%u.%u.%u",
        send_message_prefix,
        version[0],     // major version
        version[1],     // minor version
        version[2]);    // patch version

    _got_gimbal_version = true;
}

// gimbal model name message analysis
void AP_Mount_Topotek::gimbal_model_name_analyse()
{
    strncpy((char *)_model_name, (const char *)_msg_buff + 10, char_to_hex(_msg_buff[5]));

    // display gimbal model name to user
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %s", send_message_prefix, _model_name);

    _got_gimbal_model_name = true;
}

// calculate checksum
uint8_t AP_Mount_Topotek::calculate_crc(const uint8_t *cmd, uint8_t len) const
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i<len; i++) {
        crc += cmd[i];
    }
    return(crc);
}

// hexadecimal to character conversion
uint8_t AP_Mount_Topotek::hex2char(uint8_t data) const
{
    if ((9 >= data)) {
        return (data + '0');
    } else {
        return (data - 10 + 'A');
    }
}

// convert a 4 character hex number to an integer
// the characters are in the format "1234" where the most significant digit is first
int16_t AP_Mount_Topotek::hexchar4_to_int16(char high, char mid_high, char mid_low, char low) const
{
    const int16_t value = (char_to_hex(high) << 12) |
                          (char_to_hex(mid_high) << 8) |
                          (char_to_hex(mid_low) << 4) |
                          (char_to_hex(low));

    return value;
}

// send a fixed length packet
bool AP_Mount_Topotek::send_fixedlen_packet(AddressByte address, const Identifier id, bool write, uint8_t value)
{
    uint8_t databuff[3];
    hal.util->snprintf((char *)databuff, ARRAY_SIZE(databuff), "%02X", value);
    return send_variablelen_packet(HeaderType::FIXED_LEN, address, id, write, databuff, ARRAY_SIZE(databuff)-1);
}

// send variable length packet
bool AP_Mount_Topotek::send_variablelen_packet(HeaderType header, AddressByte address, const Identifier id, bool write, const uint8_t* databuff, uint8_t databuff_len)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // calculate and sanity check packet size
    const uint16_t packet_size = AP_MOUNT_TOPOTEK_PACKETLEN_MIN + databuff_len;
    if (packet_size > AP_MOUNT_TOPOTEK_PACKETLEN_MAX) {
        debug("send_packet data buff too large");
        return false;
    }

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < packet_size) {
        debug("tx buffer full");
        return false;
    }

    // create buffer for holding outgoing packet
    uint8_t send_buff[packet_size];
    uint8_t send_buff_ofs = 0;

    // packet header (bytes 0 ~ 2)
    send_buff[send_buff_ofs++] = '#';
    send_buff[send_buff_ofs++] = (header == HeaderType::FIXED_LEN) ? 'T' : 't';
    send_buff[send_buff_ofs++] = (header == HeaderType::FIXED_LEN) ? 'P' : 'p';

    // address (bytes 3, 4)
    send_buff[send_buff_ofs++] = (uint8_t)AddressByte::UART;
    send_buff[send_buff_ofs++] = (uint8_t)address;

    // data length (byte 5)
    send_buff[send_buff_ofs++] = hex2char(databuff_len);

    // control byte (byte 6)
    send_buff[send_buff_ofs++] = write ? (uint8_t)ControlByte::WRITE : (uint8_t)ControlByte::READ;

    // identified (bytes 7 ~ 9)
    send_buff[send_buff_ofs++] = id[0];
    send_buff[send_buff_ofs++] = id[1];
    send_buff[send_buff_ofs++] = id[2];

    // data
    if (databuff_len != 0) {
        memcpy(&send_buff[send_buff_ofs], databuff, databuff_len);
        send_buff_ofs += databuff_len;
    }

    // crc
    uint8_t crc = calculate_crc(send_buff, send_buff_ofs);
    send_buff[send_buff_ofs++] = hex2char((crc >> 4) & 0x0f);
    send_buff[send_buff_ofs++] = hex2char(crc & 0x0f);

    // send packet
    _uart->write(send_buff, send_buff_ofs);
    return true;
}

// set gimbal's lock vs follow mode
// lock should be true if gimbal should maintain an earth-frame target
// lock is false to follow / maintain a body-frame target
bool AP_Mount_Topotek::set_gimbal_lock(bool lock)
{
    if (_last_lock == lock) {
        return true;
    }

    // send message and update lock state
    if (send_fixedlen_packet(AddressByte::GIMBAL, AP_MOUNT_TOPOTEK_ID3CHAR_GIMBAL_MODE, true, lock ? 6 : 7)) {
        _last_lock = lock;
        return true;
    }
    return false;
}

#endif // HAL_MOUNT_TOPOTEK_ENABLED
