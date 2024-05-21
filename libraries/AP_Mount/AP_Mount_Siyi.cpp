#include "AP_Mount_Siyi.h"

#if HAL_MOUNT_SIYI_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_RTC/AP_RTC.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_SIYI_HEADER1       0x55    // first header byte
#define AP_MOUNT_SIYI_HEADER2       0x66    // second header byte
#define AP_MOUNT_SIYI_PACKETLEN_MIN 10      // minimum number of bytes in a packet.  this is a packet with no data bytes
#define AP_MOUNT_SIYI_DATALEN_MAX   (AP_MOUNT_SIYI_PACKETLEN_MAX-AP_MOUNT_SIYI_PACKETLEN_MIN) // max bytes for data portion of packet
#define AP_MOUNT_SIYI_RATE_MAX_RADS radians(90) // maximum physical rotation rate of gimbal in radans/sec
#define AP_MOUNT_SIYI_PITCH_P       1.50    // pitch controller P gain (converts pitch angle error to target rate)
#define AP_MOUNT_SIYI_YAW_P         1.50    // yaw controller P gain (converts yaw angle error to target rate)
#define AP_MOUNT_SIYI_TIMEOUT_MS    1000    // timeout for health and rangefinder readings

#define AP_MOUNT_SIYI_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_SIYI_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Siyi: " fmt, ## args); } } while (0)

// hardware lookup table indexed by HardwareModel enum values
const AP_Mount_Siyi::HWInfo AP_Mount_Siyi::hardware_lookup_table[] {
        {{'0','0'}, "Unknown"},
        {{'7','5'}, "A2"},
        {{'7','3'}, "A8"},
        {{'6','B'}, "ZR10"},
        {{'7','8'}, "ZR30"},
        {{'8','2'}, "ZT6"},
        {{'7','A'}, "ZT30"},
};

// init - performs any required initialisation for this instance
void AP_Mount_Siyi::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Gimbal, 0);
    if (_uart != nullptr) {
        _initialised = true;
    }
    AP_Mount_Backend::init();
}

// update mount position - should be called periodically
void AP_Mount_Siyi::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // reading incoming packets from gimbal
    read_incoming_packets();

    // request firmware version during startup at 1hz
    // during regular operation request configuration at 1hz
    uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_send_ms) >= 1000) {
        _last_send_ms = now_ms;
        if (!_got_hardware_id) {
            request_hardware_id();
            return;
        } else if (!_fw_version.received) {
            request_firmware_version();
            return;
        } else {
            request_configuration();
        }

#if AP_RTC_ENABLED
        // send UTC time to the camera
        if (sent_time_count < 5) {
            uint64_t utc_usec;
            if (AP::rtc().get_utc_usec(utc_usec) &&
                send_packet(SiyiCommandId::SET_TIME, (const uint8_t *)&utc_usec, sizeof(utc_usec))) {
                sent_time_count++;
            }
        }
#endif
    }

    // request attitude at regular intervals
    if ((now_ms - _last_req_current_angle_rad_ms) >= 50) {
        request_gimbal_attitude();
        _last_req_current_angle_rad_ms = now_ms;
    }

    // request rangefinder distance from ZT30 at 10hz
    if ((_hardware_model == HardwareModel::ZT30) && (now_ms - _last_rangefinder_req_ms > 100)) {
        request_rangefinder_distance();
        _last_rangefinder_req_ms = now_ms;
    }

    // send attitude to gimbal at 10Hz
    if (now_ms - _last_attitude_send_ms > 100) {
        _last_attitude_send_ms = now_ms;
        send_attitude();
    }

    // run zoom control
    update_zoom_control();

    // change to RC_TARGETING mode if RC input has changed
    set_rctargeting_on_rcinput_change();

    // Get the target angles or rates first depending on the current mount mode
    switch (get_mode()) {
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &angle_bf_target = _params.retract_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            break;
        }

        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &angle_bf_target = _params.neutral_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            break;
        }

        case MAV_MOUNT_MODE_MAVLINK_TARGETING: {
            // mavlink targets are stored while handling the incoming message
            break;
        }

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
            send_target_angles(mnt_target.angle_rad.pitch, mnt_target.angle_rad.yaw, mnt_target.angle_rad.yaw_is_ef);
            break;
        case MountTargetType::RATE:
            send_target_rates(mnt_target.rate_rads.pitch, mnt_target.rate_rads.yaw, mnt_target.rate_rads.yaw_is_ef);
            break;
    }
}

// return true if healthy
bool AP_Mount_Siyi::healthy() const
{
    // unhealthy until gimbal has been found and replied with firmware version info
    if (!_initialised || !_fw_version.received) {
        return false;
    }

    // unhealthy if attitude information NOT received recently
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_current_angle_rad_ms > AP_MOUNT_SIYI_TIMEOUT_MS) {
        return false;
    }

    // if we get this far return healthy
    return true;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Siyi::get_attitude_quaternion(Quaternion& att_quat)
{
    // Order of rotation for Siyi gimbals is (yaw, roll, pitch), which is 312 order
    att_quat.from_vector312(_current_angle_rad.x, _current_angle_rad.y, _current_angle_rad.z);
    return true;
}

// reading incoming packets from gimbal and confirm they are of the correct format
// results are held in the _parsed_msg structure
void AP_Mount_Siyi::read_incoming_packets()
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

        _msg_buff[_msg_buff_len++] = b;

        // protect against overly long messages
        if (_msg_buff_len >= AP_MOUNT_SIYI_PACKETLEN_MAX) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_parsed_msg.state) {

        case ParseState::WAITING_FOR_HEADER_LOW:
            if (b == AP_MOUNT_SIYI_HEADER1) {
                _parsed_msg.state = ParseState::WAITING_FOR_HEADER_HIGH;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_HEADER_HIGH:
            if (b == AP_MOUNT_SIYI_HEADER2) {
                _parsed_msg.state = ParseState::WAITING_FOR_CTRL;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_CTRL:
            _parsed_msg.state = ParseState::WAITING_FOR_DATALEN_LOW;
            break;

        case ParseState::WAITING_FOR_DATALEN_LOW:
            _parsed_msg.data_len = b;
            _parsed_msg.state = ParseState::WAITING_FOR_DATALEN_HIGH;
            break;

        case ParseState::WAITING_FOR_DATALEN_HIGH:
            _parsed_msg.data_len |= ((uint16_t)b << 8);
            // sanity check data length
            if (_parsed_msg.data_len <= AP_MOUNT_SIYI_DATALEN_MAX) {
                _parsed_msg.state = ParseState::WAITING_FOR_SEQ_LOW;
            } else {
                reset_parser = true;
                debug("data len too large:%u (>%u)", (unsigned)_parsed_msg.data_len, (unsigned)AP_MOUNT_SIYI_DATALEN_MAX);
            }
            break;

        case ParseState::WAITING_FOR_SEQ_LOW:
            _parsed_msg.state = ParseState::WAITING_FOR_SEQ_HIGH;
            break;

        case ParseState::WAITING_FOR_SEQ_HIGH:
            _parsed_msg.state = ParseState::WAITING_FOR_CMDID;
            break;

        case ParseState::WAITING_FOR_CMDID:
            _parsed_msg.command_id = b;
            _parsed_msg.data_bytes_received = 0;
            if (_parsed_msg.data_len > 0) {
                _parsed_msg.state = ParseState::WAITING_FOR_DATA;
            } else {
                _parsed_msg.state = ParseState::WAITING_FOR_CRC_LOW;
            }
            break;

        case ParseState::WAITING_FOR_DATA:
            _parsed_msg.data_bytes_received++;
            if (_parsed_msg.data_bytes_received >= _parsed_msg.data_len) {
                _parsed_msg.state = ParseState::WAITING_FOR_CRC_LOW;
            }
            break;

        case ParseState::WAITING_FOR_CRC_LOW:
            _parsed_msg.crc16 = b;
            _parsed_msg.state = ParseState::WAITING_FOR_CRC_HIGH;
            break;

        case ParseState::WAITING_FOR_CRC_HIGH:
            _parsed_msg.crc16 |= ((uint16_t)b << 8);

            // check crc
            const uint16_t expected_crc = crc16_ccitt(_msg_buff, _msg_buff_len-2, 0);
            if (expected_crc == _parsed_msg.crc16) {
                // successfully received a message, do something with it
                process_packet();
#if AP_MOUNT_SIYI_DEBUG
            } else {
                debug("crc expected:%x got:%x", (unsigned)expected_crc, (unsigned)_parsed_msg.crc16);
#endif
            }
            reset_parser = true;
            break;
        }

        // handle reset of parser
        if (reset_parser) {
            _parsed_msg.state = ParseState::WAITING_FOR_HEADER_LOW;
            _msg_buff_len = 0;
            reset_parser = false;
        }
    }
}

// process successfully decoded packets held in the _parsed_msg structure
void AP_Mount_Siyi::process_packet()
{
#if AP_MOUNT_SIYI_DEBUG
    // flag to warn of unexpected data buffer length
    bool unexpected_len = false;
#endif

    // process packet depending upon command id
    switch ((SiyiCommandId)_parsed_msg.command_id) {

    case SiyiCommandId::ACQUIRE_FIRMWARE_VERSION: {
        if (_parsed_msg.data_bytes_received != 12 &&    // ZR10 firmware version reply is 12bytes
            _parsed_msg.data_bytes_received != 8) {     // A8 firmware version reply is 8 bytes
#if AP_MOUNT_SIYI_DEBUG
            unexpected_len = true;
#endif
            break;
        }

        // consume and display camera firmware version
        _fw_version = {};
        _fw_version.camera.major = _msg_buff[_msg_buff_data_start+2];  // firmware major version
        _fw_version.camera.minor = _msg_buff[_msg_buff_data_start+1];  // firmware minor version
        _fw_version.camera.patch = _msg_buff[_msg_buff_data_start+0];  // firmware revision (aka patch)

        _fw_version.gimbal.major = _msg_buff[_msg_buff_data_start+6];  // firmware major version
        _fw_version.gimbal.minor = _msg_buff[_msg_buff_data_start+5];  // firmware minor version
        _fw_version.gimbal.patch = _msg_buff[_msg_buff_data_start+4];  // firmware revision (aka patch)

        // camera firmware version may be all zero soon after startup.  giveup and try again later
        if (_fw_version.camera.major == 0 && _fw_version.camera.minor == 0 && _fw_version.camera.patch == 0) {
            break;
        }

        _fw_version.received = true;

        // display camera info to user
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mount: Siyi camera fw v%u.%u.%u",
                _fw_version.camera.major,
                _fw_version.camera.minor,
                _fw_version.camera.patch);

        // display gimbal info to user
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mount: Siyi gimbal fw v%u.%u.%u",
                _fw_version.gimbal.major,
                _fw_version.gimbal.minor,
                _fw_version.gimbal.patch);

        // display zoom firmware version for those that have it
        if (_parsed_msg.data_bytes_received >= 12) {
            _fw_version.zoom.major = _msg_buff[_msg_buff_data_start+10];
            _fw_version.zoom.minor = _msg_buff[_msg_buff_data_start+ 9];
            _fw_version.zoom.patch = _msg_buff[_msg_buff_data_start+ 8];
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mount: Siyi zoom fw v%u.%u.%u",
                _fw_version.zoom.major,
                _fw_version.zoom.minor,
                _fw_version.zoom.patch);
        }

        // report to the user if gimbal firmware is not up-to-date
        check_firmware_version();

        break;
    }

    case SiyiCommandId::HARDWARE_ID: {
        // lookup first two digits of hardware id
        const uint8_t hwid0 = _msg_buff[_msg_buff_data_start];
        const uint8_t hwid1 = _msg_buff[_msg_buff_data_start+1];
        for (uint8_t i=1; i<ARRAY_SIZE(hardware_lookup_table); i++) {
            if (hwid0 == hardware_lookup_table[i].hwid[0] && hwid1 == hardware_lookup_table[i].hwid[1]) {
               _hardware_model = (HardwareModel)i;
               GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mount: Siyi %s", get_model_name());
            }
        }
        _got_hardware_id = true;
        break;
    }

    case SiyiCommandId::AUTO_FOCUS:
#if AP_MOUNT_SIYI_DEBUG
        if (_parsed_msg.data_bytes_received != 1) {
            unexpected_len = true;
            break;
        }
        debug("AutoFocus:%u", (unsigned)_msg_buff[_msg_buff_data_start]);
#endif
        break;

    case SiyiCommandId::MANUAL_ZOOM_AND_AUTO_FOCUS: {
        if (_parsed_msg.data_bytes_received != 2) {
#if AP_MOUNT_SIYI_DEBUG
            unexpected_len = true;
#endif
            break;
        }
        _zoom_mult = UINT16_VALUE(_msg_buff[_msg_buff_data_start+1], _msg_buff[_msg_buff_data_start]) * 0.1;
        debug("ZoomMult:%4.1f", (double)_zoom_mult);
        break;
    }

    case SiyiCommandId::MANUAL_FOCUS:
#if AP_MOUNT_SIYI_DEBUG
        if (_parsed_msg.data_bytes_received != 1) {
            unexpected_len = true;
            break;
        }
        debug("ManualFocus:%u", (unsigned)_msg_buff[_msg_buff_data_start]);
#endif
        break;

    case SiyiCommandId::GIMBAL_ROTATION:
#if AP_MOUNT_SIYI_DEBUG
        if (_parsed_msg.data_bytes_received != 1) {
            unexpected_len = true;
            break;
        }
        debug("GimbRot:%u", (unsigned)_msg_buff[_msg_buff_data_start]);
#endif
        break;

    case SiyiCommandId::CENTER:
#if AP_MOUNT_SIYI_DEBUG
        if (_parsed_msg.data_bytes_received != 1) {
            unexpected_len = true;
            break;
        }
        debug("Center:%u", (unsigned)_msg_buff[_msg_buff_data_start]);
#endif
        break;

    case SiyiCommandId::ACQUIRE_GIMBAL_CONFIG_INFO: {
        const RecordingStatus prev_record_status = _config_info.record_status;

        // Update Gimbal Config Information
        size_t config_sz = MIN(_parsed_msg.data_bytes_received, sizeof(_config_info));
        memcpy(&_config_info, &_msg_buff[_msg_buff_data_start], config_sz);

        // Alert user if recording status changed
        if (prev_record_status != _config_info.record_status) {
            const char * msg = "?";
            MAV_SEVERITY sev = MAV_SEVERITY_INFO;
            switch (_config_info.record_status) {
                case RecordingStatus::OFF:
                    msg = "OFF";
                    break;
                case RecordingStatus::ON:
                    msg = "ON";
                    break;
                case RecordingStatus::NO_CARD:
                    msg = "NO CARD!";
                    sev = MAV_SEVERITY_WARNING;
                    break;
                case RecordingStatus::DATA_LOSS:
                    msg = "DATA LOSS!";
                    sev = MAV_SEVERITY_WARNING;
                    break;
            }
            (void)msg;  // in case GCS_SEND_TEXT not available
            (void)sev;  // in case GCS_SEND_TEXT not available
            GCS_SEND_TEXT(sev, "Siyi: recording %s", msg);
        }

        debug(
            "GimConf hdr:%u rec:%u foll:%u mntdir:%u vid:%u",
            (uint8_t)_config_info.hdr_status,
            (uint8_t)_config_info.record_status,
            (uint8_t)_config_info.motion_mode,
            (uint8_t)_config_info.mounting_dir,
            (uint8_t)_config_info.video_mode
        );
        break;
    }

    case SiyiCommandId::FUNCTION_FEEDBACK_INFO: {
        if (_parsed_msg.data_bytes_received != 1) {
#if AP_MOUNT_SIYI_DEBUG
            unexpected_len = true;
#endif
            break;
        }
        const uint8_t func_feedback_info = _msg_buff[_msg_buff_data_start];
        const char* err_prefix = "Mount: Siyi";
        (void)err_prefix;  // in case !HAL_GCS_ENABLED
        switch ((FunctionFeedbackInfo)func_feedback_info) {
        case FunctionFeedbackInfo::SUCCESS:
            debug("FnFeedB success");
            break;
        case FunctionFeedbackInfo::FAILED_TO_TAKE_PHOTO:
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s failed to take picture", err_prefix);
            break;
        case FunctionFeedbackInfo::HDR_ON:
            debug("HDR on");
            break;
        case FunctionFeedbackInfo::HDR_OFF:
            debug("HDR off");
            break;
        case FunctionFeedbackInfo::FAILED_TO_RECORD_VIDEO:
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s failed to record video", err_prefix);
            break;
        default:
            debug("FnFeedB unexpected val:%u", (unsigned)func_feedback_info);
        }
        break;
    }

    case SiyiCommandId::PHOTO:
        // no ack should ever be sent by the gimbal
        break;

    case SiyiCommandId::ACQUIRE_GIMBAL_ATTITUDE: {
        if (_parsed_msg.data_bytes_received != 12) {
#if AP_MOUNT_SIYI_DEBUG
            unexpected_len = true;
#endif
            break;
        }
        _last_current_angle_rad_ms = AP_HAL::millis();
        _current_angle_rad.z = -radians((int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+1], _msg_buff[_msg_buff_data_start]) * 0.1);   // yaw angle
        _current_angle_rad.y = radians((int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+3], _msg_buff[_msg_buff_data_start+2]) * 0.1);  // pitch angle
        _current_angle_rad.x = radians((int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+5], _msg_buff[_msg_buff_data_start+4]) * 0.1);  // roll angle
        _current_rates_rads.z = -radians((int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+7], _msg_buff[_msg_buff_data_start+6]) * 0.1);   // yaw rate
        _current_rates_rads.y = radians((int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+9], _msg_buff[_msg_buff_data_start+8]) * 0.1);   // pitch rate
        _current_rates_rads.x = radians((int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+11], _msg_buff[_msg_buff_data_start+10]) * 0.1);  // roll rate
        break;
    }

    case SiyiCommandId::READ_RANGEFINDER: {
        _rangefinder_dist_m = UINT16_VALUE(_msg_buff[_msg_buff_data_start+1], _msg_buff[_msg_buff_data_start]);
        _last_rangefinder_dist_ms = AP_HAL::millis();
        break;
    }

    default:
        debug("Unhandled CmdId:%u", (unsigned)_parsed_msg.command_id);
        break;
    }

#if AP_MOUNT_SIYI_DEBUG
    // handle unexpected data buffer length
    if (unexpected_len) {
        debug("CmdId:%u unexpected len:%u", (unsigned)_parsed_msg.command_id, (unsigned)_parsed_msg.data_bytes_received);
    }
#endif
}

// methods to send commands to gimbal
// returns true on success, false if outgoing serial buffer is full
bool AP_Mount_Siyi::send_packet(SiyiCommandId cmd_id, const uint8_t* databuff, uint8_t databuff_len)
{
    if (!_initialised) {
        return false;
    }
    // calculate and sanity check packet size
    const uint16_t packet_size = AP_MOUNT_SIYI_PACKETLEN_MIN + databuff_len;
    if (packet_size > AP_MOUNT_SIYI_PACKETLEN_MAX) {
        debug("send_packet data buff too large");
        return false;
    }

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < packet_size) {
        return false;
    }

    // buffer for holding outgoing packet
    uint8_t send_buff[packet_size];
    uint8_t send_buff_ofs = 0;

    // packet header
    send_buff[send_buff_ofs++] = AP_MOUNT_SIYI_HEADER1;
    send_buff[send_buff_ofs++] = AP_MOUNT_SIYI_HEADER2;

    // CTRL.  Always request ACK
    send_buff[send_buff_ofs++] = 1;

    // Data_len.  protocol supports uint16_t but messages are never longer than 22 bytes
    send_buff[send_buff_ofs++] = databuff_len;
    send_buff[send_buff_ofs++] = 0;

    // SEQ (sequence)
    send_buff[send_buff_ofs++] = LOWBYTE(_last_seq);
    send_buff[send_buff_ofs++] = HIGHBYTE(_last_seq++);

    // CMD_ID
    send_buff[send_buff_ofs++] = (uint8_t)cmd_id;

    // DATA
    if (databuff_len != 0) {
        memcpy(&send_buff[send_buff_ofs], databuff, databuff_len);
        send_buff_ofs += databuff_len;
    }

    // CRC16
    const uint16_t crc = crc16_ccitt(send_buff, send_buff_ofs, 0);
    send_buff[send_buff_ofs++] = LOWBYTE(crc);
    send_buff[send_buff_ofs++] = HIGHBYTE(crc);

    // send packet
    _uart->write(send_buff, send_buff_ofs);

    return true;
}

// send a packet with a single data byte to gimbal
// returns true on success, false if outgoing serial buffer is full
bool AP_Mount_Siyi::send_1byte_packet(SiyiCommandId cmd_id, uint8_t data_byte)
{
    return send_packet(cmd_id, &data_byte, 1);
}

// rotate gimbal.  pitch_rate and yaw_rate are scalars in the range -100 ~ +100
// yaw_is_ef should be true if gimbal should maintain an earth-frame target (aka lock)
void AP_Mount_Siyi::rotate_gimbal(int8_t pitch_scalar, int8_t yaw_scalar, bool yaw_is_ef)
{
    // send lock/follow value
    const GimbalMotionMode mode = yaw_is_ef ? GimbalMotionMode::LOCK : GimbalMotionMode::FOLLOW;
    if (!set_motion_mode(mode)) {
        // couldn't set mode, so don't send rotation
        return;
    }

    const uint8_t yaw_and_pitch_rates[] {(uint8_t)yaw_scalar, (uint8_t)pitch_scalar};
    send_packet(SiyiCommandId::GIMBAL_ROTATION, yaw_and_pitch_rates, ARRAY_SIZE(yaw_and_pitch_rates));
}

// Set gimbal's motion mode if it has changed. Use force=true to always send.
//   FOLLOW: roll and pitch are in earth-frame, yaw is in body-frame
//   LOCK: roll, pitch and yaw are all in earth-frame
//   FPV: roll, pitch and yaw are all in body-frame
// Returns true if mode successfully sent to Gimbal
bool AP_Mount_Siyi::set_motion_mode(const GimbalMotionMode mode, const bool force)
{
    if (!force && (mode == _config_info.motion_mode)) {
        // we're already in the right mode...
        return true;
    }

    PhotoFunction data = PhotoFunction::LOCK_MODE;
    switch (mode) {
        case GimbalMotionMode::LOCK:   data = PhotoFunction::LOCK_MODE; break;
        case GimbalMotionMode::FOLLOW: data = PhotoFunction::FOLLOW_MODE; break;
        case GimbalMotionMode::FPV:    data = PhotoFunction::FPV_MODE; break;
    }
    bool sent = send_1byte_packet(SiyiCommandId::PHOTO, (uint8_t)data);
    if (sent) {
        // assume the mode is set correctly until told otherwise
        _config_info.motion_mode = mode;
        request_configuration();
    }
    return sent;
}

// send target pitch and yaw rates to gimbal
// yaw_is_ef should be true if yaw_rads target is an earth frame rate, false if body_frame
void AP_Mount_Siyi::send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef)
{
    const float pitch_rate_scalar = constrain_float(100.0 * pitch_rads / AP_MOUNT_SIYI_RATE_MAX_RADS, -100, 100);
    const float yaw_rate_scalar = constrain_float(100.0 * yaw_rads / AP_MOUNT_SIYI_RATE_MAX_RADS, -100, 100);
    rotate_gimbal(pitch_rate_scalar, yaw_rate_scalar, yaw_is_ef);
}

// send target pitch and yaw angles to gimbal
// yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
void AP_Mount_Siyi::send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef)
{
    // stop gimbal if no recent actual angles
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_current_angle_rad_ms >= 200) {
        rotate_gimbal(0, 0, false);
        return;
    }

    // if gimbal mounting direction is 2 i.e. upside down, then transform the angles
    Vector3f current_angle_transformed = _current_angle_rad;
    if (_config_info.mounting_dir == GimbalMountingDirection::UPSIDE_DOWN) {
        current_angle_transformed.y = -wrap_PI(_current_angle_rad.y + M_PI);
        current_angle_transformed.z = -_current_angle_rad.z;
    }

    // use simple P controller to convert pitch angle error (in radians) to a target rate scalar (-100 to +100)
    const float pitch_err_rad = (pitch_rad - current_angle_transformed.y);
    const float pitch_rate_scalar = constrain_float(100.0 * pitch_err_rad * AP_MOUNT_SIYI_PITCH_P / AP_MOUNT_SIYI_RATE_MAX_RADS, -100, 100);

    // convert yaw angle to body-frame
    float yaw_bf_rad = yaw_is_ef ? wrap_PI(yaw_rad - AP::ahrs().get_yaw()) : yaw_rad;

    // enforce body-frame yaw angle limits.  If beyond limits always use body-frame control
    const float yaw_bf_min = radians(_params.yaw_angle_min);
    const float yaw_bf_max = radians(_params.yaw_angle_max);
    if (yaw_bf_rad < yaw_bf_min || yaw_bf_rad > yaw_bf_max) {
        yaw_bf_rad = constrain_float(yaw_bf_rad, yaw_bf_min, yaw_bf_max);
        yaw_is_ef = false;
    }

    // use simple P controller to convert yaw angle error to a target rate scalar (-100 to +100)
    const float yaw_err_rad = (yaw_bf_rad - current_angle_transformed.z);
    const float yaw_rate_scalar = constrain_float(100.0 * yaw_err_rad * AP_MOUNT_SIYI_YAW_P / AP_MOUNT_SIYI_RATE_MAX_RADS, -100, 100);

    // rotate gimbal.  pitch_rate and yaw_rate are scalars in the range -100 ~ +100
    rotate_gimbal(pitch_rate_scalar, yaw_rate_scalar, yaw_is_ef);
}

// take a picture.  returns true on success
bool AP_Mount_Siyi::take_picture()
{
    return send_1byte_packet(SiyiCommandId::PHOTO, (uint8_t)PhotoFunction::TAKE_PICTURE);
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_Siyi::record_video(bool start_recording)
{
    // exit immediately if not initialised to reduce mismatch
    // between internal and actual state of recording
    if (!_initialised) {
        return false;
    }

    bool success = true;
    bool send_toggle = false;
    if (start_recording) {
        switch (_config_info.record_status) {
            case RecordingStatus::ON:
                // already recording...
                break;
            // assume that DATA_LOSS is the same as OFF
            case RecordingStatus::DATA_LOSS:
            case RecordingStatus::OFF:
                send_toggle = true;
                break;
            case RecordingStatus::NO_CARD:
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Siyi: can't start recording: No Card");
                success = false;
                break;
        }
    } else {
        switch (_config_info.record_status) {
            case RecordingStatus::ON:
                send_toggle = true;
                break;
            // assume that DATA_LOSS is the same as OFF
            case RecordingStatus::DATA_LOSS:
            case RecordingStatus::OFF:
            case RecordingStatus::NO_CARD:
                // already off...
                break;
        }
    }

    if (send_toggle) {
        success = send_1byte_packet(SiyiCommandId::PHOTO, (uint8_t)PhotoFunction::RECORD_VIDEO_TOGGLE);
    }

    // request recording state update from gimbal
    request_configuration();

    return success;
}

// send zoom rate command to camera. zoom out = -1, hold = 0, zoom in = 1
bool AP_Mount_Siyi::send_zoom_rate(float zoom_value)
{
    uint8_t zoom_step = 0;
    if (zoom_value > 0) {
        // zoom in
        zoom_step = 1;
    }
    if (zoom_value < 0) {
        // zoom out. Siyi API specifies -1 should be sent as 255
        zoom_step = UINT8_MAX;
    }
    return send_1byte_packet(SiyiCommandId::MANUAL_ZOOM_AND_AUTO_FOCUS, zoom_step);
}

// send zoom multiple command to camera. e.g. 1x, 10x, 30x
// only works on ZR10 and ZR30
bool AP_Mount_Siyi::send_zoom_mult(float zoom_mult)
{
    // separate zoom_mult into integral and fractional parts
    float intpart;
    uint8_t fracpart = (uint8_t)constrain_int16(modf(zoom_mult, &intpart) * 10, 0, UINT8_MAX);

    // create and send 2 byte array
    const uint8_t zoom_mult_data[] {(uint8_t)(intpart), fracpart};
    return send_packet(SiyiCommandId::ABSOLUTE_ZOOM, zoom_mult_data, ARRAY_SIZE(zoom_mult_data));
}

// get zoom multiple max
float AP_Mount_Siyi::get_zoom_mult_max() const
{
    switch (_hardware_model) {
    case HardwareModel::UNKNOWN:
        return 0;
    case HardwareModel::A2:
    case HardwareModel::A8:
    case HardwareModel::ZT6:
        // a8, zt6 have 6x digital zoom
        return 6;
    case HardwareModel::ZR10:
    case HardwareModel::ZR30:
    case HardwareModel::ZT30:
        // 30x hybrid zoom (optical + digital)
        return 30;
    }
    return 0;
}

// set zoom specified as a rate or percentage
bool AP_Mount_Siyi::set_zoom(ZoomType zoom_type, float zoom_value)
{
    switch (zoom_type) {
    case ZoomType::RATE:
        if (send_zoom_rate(zoom_value)) {
            _zoom_type = zoom_type;
            _zoom_rate_target = zoom_value;
            return true;
        }
        return false;
    case ZoomType::PCT: {
        // absolute zoom
        float zoom_mult_max = get_zoom_mult_max();
        if (is_positive(zoom_mult_max)) {
            // convert zoom percentage (0~100) to target zoom multiple (e.g. 0~6x or 0~30x)
            const float zoom_mult = linear_interpolate(1, zoom_mult_max, zoom_value, 0, 100);
            if (send_zoom_mult(zoom_mult)) {
                _zoom_type = zoom_type;
                return true;
            }
            return false;
        }
        return false;
    }
    }

    // unsupported zoom type
    return false;
}

// update zoom controller
void AP_Mount_Siyi::update_zoom_control()
{
    if (_zoom_type == ZoomType::RATE) {
        // limit updates to 1hz
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - _last_zoom_control_ms < 1000) {
            return;
        }
        _last_zoom_control_ms = now_ms;

        // only send zoom rate target if it's non-zero because if zero it has already been sent
        // and sending zero rate also triggers autofocus
        if (!is_zero(_zoom_rate_target)) {
            send_zoom_rate(_zoom_rate_target);
        }
    }
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Mount_Siyi::set_focus(FocusType focus_type, float focus_value)
{
    switch (focus_type) {
    case FocusType::RATE: {
        uint8_t focus_step = 0;
        if (focus_value > 0) {
            focus_step = 1;
        } else if (focus_value < 0) {
            // Siyi API specifies -1 should be sent as 255
            focus_step = UINT8_MAX;
        }
        if (!send_1byte_packet(SiyiCommandId::MANUAL_FOCUS, (uint8_t)focus_step)) {
            return SetFocusResult::FAILED;
        }
        return SetFocusResult::ACCEPTED;
    }
    case FocusType::PCT:
        // not supported
        return SetFocusResult::INVALID_PARAMETERS;
    case FocusType::AUTO:
        if (!send_1byte_packet(SiyiCommandId::AUTO_FOCUS, 1)) {
            return SetFocusResult::FAILED;
        }
        return SetFocusResult::ACCEPTED;
    }

    // unsupported focus type
    return SetFocusResult::INVALID_PARAMETERS;
}

// set camera lens as a value from 0 to 8
bool AP_Mount_Siyi::set_lens(uint8_t lens)
{
    // only supported on ZT30.  sanity check lens values
    if ((_hardware_model != HardwareModel::ZT30) || (lens > 8)) {
        return false;
    }

    // maps lens to siyi camera image type so that lens of 0, 1, 2 are more useful
    CameraImageType cam_image_type = CameraImageType::MAIN_ZOOM_SUB_THERMAL;
    switch (lens) {
        case 0:
            cam_image_type = CameraImageType::MAIN_ZOOM_SUB_THERMAL; // 3
            break;
        case 1:
            cam_image_type = CameraImageType::MAIN_WIDEANGLE_SUB_THERMAL; // 5
            break;
        case 2:
            cam_image_type = CameraImageType::MAIN_THERMAL_SUB_ZOOM; // 7
            break;
        case 3:
            cam_image_type = CameraImageType::MAIN_PIP_ZOOM_THERMAL_SUB_WIDEANGLE; // 0
            break;
        case 4:
            cam_image_type = CameraImageType::MAIN_PIP_WIDEANGLE_THERMAL_SUB_ZOOM; // 1
            break;
        case 5:
            cam_image_type = CameraImageType::MAIN_PIP_ZOOM_WIDEANGLE_SUB_THERMAL; // 2
            break;
        case 6:
            cam_image_type = CameraImageType::MAIN_ZOOM_SUB_WIDEANGLE; // 4
            break;
        case 7:
            cam_image_type = CameraImageType::MAIN_WIDEANGLE_SUB_ZOOM; // 6
            break;
        case 8:
            cam_image_type = CameraImageType::MAIN_THERMAL_SUB_WIDEANGLE; // 8
            break;
    }

    // send desired image type to camera
    return send_1byte_packet(SiyiCommandId::SET_CAMERA_IMAGE_TYPE, (uint8_t)cam_image_type);
}

// set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
// primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
bool AP_Mount_Siyi::set_camera_source(uint8_t primary_source, uint8_t secondary_source)
{
    // only supported on ZT30.  sanity check lens values
    if (_hardware_model != HardwareModel::ZT30) {
        return false;
    }

    // maps primary and secondary source to siyi camera image type
    CameraImageType cam_image_type;
    switch (primary_source) {
    case 0: // Default (RGB)
        FALLTHROUGH;
    case 1: // RGB
        switch (secondary_source) {
        case 0: // RGB + Default (None)
            cam_image_type = CameraImageType::MAIN_ZOOM_SUB_THERMAL;                // 3
            break;
        case 2: // PIP RGB+IR
            cam_image_type = CameraImageType::MAIN_PIP_ZOOM_THERMAL_SUB_WIDEANGLE;  // 0
            break;
        case 4: // PIP RGB+RGB_WIDEANGLE
            cam_image_type = CameraImageType::MAIN_PIP_ZOOM_WIDEANGLE_SUB_THERMAL;  // 2
            break;
        default:
            return false;
        }
        break;
    case 2: // IR
        switch (secondary_source) {
        case 0: // IR + Default (None)
            cam_image_type = CameraImageType::MAIN_THERMAL_SUB_ZOOM;                // 7
            break;
        default:
            return false;
        }
        break;
    case 4: // RGB_WIDEANGLE
        switch (secondary_source) {
        case 0: // RGB_WIDEANGLE + Default (None)
            cam_image_type = CameraImageType::MAIN_WIDEANGLE_SUB_THERMAL;           // 5
            break;
        case 2: // PIP RGB_WIDEANGLE+IR
            cam_image_type = CameraImageType::MAIN_PIP_WIDEANGLE_THERMAL_SUB_ZOOM;  // 1
            break;
        default:
            return false;
        }
        break;
    default:
        return false;
    }

    // send desired image type to camera
    return send_1byte_packet(SiyiCommandId::SET_CAMERA_IMAGE_TYPE, (uint8_t)cam_image_type);
}

// send camera information message to GCS
void AP_Mount_Siyi::send_camera_information(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised || !_fw_version.received) {
        return;
    }

    static const uint8_t vendor_name[32] = "Siyi";
    static uint8_t model_name[32] {};
    const uint32_t fw_version = _fw_version.camera.major | (_fw_version.camera.minor << 8) | (_fw_version.camera.patch << 16);
    const char cam_definition_uri[140] {};

    // copy model name
    strncpy((char *)model_name, get_model_name(), sizeof(model_name)-1);

    // focal length
    // To-Do: check these values are correct for A2, ZR30, ZT30
    float focal_length_mm = 0;
    switch (_hardware_model) {
    case HardwareModel::UNKNOWN:
    case HardwareModel::A2:
    case HardwareModel::A8:
    case HardwareModel::ZT6:
        focal_length_mm = 21;
        break;
    case HardwareModel::ZR10:
    case HardwareModel::ZR30:
    case HardwareModel::ZT30:
        // focal length range from 5.15 ~ 47.38
        focal_length_mm = 5.15;
        break;
    }

    // capability flags
    const uint32_t flags = CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                           CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                           CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM |
                           CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS;

    // send CAMERA_INFORMATION message
    mavlink_msg_camera_information_send(
        chan,
        AP_HAL::millis(),       // time_boot_ms
        vendor_name,            // vendor_name uint8_t[32]
        model_name,             // model_name uint8_t[32]
        fw_version,             // firmware version uint32_t
        focal_length_mm,        // focal_length float (mm)
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
void AP_Mount_Siyi::send_camera_settings(mavlink_channel_t chan) const
{
    const uint8_t mode_id = (_config_info.record_status == RecordingStatus::ON) ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE;
    const float NaN = nanf("0x4152");
    const float zoom_mult_max = get_zoom_mult_max();
    float zoom_pct = 0.0;
    if (is_positive(zoom_mult_max)) {
        zoom_pct = linear_interpolate(0, 100, _zoom_mult, 1.0, zoom_mult_max);
    }

    // send CAMERA_SETTINGS message
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        mode_id,            // camera mode (0:image, 1:video, 2:image survey)
        zoom_pct,           // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaN);               // focusLevel float, percentage from 0 to 100, NaN if unknown
}

// get model name string. returns "Unknown" if hardware model is not yet known
const char* AP_Mount_Siyi::get_model_name() const
{
    uint8_t model_idx = (uint8_t)_hardware_model;
    if (model_idx < ARRAY_SIZE(hardware_lookup_table)) {
        return hardware_lookup_table[model_idx].model_name;
    }
    return hardware_lookup_table[0].model_name;
}

// get rangefinder distance.  Returns true on success
bool AP_Mount_Siyi::get_rangefinder_distance(float& distance_m) const
{
    // only supported on ZT30
    if (_hardware_model != HardwareModel::ZT30) {
        return false;
    }

    // unhealthy if distance not received recently
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_rangefinder_dist_ms > AP_MOUNT_SIYI_TIMEOUT_MS) {
        return false;
    }

    distance_m = _rangefinder_dist_m;
    return true;
}

// Checks that the firmware version on the Gimbal meets the minimum supported version.
void AP_Mount_Siyi::check_firmware_version() const
{
    if (!_fw_version.received) {
        debug("Can't check firmware if we haven't received it...");
        return;
    }

    if (!_got_hardware_id) {
        debug("Can't check firmware without Hardware ID!");
        return;
    }

    FirmwareVersion minimum_ver {};
    switch (_hardware_model) {
        case HardwareModel::A8:
            minimum_ver.camera.major = 0;
            minimum_ver.camera.minor = 2;
            minimum_ver.camera.patch = 1;
            break;

        case HardwareModel::A2:
        case HardwareModel::ZR10:
        case HardwareModel::ZR30:
        case HardwareModel::ZT6:
        case HardwareModel::ZT30:
            // TBD
            break;

        case HardwareModel::UNKNOWN:
            debug("Can't check FW on unknown hardware model!");
            return;
    }

    const uint32_t minimum_camera_val =  (minimum_ver.camera.major << 16) + (minimum_ver.camera.minor << 8) + minimum_ver.camera.patch;
    const uint32_t firmware_camera_val = (_fw_version.camera.major << 16) + (_fw_version.camera.minor << 8) + _fw_version.camera.patch;

    const bool is_camera_supported = firmware_camera_val >= minimum_camera_val;

    if (!is_camera_supported) {
        GCS_SEND_TEXT(
            MAV_SEVERITY_WARNING,
            "Mount: Siyi running old camera fw (need v%u.%u.%u)",
            minimum_ver.camera.major, minimum_ver.camera.minor, minimum_ver.camera.patch
        );
    }
}

/*
 send ArduPilot attitude to gimbal
*/
void AP_Mount_Siyi::send_attitude(void)
{
    const auto &ahrs = AP::ahrs();
    struct {
        uint32_t time_boot_ms;
        float roll, pitch, yaw;
        float rollspeed, pitchspeed, yawspeed;
    } attitude;

    // get attitude as euler 321
    const auto &gyro = ahrs.get_gyro();
    const uint32_t now_ms = AP_HAL::millis();

    attitude.time_boot_ms = now_ms;
    attitude.roll = ahrs.get_roll();
    attitude.pitch = ahrs.get_pitch();
    attitude.yaw = ahrs.get_yaw();
    attitude.rollspeed = gyro.x;
    attitude.pitchspeed = gyro.y;
    attitude.yawspeed = gyro.z;

    send_packet(SiyiCommandId::EXTERNAL_ATTITUDE, (const uint8_t *)&attitude, sizeof(attitude));
}

#endif // HAL_MOUNT_SIYI_ENABLED
