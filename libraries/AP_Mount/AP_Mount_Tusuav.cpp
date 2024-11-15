#include "AP_Mount_config.h"
#if HAL_MOUNT_TUSUAV_ENABLED
#include "AP_Mount_Tusuav.h"
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RTC/AP_RTC.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>

extern const AP_HAL::HAL& hal;

constexpr uint8_t AP_MOUNT_TUSUAV_HEADER1 = 0x5A; // first header byte
#define AP_MOUNT_TUSUAV_PACKETLEN_MIN 6        // Minimum number of bytes in a packet (1 header byte, 1 flag byte, 1 command byte, 1 sub-command byte, length byte, and 1 CRC byte)
#define AP_MOUNT_TUSUAV_DATALEN_MAX   (AP_MOUNT_TUSUAV_PACKETLEN_MAX - AP_MOUNT_TUSUAV_PACKETLEN_MIN) // Maximum number of bytes for the data portion of the packet
#define AP_MOUNT_TUSUAV_HEALTH_TIMEOUT_MS 1000 // Timeout in milliseconds for considering the state unhealthy if no attitude data is received
#define AP_MOUNT_TUSUAV_SEND_INTERVAL_MS 100 // Interval in milliseconds for resending angle or rate targets to the gimbal
#define AP_MOUNT_TUSUAV_PARSE_INTERVAL_MS 1 // Interval in milliseconds for parsing data from the gimbal
#define AP_MOUNT_TUSUAV_TIME_SYNC_INTERVAL_MS 1000 // Interval in milliseconds for resending time synchronization packets to the gimbal
#define AP_MOUNT_TUSUAV_ZOOM_SPEED     500    // Hard-coded zoom speed (high speed)
#define AP_MOUNT_TUSUAV_ZOOM_MAX       30      // Maximum zoom level
#define AP_MOUNT_TUSUAV_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_TUSUAV_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Tusuav: " fmt, ## args); } } while (0)

const char* AP_Mount_Tusuav::send_text_prefix = "Tusuav:";

// update mount position - should be called periodically
void AP_Mount_Tusuav::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }
    // below here we sent angle or rate targets
    // throttle sends of target angles or rates
    const uint32_t now_ms = AP_HAL::millis();
    
    if (now_ms - _last_update_ms_parse >= AP_MOUNT_TUSUAV_PARSE_INTERVAL_MS) {
        _last_update_ms_parse = now_ms;
        // reading incoming packets from gimbal
        read_incoming_packets();
    }

    // send time sync
    if (now_ms - _last_time_sync_ms_send >= AP_MOUNT_TUSUAV_TIME_SYNC_INTERVAL_MS) {
        _last_time_sync_ms_send = now_ms;
        send_time_sync();
    }

    if (now_ms - _last_update_ms_send < AP_MOUNT_TUSUAV_SEND_INTERVAL_MS) {
        return;
    }
    _last_update_ms_send = now_ms;

    //request tracking status and pod angle
    request_tracking_status();

    // request model name
    if (!_got_model_name) {
        request_models();
    }

    // request firmware version
    if (!_got_firmware_version) {
        request_versions();
    }

    // send vehicle attitude and position
    send_m_ahrs();

    // change to RC_TARGETING mode if RC input has changed
    set_rctargeting_on_rcinput_change();

    MountTarget rc_target;
    get_rc_target(mnt_target.target_type, rc_target);
	if (mnt_target.target_type == MountTargetType::RATE) {
		const int16_t pitch_value = degrees(rc_target.pitch) / _params.rc_rate_max.get() * 500;
		const int16_t yaw_value = degrees(rc_target.yaw) / _params.rc_rate_max.get() * 500;
		send_stick_pitch_yaw(pitch_value,yaw_value);
        debug("stick ：%d  %d", pitch_value, yaw_value);
	}

    // if tracking is active we do not send new targets to the gimbal
    if (_last_tracking_status == TrackingStatus::Searching || _last_tracking_status == TrackingStatus::Tracking) {
        return;
    }

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
    MessageGimbalAngleRates gimbal_angle_mode{};
    switch (mnt_target.target_type) {
    case MountTargetType::ANGLE:
        if (mnt_target.angle_rad.yaw_is_ef) {
            gimbal_angle_mode.mode.bits.yaw = static_cast<uint8_t>(GimbalAngleMode::Earth_Frame_Angle);
            debug("euler：%f  %f", gimbal_angle_mode.ef_angle[0], gimbal_angle_mode.ef_angle[2]);
        } else {
            gimbal_angle_mode.mode.bits.yaw = static_cast<uint8_t>(GimbalAngleMode::Body_Frame_Angle);
            debug("enc：%f  %f", gimbal_angle_mode.bf_angle[0], gimbal_angle_mode.bf_angle[2]);
        }
        gimbal_angle_mode.mode.bits.pitch = static_cast<uint8_t>(GimbalAngleMode::Earth_Frame_Angle);
        gimbal_angle_mode.mode.bits.roll = static_cast<uint8_t>(GimbalAngleMode::Stabilize);
        gimbal_angle_mode.mode.bits.special = 0;
        gimbal_angle_mode.ef_angle[0] = degrees(mnt_target.angle_rad.pitch);
        gimbal_angle_mode.ef_angle[2] = degrees(mnt_target.angle_rad.yaw);
        debug("ANGLE_mode：%d", (uint8_t)gimbal_angle_mode.mode.all);
        break;
    case MountTargetType::RATE:
        gimbal_angle_mode.mode.bits.pitch = static_cast<uint8_t>(GimbalAngleMode::Stabilize);
        gimbal_angle_mode.mode.bits.yaw = static_cast<uint8_t>(GimbalAngleMode::Stabilize);
        debug("ANGLE_mode：%d", (uint8_t)gimbal_angle_mode.mode.all);
        break;
    }
    send_target_angles_rates(gimbal_angle_mode);
}

// return true if healthy
bool AP_Mount_Tusuav::healthy() const
{
    // unhealthy until gimbal has been found and replied with firmware version info
    if (!_initialised) {
        return false;
    }
    // unhealthy if attitude information NOT received recently
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_current_angle_rad_ms > AP_MOUNT_TUSUAV_HEALTH_TIMEOUT_MS) {
        return false;
    }
    return true;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Tusuav::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(_current_angle_rad.x, _current_angle_rad.y, _current_angle_rad.z);
    return true;
}

// reading incoming packets from gimbal and confirm they are of the correct format
void AP_Mount_Tusuav::read_incoming_packets()
{
    // check for bytes on the serial port
    const auto nbytes = MIN(_uart->available(), 1024U);
    if (nbytes <= 0 ) {
        return;
    }

    // flag to allow cases below to reset parser state
    bool reset_parser = false;

    // process bytes received
    for (auto i=0U; i<nbytes; i++) {
        uint8_t b;
        if (!_uart->read(b)) {
            break;
        }

        _msg_buff[_msg_buff_len++] = b;

        // protect against overly long messages
        if (_msg_buff_len >= ARRAY_SIZE(_msg_buff)) {
            reset_parser = true;
            debug("tus buff full s:%u len:%u", (unsigned)_parsed_msg.state, (unsigned)_msg_buff_len);
        }
        
        // process byte depending upon current state
        switch (_parsed_msg.state) {
        case ParseState::Waiting_For_Header:
            if (b == AP_MOUNT_TUSUAV_HEADER1) {
                _parsed_msg.state = ParseState::Waiting_For_Total_Length;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::Waiting_For_Total_Length:
            if (b >= AP_MOUNT_TUSUAV_PACKETLEN_MIN) {
                _parsed_msg.msg.total_len = b;
                _parsed_msg.msg.data_len = _parsed_msg.msg.total_len - AP_MOUNT_TUSUAV_PACKETLEN_MIN;
                _parsed_msg.state = ParseState::Waiting_For_Flag;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::Waiting_For_Flag:
            // length held in bits 0 ~ 5.  length includes this length byte, frame id and final crc
            // ignore frame counter held in bits 6~7
            _parsed_msg.msg.flag.all = b;
            _parsed_msg.state = ParseState::Waiting_For_Cmd;
            break;

        case ParseState::Waiting_For_Cmd:
            _parsed_msg.msg.cmd = b;
            // _parsed_msg.data_bytes_received = 0;
            _parsed_msg.state = ParseState::Waiting_For_Sub_Cmd;
            break;

        case ParseState::Waiting_For_Sub_Cmd:
            _parsed_msg.msg.sub_cmd = b;
            _parsed_msg.state = ParseState::Waiting_For_Data;
            break;

        case ParseState::Waiting_For_Data:
            if (_msg_buff_len >= _parsed_msg.msg.total_len - 1) {
                _parsed_msg.state = ParseState::Waiting_For_CRC;
            }
            break;

        case ParseState::Waiting_For_CRC: {
            _parsed_msg.msg.crc = b;
            const uint8_t expected_crc = (uint8_t)crc_calculate(_msg_buff, _parsed_msg.msg.total_len - 1);
            if (expected_crc == _parsed_msg.msg.crc) {
                // successfully received a message, do something with it
                _parsed_msg.msg.data = &_msg_buff[5];
                process_packet();
            } else {
                debug("crc expected:%x got:%x data: %x %x %x %x %x %x %x", (unsigned)expected_crc, (unsigned)_parsed_msg.msg.crc,
                _msg_buff[0],_msg_buff[1],_msg_buff[2],_msg_buff[3],_msg_buff[4],_msg_buff[5],_msg_buff[6]);
            }
            reset_parser = true;
            break;
            }
        }

        // handle reset of parser
        if (reset_parser) {
            _msg_buff_len = 0;
            _parsed_msg.state = ParseState::Waiting_For_Header;
            reset_parser = false;
        }
    }
}

// process successfully decoded packets held in the _parsed_msg structure
void AP_Mount_Tusuav::process_packet()
{
    switch ((GimbalCmd)_parsed_msg.msg.cmd) {
    case GimbalCmd::MasCmd_Gimbal:{
        switch ((GimbalSubCmd)_parsed_msg.msg.sub_cmd){
        case GimbalSubCmd::DebugSubCmd_InnerFramePIDPara:{
            
            break;
        }

        case GimbalSubCmd::GimbalSubCmd_GimbalAngle:{
            MessageGimbalAngle msg;

            if (_parsed_msg.msg.data_len >= sizeof(msg)) {
                msg = *reinterpret_cast<const MessageGimbalAngle*>(_parsed_msg.msg.data);                
                _last_current_angle_rad_ms = AP_HAL::millis();
                _current_angle_rad.x = radians(msg.gimbal_angle[1]); // roll angle in rad
                _current_angle_rad.z = radians(msg.gimbal_angle[2]); // yaw angle in rad
                _current_angle_rad.y = radians(msg.gimbal_angle[0]); // pitch angle in rad
                debug("r:%4.1f p:%4.1f y:%4.1f", (double)degrees(_current_angle_rad.x), (double)degrees(_current_angle_rad.y), (double)degrees(_current_angle_rad.z));

                // get recording status
                const bool recording = msg.status.bits.recording;
                if (recording != _recording) {
                    _recording = recording;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,  "%s recording %s", send_text_prefix, _recording ? "ON" : "OFF");
                }

                // do something when recieve tracking command 
                TrackingStatus tracking_status;
                if (msg.status.bits.is_tracking) {
                    switch(msg.quality){
                    case MessageGimbalAngle::QualityEnum::Track_Quality_None:{
                        tracking_status = TrackingStatus::Lost;
                        break;
                    }
                    case MessageGimbalAngle::QualityEnum::Track_Quality_Weak:{
                        tracking_status = TrackingStatus::Searching;
                        break;
                    }
                    case MessageGimbalAngle::QualityEnum::Track_Quality_Normal:{
                        tracking_status = TrackingStatus::Tracking;
                        break;
                    }
                    default:{
                        tracking_status = TrackingStatus::Lost;
                        break;
                    }
                    }
                } else {
                    tracking_status = TrackingStatus::Stopped;
                }
                // tracking status push to GCS
                if (tracking_status != _last_tracking_status) {
                    _last_tracking_status = tracking_status;
                    switch (tracking_status) {
                    case TrackingStatus::Stopped:
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s tracking OFF", send_text_prefix);
                        break;
                    case TrackingStatus::Searching:
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s tracking searching", send_text_prefix);
                        break;
                    case TrackingStatus::Tracking:
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s tracking ON", send_text_prefix);
                        break;
                    case TrackingStatus::Lost:
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s tracking Lost", send_text_prefix);
                        break;
                    }
                }
            }
            break;
        }

        case GimbalSubCmd::GimbalSubCmd_LaserDistance:{
            // recieve laser rangefinder value in decimeters and save the value in meters
            const uint16_t temp = (uint16_t)_parsed_msg.msg.data[1] << 8 | _parsed_msg.msg.data[0];
            _rangefinder_dist_m = temp * 0.1f;
            break;
        }

        case GimbalSubCmd::GimbalSubCmd_GimbalSn:{
            // gimbal model, length is over 14 bytes and less than 18 bytes
            if(_parsed_msg.msg.data_len >= 14){
                _got_model_name = true;
                if(_parsed_msg.msg.data_len > 18){
                    _parsed_msg.msg.data_len = 18;
                }
                
                strncpy((char *)_model_sn, (const char *)_parsed_msg.msg.data, MIN(ARRAY_SIZE(_model_sn), _parsed_msg.msg.data_len));
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s SN: %s", send_text_prefix, (const char*)_model_sn);
            }
            break;
        }

        default:
            break;           
        }
        break;
    }

    case GimbalCmd::MasCmd_Device: {
        switch ((DeviceSubCmd)_parsed_msg.msg.sub_cmd) {
        case DeviceSubCmd::Firmware_Info: {
            if (_parsed_msg.msg.data_len >= 10) { 
                    _got_firmware_version = true;

                // print both app&boot version and compiling time in UTC+8
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s AppVersion: %d.%d.%d %02d-%02d-%02d %02d:%02d", send_text_prefix, 
                        (unsigned char)_parsed_msg.msg.data[0],  
                        (unsigned char)_parsed_msg.msg.data[1],  
                        (unsigned char)_parsed_msg.msg.data[2],  
                        (unsigned char)_parsed_msg.msg.data[3],  
                        (unsigned char)_parsed_msg.msg.data[4],  
                        (unsigned char)_parsed_msg.msg.data[5], 
                        (unsigned char)_parsed_msg.msg.data[6],
                        (unsigned char)_parsed_msg.msg.data[7]); 
                            
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s BootVersion: %d.%d.%d %02d-%02d-%02d %02d:%02d", send_text_prefix,                                    
                        (unsigned char)_parsed_msg.msg.data[8],
                        (unsigned char)_parsed_msg.msg.data[9],
                        (unsigned char)_parsed_msg.msg.data[10],
                        (unsigned char)_parsed_msg.msg.data[11],
                        (unsigned char)_parsed_msg.msg.data[12],
                        (unsigned char)_parsed_msg.msg.data[13],
                        (unsigned char)_parsed_msg.msg.data[14], 
                        (unsigned char)_parsed_msg.msg.data[15]);
                    break;
            }
            break;
        }
        default:
            break;
        }
        break;
    }

    case GimbalCmd::MasCmd_Load:{

        break;
    }

    default:
        break;
    }
}

// send packet to gimbal.  total_len includes everything including the header '0x5A' and crc
// returns true on success, false if outgoing serial buffer is full or other errors
bool AP_Mount_Tusuav::send_packet(const struct FrameType &frame)
{
    if (!_initialised) {
        return false;
    }

    if (frame.data == NULL && frame.data_len > 0) {
        return false;
    }

    AP_Mount_Tusuav::FrameType modifiable_frame = frame;
    modifiable_frame.total_len = modifiable_frame.data_len + AP_MOUNT_TUSUAV_PACKETLEN_MIN;
    // calculate and sanity check packet size
    if (frame.total_len >= AP_MOUNT_TUSUAV_PACKETLEN_MAX) {
        debug("send_packet data buff too large");
        return false;
    }

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < frame.total_len) {
        debug("tx space too low (%u < %u)", (unsigned)_uart->txspace(), (unsigned)frame.total_len);
        return false;
    }

    // buffer for holding outgoing packet
    uint8_t send_buff[frame.total_len];
    uint8_t send_buff_ofs = 0;

    // packet header length cmd and sub-cmd
    send_buff[send_buff_ofs++] = AP_MOUNT_TUSUAV_HEADER1;
    send_buff[send_buff_ofs++] = frame.total_len;
    send_buff[send_buff_ofs++] = frame.flag.all;
    send_buff[send_buff_ofs++] = frame.cmd;
    send_buff[send_buff_ofs++] = frame.sub_cmd;

    // data
    if (frame.data_len > 0) {
        auto* data_ptr = reinterpret_cast<const uint8_t*>(frame.data);
        for (size_t i = 0; i < frame.data_len; ++i) {
            send_buff[send_buff_ofs + i] = data_ptr[i];
        }
        send_buff_ofs += frame.data_len;
    }

    // crc
    send_buff[send_buff_ofs] = crc_calculate(send_buff, send_buff_ofs);
    send_buff_ofs++;
    // write packet to serial port
    _uart->write(send_buff, send_buff_ofs);   

    return true;
}

// send handshake, gimbal will respond with T1_F1_B1_D1 paket that includes current angles
void AP_Mount_Tusuav::send_time_sync()
{
    // get current location
    Location loc;
    int32_t alt_amsl_cm = 0;
    if (!AP::ahrs().get_location(loc) || !loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm)) {
        return;      
    }

    uint16_t year, ms;
    uint8_t month, day, hour, min, sec;
#if AP_RTC_ENABLED
    // get date and time
 
    if (!AP::rtc().get_date_and_time_utc(year, month, day, hour, min, sec, ms)) {
        year = month = day = hour = min = sec = ms = 0;
    }

#endif
    debug("tus time sync :%d %d %d", year, month,day);

    // fill in packet
    const MessageTimeSync msg_sync{
        .time = {
            .year = static_cast<uint8_t>(year-2000),
            .month = month,
            .day = day,
            .hour = hour,
            .min = min,
            .sec = sec
        },
        .loc = {
            .lat = loc.lat,
            .lng = loc.lng,
            .altitude_cm = alt_amsl_cm
        }
    };

    const FrameType frame_to_send{
        .flag={1},
        data_len : sizeof(msg_sync),
        cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
        sub_cmd : static_cast<uint8_t>(GimbalSubCmd::GimbalSubCmd_TimeSynchronize),
        data : (uint8_t*)&msg_sync
    };

    // send packet to gimbal
    send_packet(frame_to_send);
}


// send request for gimbal angle and status, gimbal will respond euler angle and frame angle after recieve this message
bool AP_Mount_Tusuav::request_tracking_status()
{
    // fill in packet
    const FrameType frame_to_send{
        .flag={0x28},
        data_len : 0,
        cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
        sub_cmd : static_cast<uint8_t>(GimbalSubCmd::GimbalSubCmd_GimbalAngle),
        data : NULL
    };
    
    // send packet to gimbal
    return send_packet(frame_to_send);
}

// send request for version, gimbal will respond versions after recieve this message
bool AP_Mount_Tusuav::request_versions()
{
    // fill in packet
   const FrameType frame_to_send{
        .flag={0x28},
        data_len : 0,
        cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Device),
        sub_cmd : static_cast<uint8_t>(DeviceSubCmd::Firmware_Info),
        data : NULL
    };
    
    // send packet to gimbal
    return send_packet(frame_to_send);
}

// send request for models, gimbal will respond models after recieve this message
bool AP_Mount_Tusuav::request_models()
{
    // fill in packet
    const FrameType frame_to_send{
        .flag={0x28},
        data_len : 0,
        cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
        sub_cmd : static_cast<uint8_t>(GimbalSubCmd::GimbalSubCmd_GimbalSn),
        data : NULL
    };
    
    // send packet to gimbal
    return send_packet(frame_to_send);
}


// set gimbal's lock vs follow mode
// lock should be true if gimbal should maintain an earth-frame target
// lock is false to follow / maintain a body-frame target
bool AP_Mount_Tusuav::set_lock(bool lock)
{
    // do not send if lock mode has already been sent recently
    if (_last_lock == lock) {
        return true;
    }

    PodCtrlUnion gimbal_ctrl_settings;
    gimbal_ctrl_settings.all = _gimbal_ctrl_settings.all;

    gimbal_ctrl_settings.bits.lock_ef = lock;

    // fill in packet
    const FrameType frame_to_send{
        .flag={0x00},
        data_len : sizeof(gimbal_ctrl_settings),
        cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
        sub_cmd : static_cast<uint8_t>(GimbalSubCmd::GimbalSubCmd_PodCtrlReg),
        data : &gimbal_ctrl_settings.all
    };
    
    // send packet to gimbal
    return send_packet(frame_to_send);
}

// send target pitch and yaw angles to gimbal
// fill the settings for choosing an earth-frame / body-frame before calling send_target_angles_rates function
bool AP_Mount_Tusuav::send_target_angles_rates(MessageGimbalAngleRates mode)
{
    // gimbal does not support lock in angle control mode
    if (!set_lock(false)) {
        return false;
    }

    // fill in packet
    const FrameType frame_to_send{
        .flag={0},
        data_len : sizeof(mode),
        cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
        sub_cmd : static_cast<uint8_t>(GimbalSubCmd::GimbalSubCmd_PlaneControlPara),
        data : (uint8_t*)&mode
    };

    // send targets to gimbal
    return send_packet(frame_to_send);
}

// send reference of the joystick rocker for moving the gimbal or tracking window
// when gimbal runs in tracking mode this command will move the tracking window for selecting the target
// when in other mode(except angle mode) this command will set the rates of the yaw axis and pitch axis(±500 map to max/min rate speed)
bool AP_Mount_Tusuav::send_stick_pitch_yaw(int16_t pitch,int16_t yaw)
{
    // gimbal does not support lock in angle control mode
    if (!set_lock(false)) {
        return false;
    }

    if (pitch > 500) {
		pitch = 500;
	}
	else if (pitch < -500) {
		pitch = -500;
	}
    if (yaw > 500) {
    	yaw = 500;
	}
	else if (yaw < -500) {
		yaw = -500;
	}

    const int16_t value[2] = {pitch,yaw};
    // fill in packet
    const FrameType frame_to_send{
        .flag={0},
        data_len : sizeof(value),
        cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
        sub_cmd : static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_stick_control),
        data : (uint8_t*)&value[0]
    };
    debug("sticking %d %d", value[0], value[1]);
    // send targets to gimbal
    return send_packet(frame_to_send);
}

// send vehicle attitude and position to gimbal
bool AP_Mount_Tusuav::send_m_ahrs()
{
    // get current location
    Location loc;
    int32_t alt_amsl_cm = 0;
    if (!AP::ahrs().get_location(loc) || !loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm)) {
        debug("tus ahrs fault:%d %d %d",(int)loc.lat,(int)loc.lng,(int)loc.alt); 

    }

    // get vehicle velocity in m/s in earth frame
    Vector3f vel_NED;
    IGNORE_RETURN(AP::ahrs().get_velocity_NED(vel_NED));
    Vector3f acc_NED;
    acc_NED = AP::ahrs().get_accel_ef();
    Vector3f gyro_NED;
    gyro_NED = AP::ahrs().get_gyro();

// _gps_update_count will increase and send to gimbal if gps fixed 2D or better
    #if AP_GPS_ENABLED
    if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D){
        uint32_t gps_last_time = AP::gps().last_message_time_ms();
        if (_gps_last_update_ms != gps_last_time) {
            _gps_last_update_ms = gps_last_time;
            if (++_gps_update_count >= 255) {
                _gps_update_count = 0;
            }
        }
    }
    else{
        _gps_update_count = 0;
    }
    #else
    _gps_update_count = 0;
    #endif

 // get vehicle yaw in the range 0 to 360
    MessageAhrs::FcStatus fc_status;

    if (AP_Param::check_frame_type(AP_PARAM_FRAME_COPTER)) {
        fc_status.bits.aircraft_type = static_cast<uint8_t>(AircraftType::Aircraft_Copter);
    } else {
        fc_status.bits.aircraft_type = static_cast<uint8_t>(AircraftType::Aircraft_Vtol);
    }
    
    float rel_height;
    AP::ahrs().get_relative_position_D_home(rel_height);

    // fill in packet
    MessageAhrs msg_ahrs = {
        .gps_update_flag = _gps_update_count,
        .fc_status = {fc_status.all},
        .fc_gyro = {gyro_NED.x, gyro_NED.y, gyro_NED.z},
        .fc_acc = {acc_NED.x,acc_NED.y,acc_NED.z},
        .vel = {vel_NED.x, vel_NED.y, vel_NED.z},
        .euler = {degrees(AP::ahrs().get_pitch()), degrees(AP::ahrs().get_roll()), degrees(AP::ahrs().get_yaw())},
        .relative_height = rel_height,
        .altitude = alt_amsl_cm*0.01,
        .longitude = loc.lng*1e-7,
        .latitude = loc.lat*1e-7,
    };

    const FrameType frame_to_send{
        .flag={0},
        data_len : sizeof(msg_ahrs),
        cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
        sub_cmd : static_cast<uint8_t>(GimbalSubCmd::GimbalSubCmd_PlaneSensor_Attitude),
        data : (uint8_t*)&msg_ahrs
    };

    
    // send packet to gimbal
    return send_packet(frame_to_send);
}

// take a picture.  returns true on success
bool AP_Mount_Tusuav::take_picture()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    RecordPhotoSettings settings = {.bits={1,RecordingCmd::No_Action}};
    const FrameType frame_to_send{
        .flag={0},
        .data_len = sizeof(settings),
        .cmd = static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
        .sub_cmd = static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_record_switch),
        .data = (uint8_t*)&settings
    };
    return send_packet(frame_to_send);
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_Tusuav::record_video(bool start_recording)
{
    debug("recording %d", _initialised);
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }
    RecordPhotoSettings settings = {.bits={0,RecordingCmd::No_Action}};

    if (start_recording) {
        settings.bits.record = RecordingCmd::Start;
    } else {
        settings.bits.record = RecordingCmd::Stop;
    }

    const FrameType frame_to_send{
    .flag={0},
    data_len : sizeof(settings),
    cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
    sub_cmd : static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_record_switch),
    data : (uint8_t*)&settings
    };

    // send packet to gimbal
    return send_packet(frame_to_send);
}

// set zoom specified as a rate or percentage
bool AP_Mount_Tusuav::set_zoom(ZoomType zoom_type, float zoom_value)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    const int32_t int_zoom_value = static_cast<int32_t>(zoom_value);
    int16_t speed = AP_MOUNT_TUSUAV_ZOOM_SPEED;
    
    switch (zoom_type) {
        case ZoomType::RATE:
            if (int_zoom_value < 0) {
                speed = -speed;
            } else if (int_zoom_value == 0) {
                speed = 0;
            }
            {
                _zoom_times = int_zoom_value;
                const FrameType frame_to_send{
                    .flag = {0},
                    .data_len = sizeof(speed),
                    .cmd = static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
                    .sub_cmd = static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_zoomspeed_set),
                    .data = reinterpret_cast<uint8_t*>(&speed)
                };
                return send_packet(frame_to_send);
            }
            break;
        
        case ZoomType::PCT:
            {
                int16_t percentage = linear_interpolate(10, AP_MOUNT_TUSUAV_ZOOM_MAX * 10, zoom_value, 0, 100);
                const FrameType frame_to_send{
                    .flag = {0},
                    .data_len = sizeof(percentage),
                    .cmd = static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
                    .sub_cmd = static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_zoomvalue_set),
                    .data = reinterpret_cast<uint8_t*>(&percentage)
                };
                return send_packet(frame_to_send);
            }
            break;
        
        default:
            // Handle any other unanticipated cases if needed
            return false;
    }

    return false;
}

// set tracking to none, point or rectangle (see TrackingType enum)
bool AP_Mount_Tusuav::set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    FrameType frame_to_send; // Declare the variable once and reuse it.

    switch (tracking_type) {
    case TrackingType::TRK_NONE: {
        bool disable_track = false;
        frame_to_send = {
            .flag = {0},
            .data_len = sizeof(disable_track),
            .cmd = static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
            .sub_cmd = static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_track_switch),
            .data = (uint8_t*)&disable_track
        };
        // send packet to gimbal
        return send_packet(frame_to_send);
    }
    case TrackingType::TRK_POINT: {
        const uint16_t pos[2] = {uint16_t(p1.x * 10000), uint16_t(p1.y * 10000)};
        bool start_track = true;
        if (pos[0] == 5000 && pos[1] == 5000) {
            // set deadtime over 500ms to avoid trigger tracking mistake
            if (AP_HAL::millis() - _last_tracking_ms < 500) {
                return 0;
            }
            frame_to_send = {
                .flag = {0},
                .data_len = sizeof(start_track),
                .cmd = static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
                .sub_cmd = static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_track_switch),
                .data = (uint8_t*)&start_track
            };
            _last_tracking_ms = AP_HAL::millis();
        } else {
            frame_to_send = {
                .flag = {0},
                .data_len = sizeof(pos),
                .cmd = static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
                .sub_cmd = static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_trackpos_set),
                .data = (uint8_t*)pos
            };
        }
        return send_packet(frame_to_send);
    }
    case TrackingType::TRK_RECTANGLE: {
        const uint8_t track_window[2] = {uint8_t((p2.x - p1.x) * 255), uint8_t((p2.y - p1.y) * 255)};
        frame_to_send = {
            .flag = {0},
            .data_len = sizeof(track_window),
            .cmd = static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
            .sub_cmd = static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_trackbox_set),
            .data = (uint8_t*)track_window
        };
        send_packet(frame_to_send);

        const uint16_t pos[2] = {uint16_t((p2.x - p1.x) * 10000 / 2), uint16_t((p2.y - p1.y) * 10000 / 2)};
        frame_to_send = {
            .flag = {0},
            .data_len = sizeof(pos),
            .cmd = static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
            .sub_cmd = static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_trackpos_set),
            .data = (uint8_t*)pos
        };
        return send_packet(frame_to_send);
    }
    default: {
        return false;
    }
    }
}

// set camera lens as a value from 0 to 3
bool AP_Mount_Tusuav::set_lens(uint8_t lens) 
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    if (lens > (uint8_t)CameraSource::PIP_IR) {
        return false;
    }

    MessageCameraSource camera = {
        .color = _IR_color,
        .source = CameraSource(lens)
    };
    
    const FrameType frame_to_send{
        .flag={0},
        data_len : sizeof(camera),
        cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
        sub_cmd : static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_infrared_display),
        data : (uint8_t*)&camera
    };
    return send_packet(frame_to_send);
    
}

// set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
// primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
bool AP_Mount_Tusuav::set_camera_source(uint8_t primary_source, uint8_t secondary_source)
{
    // maps primary and secondary source to tusuav image sensor
    CameraSource source;
    switch (primary_source) {
    case 0: // Default (RGB)
        FALLTHROUGH;
    case 1: // RGB
        switch (secondary_source) {
        case 0: // RGB + Default (None)
            source = CameraSource::EO;
            break;
        case 2: // PIP RGB+IR
            source = CameraSource::PIP_EO;
            break;
        default:
            return false;
        }
        break;
    case 2: // IR
        switch (secondary_source) {
        case 0: // IR + Default (None)
            source = CameraSource::IR;
            break;
        case 1: // PIP IR+RGB
            source = CameraSource::PIP_IR;
            break;
        default:
            return false;
        }
        break;
    default:
        return false;
    }

    MessageCameraSource camera = {
        .color = _IR_color,
        .source = source
    };
    
    const FrameType frame_to_send{
        .flag={0},
        data_len : sizeof(camera),
        cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
        sub_cmd : static_cast<uint8_t>(GimbalSubCmd::gimbal_single_cmd_infrared_display),
        data : (uint8_t*)&camera
    };

    // send desired image type to camera
    return send_packet(frame_to_send);
}

// send camera information message to GCS
void AP_Mount_Tusuav::send_camera_information(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
        
    }

    static const uint8_t vendor_name[32] = "Tusuav";
    uint8_t model_name[32] {};
    if (_got_model_name) {
        strncpy((char *)model_name, (const char*)_model_sn, MIN(sizeof(model_name), sizeof(_model_sn)));
    }
    const char cam_definition_uri[140] {};

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
        _firmware_version,      // firmware version uint32_t
        NaNf,                   // sensor_size_h float (mm)
        NaNf,                   // sensor_size_v float (mm)
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
void AP_Mount_Tusuav::send_camera_settings(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // convert zoom times (e.g. 1x ~ 20x) to target zoom level (e.g. 0 to 100)
    const float zoom_level = linear_interpolate(0, 100, _zoom_times, 1, AP_MOUNT_TUSUAV_ZOOM_MAX);

    // send CAMERA_SETTINGS message
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        _recording ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE, // camera mode (0:image, 1:video, 2:image survey)
        zoom_level,         // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaNf);               // focusLevel float, percentage from 0 to 100, NaN if unknown
}

// get rangefinder distance.  returns true on success
bool AP_Mount_Tusuav::get_rangefinder_distance(float& distance_m) const
{
    // if not healthy or zero distance return false
    // healthy() checks attitude timeout which is in same message as rangefinder distance
    if (!healthy()) {
        return false;
    }

    distance_m = _rangefinder_dist_m;
    return true;
}

// enable/disable rangefinder.  Returns true on success
bool AP_Mount_Tusuav::set_rangefinder_enable(bool enable)
{
    // fill in packet
    LRFSettings cmd = LRFSettings::RangeFinder_Off;

    if (enable) {
        cmd = LRFSettings::RangeFinder_Continuously_Measure;
    }


    const FrameType frame_to_send{
        .flag={0},
        data_len : sizeof(cmd),
        cmd : static_cast<uint8_t>(GimbalCmd::MasCmd_Gimbal),
        sub_cmd : static_cast<uint8_t>(GimbalSubCmd::GimbalSubcmd_LaserState),
        data : (uint8_t*)&cmd
    };
    
    // send packet to gimbal
    return send_packet(frame_to_send);
}

#endif // HAL_MOUNT_TUSUAV_ENABLED
