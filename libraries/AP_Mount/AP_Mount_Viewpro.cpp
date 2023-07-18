#include "AP_Mount_Viewpro.h"

#if HAL_MOUNT_VIEWPRO_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RTC/AP_RTC.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_VIEWPRO_HEADER1       0x55     // first header byte
#define AP_MOUNT_VIEWPRO_HEADER2       0xAA     // second header byte
#define AP_MOUNT_VIEWPRO_HEADER3       0xDC     // third header byte
#define AP_MOUNT_VIEWPRO_PACKETLEN_MIN 5        // min number of bytes in a packet (3 header bytes, length, crc)
#define AP_MOUNT_VIEWPRO_DATALEN_MAX   (AP_MOUNT_VIEWPRO_PACKETLEN_MAX-AP_MOUNT_VIEWPRO_PACKETLEN_MIN) // max bytes for data portion of packet
#define AP_MOUNT_VIEWPRO_HEALTH_TIMEOUT_MS 1000 // state will become unhealthy if no attitude is received within this timeout
#define AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS 100 // resend angle or rate targets to gimbal at this interval
#define AP_MOUNT_VIEWPRO_ZOOM_SPEED     0x07    // hard-coded zoom speed (fast)
#define AP_MOUNT_VIEWPRO_ZOOM_MAX       20      // hard-coded absolute zoom times max
#define AP_MOUNT_VIEWPRO_DEG_TO_OUTPUT  (65536.0 / 360.0)   // scalar to convert degrees to the viewpro angle scaling
#define AP_MOUNT_VIEWPRO_OUTPUT_TO_DEG  (360.0 / 65536.0)   // scalar to convert viewpro angle scaling to degrees

#define AP_MOUNT_VIEWPRO_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_VIEWPRO_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Viewpro: " fmt, ## args); } } while (0)

const char* AP_Mount_Viewpro::send_text_prefix = "Viewpro:";

// init - performs any required initialisation for this instance
void AP_Mount_Viewpro::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Gimbal, 0);
    if (_uart != nullptr) {
        _initialised = true;
    }

    AP_Mount_Backend::init();
}

// update mount position - should be called periodically
void AP_Mount_Viewpro::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // below here we sent angle or rate targets
    // throttle sends of target angles or rates
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_update_ms < AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS) {
        return;
    }
    _last_update_ms = now_ms;

    // reading incoming packets from gimbal
    read_incoming_packets();

    // send handshake
    send_handshake();

    // send vehicle attitude and position
    send_m_ahrs();

    // if tracking is active we do not send new targets to the gimbal
    if (_last_tracking_status == TrackingStatus::SEARCHING || _last_tracking_status == TrackingStatus::TRACKING) {
        return;
    }

    // update based on mount mode
    switch (get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &angle_bf_target = _params.retract_angles.get();
            send_target_angles(ToRad(angle_bf_target.y), ToRad(angle_bf_target.z), false);
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &angle_bf_target = _params.neutral_angles.get();
            send_target_angles(ToRad(angle_bf_target.y), ToRad(angle_bf_target.z), false);
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            switch (mavt_target.target_type) {
            case MountTargetType::ANGLE:
                send_target_angles(mavt_target.angle_rad.pitch, mavt_target.angle_rad.yaw, mavt_target.angle_rad.yaw_is_ef);
                break;
            case MountTargetType::RATE:
                send_target_rates(mavt_target.rate_rads.pitch, mavt_target.rate_rads.yaw, mavt_target.rate_rads.yaw_is_ef);
                break;
            }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's rc inputs
            MountTarget rc_target {};
            if (get_rc_rate_target(rc_target)) {
                send_target_rates(rc_target.pitch, rc_target.yaw, rc_target.yaw_is_ef);
            } else if (get_rc_angle_target(rc_target)) {
                send_target_angles(rc_target.pitch, rc_target.yaw, rc_target.yaw_is_ef);
            }
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT: {
            MountTarget angle_target_rad {};
            if (get_angle_target_to_roi(angle_target_rad)) {
                send_target_angles(angle_target_rad.pitch, angle_target_rad.yaw, angle_target_rad.yaw_is_ef);
            }
            break;
        }

        case MAV_MOUNT_MODE_HOME_LOCATION: {
            MountTarget angle_target_rad {};
            if (get_angle_target_to_home(angle_target_rad)) {
                send_target_angles(angle_target_rad.pitch, angle_target_rad.yaw, angle_target_rad.yaw_is_ef);
            }
            break;
        }

        case MAV_MOUNT_MODE_SYSID_TARGET:{
            MountTarget angle_target_rad {};
            if (get_angle_target_to_sysid(angle_target_rad)) {
                send_target_angles(angle_target_rad.pitch, angle_target_rad.yaw, angle_target_rad.yaw_is_ef);
            }
            break;
        }

        default:
            // we do not know this mode so raise internal error
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }
}

// return true if healthy
bool AP_Mount_Viewpro::healthy() const
{
    // unhealthy until gimbal has been found and replied with firmware version info
    if (!_initialised) {
        return false;
    }

    // unhealthy if attitude information NOT received recently
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_current_angle_rad_ms > AP_MOUNT_VIEWPRO_HEALTH_TIMEOUT_MS) {
        return false;
    }

    // if we get this far return healthy
    return true;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Viewpro::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(_current_angle_rad.x, _current_angle_rad.y, _current_angle_rad.z);
    return true;
}

// reading incoming packets from gimbal and confirm they are of the correct format
// results are held in the _parsed_msg structure
void AP_Mount_Viewpro::read_incoming_packets()
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
        if (_msg_buff_len > AP_MOUNT_VIEWPRO_PACKETLEN_MAX) {
            reset_parser = true;
            debug("vp buff full s:%u len:%u", (unsigned)_parsed_msg.state, (unsigned)_msg_buff_len);
        }

        // process byte depending upon current state
        switch (_parsed_msg.state) {

        case ParseState::WAITING_FOR_HEADER1:
            if (b == AP_MOUNT_VIEWPRO_HEADER1) {
                // throw away byte
                _msg_buff_len = 0;
                _parsed_msg.state = ParseState::WAITING_FOR_HEADER2;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_HEADER2:
            if (b == AP_MOUNT_VIEWPRO_HEADER2) {
                // throw away byte
                _msg_buff_len = 0;
                _parsed_msg.state = ParseState::WAITING_FOR_HEADER3;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_HEADER3:
            if (b == AP_MOUNT_VIEWPRO_HEADER3) {
                // throw away byte
                _msg_buff_len = 0;
                _parsed_msg.state = ParseState::WAITING_FOR_LENGTH;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_LENGTH:
            // length held in bits 0 ~ 5.  length includes this length byte, frame id and final crc
            // ignore frame counter held in bits 6~7
            _parsed_msg.data_len = b & 0x3F;
            _parsed_msg.state = ParseState::WAITING_FOR_FRAMEID;
            break;

        case ParseState::WAITING_FOR_FRAMEID:
            _parsed_msg.frame_id = b;
            _parsed_msg.data_bytes_received = 0;
            _parsed_msg.state = ParseState::WAITING_FOR_DATA;
            break;

        case ParseState::WAITING_FOR_DATA:
            _parsed_msg.data_bytes_received++;
            // check if we have received all data bytes.  subtract 3 to remove length byte, frame id and final crc
            if (_parsed_msg.data_bytes_received >= _parsed_msg.data_len - 3) {
                _parsed_msg.state = ParseState::WAITING_FOR_CRC;
            }
            break;

        case ParseState::WAITING_FOR_CRC: {
            _parsed_msg.crc = b;
            const uint16_t expected_crc = calc_crc(_msg_buff, _msg_buff_len-1);
            if (expected_crc == _parsed_msg.crc) {
                // successfully received a message, do something with it
                process_packet();
            } else {
                debug("crc expected:%x got:%x", (unsigned)expected_crc, (unsigned)_parsed_msg.crc);
            }
            reset_parser = true;
            break;
            }
        }

        // handle reset of parser
        if (reset_parser) {
            _msg_buff_len = 0;
            _parsed_msg.state = ParseState::WAITING_FOR_HEADER1;
            reset_parser = false;
        }
    }
}

// process successfully decoded packets held in the _parsed_msg structure
void AP_Mount_Viewpro::process_packet()
{
    // process packet depending upon frame id
    switch ((FrameId)_parsed_msg.frame_id) {

    case FrameId::HANDSHAKE:
        break;

    case FrameId::T1_F1_B1_D1: {
        // T1 holds target info including target lean angles
        // F1 holds tracker sensor status (which camera, tracking vs lost)
        // B1 section holds actual lean angles
        // D1 section holds camera status including zoom level
        //const int8_t servo_status = (_msg_buff[_msg_buff_data_start+24] & 0xF0) >> 4;
        const TrackingStatus tracking_status = (TrackingStatus)((_msg_buff[_msg_buff_data_start+22] & 0x18) >> 3);
        if (tracking_status != _last_tracking_status) {
            _last_tracking_status = tracking_status;
            switch (tracking_status) {
            case TrackingStatus::STOPPED:
                gcs().send_text(MAV_SEVERITY_INFO, "%s tracking OFF", send_text_prefix);
                break;
            case TrackingStatus::SEARCHING:
                gcs().send_text(MAV_SEVERITY_INFO, "%s tracking searching", send_text_prefix);
                break;
            case TrackingStatus::TRACKING:
                gcs().send_text(MAV_SEVERITY_INFO, "%s tracking ON", send_text_prefix);
                break;
            case TrackingStatus::LOST:
                gcs().send_text(MAV_SEVERITY_INFO, "%s tracking Lost", send_text_prefix);
                break;
            }
        }

        _last_current_angle_rad_ms = AP_HAL::millis();
        _current_angle_rad.x = radians((int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+23] & 0x0F, _msg_buff[_msg_buff_data_start+24]) * (180.0/4095.0) - 90.0);   // roll angle
        _current_angle_rad.z = radians((int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+25], _msg_buff[_msg_buff_data_start+26]) * AP_MOUNT_VIEWPRO_OUTPUT_TO_DEG); // yaw angle
        _current_angle_rad.y = radians((int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+27], _msg_buff[_msg_buff_data_start+28]) * AP_MOUNT_VIEWPRO_OUTPUT_TO_DEG); // pitch angle
        debug("r:%4.1f p:%4.1f y:%4.1f", (double)degrees(_current_angle_rad.x), (double)degrees(_current_angle_rad.y), (double)degrees(_current_angle_rad.z));
        break;
    }

    default:
        debug("Unhandled FrameId:%u", (unsigned)_parsed_msg.frame_id);
        break;
    }
}

// calculate crc of the received message
uint8_t AP_Mount_Viewpro::calc_crc(const uint8_t *buf, uint8_t len) const
{
    uint8_t res = 0;
    for (uint8_t i=0; i<len; i++) {
        res = (res ^ buf[i]) & 0xFF;
    }
    return res;
}

// calculate the length and frame count byte (3rd byte of all messages)
// length is all bytes after the header including CRC
uint8_t AP_Mount_Viewpro::get_length_and_frame_count_byte(uint8_t length)
{
  // increment frame counter
  _last_frame_counter = (_last_frame_counter + 1) & 0x03;
  return ((_last_frame_counter << 6) | (length & 0x3F));
}

// send packet to gimbal.  databuff includes everything after the length-and-frame-counter, does not include crc
// returns true on success, false if outgoing serial buffer is full
bool AP_Mount_Viewpro::send_packet(const uint8_t* databuff, uint8_t databuff_len)
{
    if (!_initialised) {
        return false;
    }

    // calculate and sanity check packet size
    const uint16_t packet_size = AP_MOUNT_VIEWPRO_PACKETLEN_MIN + databuff_len;
    if (packet_size > AP_MOUNT_VIEWPRO_PACKETLEN_MAX) {
        debug("send_packet data buff too large");
        return false;
    }

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < packet_size) {
        debug("tx space too low (%u < %u)", (unsigned)_uart->txspace(), (unsigned)packet_size);
        return false;
    }

    // buffer for holding outgoing packet
    uint8_t send_buff[packet_size];
    uint8_t send_buff_ofs = 0;

    // packet header
    send_buff[send_buff_ofs++] = AP_MOUNT_VIEWPRO_HEADER1;
    send_buff[send_buff_ofs++] = AP_MOUNT_VIEWPRO_HEADER2;
    send_buff[send_buff_ofs++] = AP_MOUNT_VIEWPRO_HEADER3;

    // length and frame counter. length is databuffer length + 2 (1 for length, 1 for crc)
    send_buff[send_buff_ofs++] = get_length_and_frame_count_byte(databuff_len + 2);

    // data
    if (databuff_len != 0) {
        memcpy(&send_buff[send_buff_ofs], databuff, databuff_len);
        send_buff_ofs += databuff_len;
    }

    // crc
    const uint8_t crc = calc_crc(&send_buff[3], send_buff_ofs-3);
    send_buff[send_buff_ofs++] = crc;

    // write packet to serial port
    _uart->write(send_buff, send_buff_ofs);
    

    return true;
}

// send handshake, gimbal will respond with T1_F1_B1_D1 paket that includes current angles
void AP_Mount_Viewpro::send_handshake()
{
    HandshakePacket hs_packet {FrameId::HANDSHAKE, 0};
    send_packet(hs_packet.bytes, sizeof(hs_packet.bytes));
}

// set gimbal's lock vs follow mode
// lock should be true if gimbal should maintain an earth-frame target
// lock is false to follow / maintain a body-frame target
bool AP_Mount_Viewpro::set_lock(bool lock)
{
    // do not send if lock mode has already been sent recently
    if (_last_lock == lock) {
        return true;
    }

    // fill in packet
    A1Packet a1_packet {};
    a1_packet.content.frame_id = FrameId::A1;
    a1_packet.content.servo_status = lock ? ServoStatus::follow_yaw_disable : ServoStatus::follow_yaw;

    // send targets to gimbal
    if (send_packet(a1_packet.bytes, sizeof(a1_packet.bytes))) {
        _last_lock = lock;
        return true;
    }
    return false;
}

// send target pitch and yaw rates to gimbal
// yaw_is_ef should be true if yaw_rads target is an earth frame rate, false if body_frame
bool AP_Mount_Viewpro::send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef)
{
    // set lock value
    if (!set_lock(yaw_is_ef)) {
        return false;
    }

    // scale pitch and yaw to values gimbal understands
    const int16_t pitch_rate_output = -degrees(pitch_rads) * 100.0;
    const int16_t yaw_rate_output = degrees(yaw_rads) * 100.0;

    // fill in packet
    A1Packet a1_packet {};
    a1_packet.content.frame_id = FrameId::A1;
    a1_packet.content.servo_status = ServoStatus::manual_speed_mode;
    a1_packet.content.yaw_be = htobe16(yaw_rate_output);
    a1_packet.content.pitch_be = htobe16(pitch_rate_output);

    // send targets to gimbal
    return send_packet(a1_packet.bytes, sizeof(a1_packet.bytes));
}

// send target pitch and yaw angles to gimbal
// yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
bool AP_Mount_Viewpro::send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef)
{
    // gimbal does not support lock in angle control mode
    if (!set_lock(false)) {
        return false;
    }

    // convert yaw angle to body-frame
    float yaw_bf_rad = yaw_is_ef ? wrap_PI(yaw_rad - AP::ahrs().yaw) : yaw_rad;

    // enforce body-frame yaw angle limits.  If beyond limits always use body-frame control
    const float yaw_bf_min = radians(_params.yaw_angle_min);
    const float yaw_bf_max = radians(_params.yaw_angle_max);
    if (yaw_bf_rad < yaw_bf_min || yaw_bf_rad > yaw_bf_max) {
        yaw_bf_rad = constrain_float(yaw_bf_rad, yaw_bf_min, yaw_bf_max);
        yaw_is_ef = false;
    }

    // scale pitch and yaw to values gimbal understands
    const int16_t pitch_angle_output = -degrees(pitch_rad) * AP_MOUNT_VIEWPRO_DEG_TO_OUTPUT;
    const int16_t yaw_angle_output = degrees(yaw_bf_rad) * AP_MOUNT_VIEWPRO_DEG_TO_OUTPUT;

    // fill in packet
    A1Packet a1_packet {};
    a1_packet.content.frame_id = FrameId::A1;
    a1_packet.content.servo_status = ServoStatus::manual_absolute_angle_mode;
    a1_packet.content.yaw_be = htobe16(yaw_angle_output);
    a1_packet.content.pitch_be = htobe16(pitch_angle_output);

    // send targets to gimbal
    return send_packet(a1_packet.bytes, sizeof(a1_packet.bytes));
}

// send camera command and corresponding value (e.g. zoom speed)
bool AP_Mount_Viewpro::send_camera_command(CameraCommand cmd, uint8_t value)
{
    // fill in packet
    C1Packet c1_packet {};
    c1_packet.content.frame_id = FrameId::C1;

    // fill in 2 bytes containing sensor, zoom speed, operation command and LRF
    // bit0~2: sensor
    // bit3~5: zoom speed
    // bit6~12: operation command no
    // bit13~15: LRF command (unused)
    const uint16_t sensor_id = (uint16_t)ImageSensor::EO1;
    const uint16_t zoom_speed = ((uint16_t)value & 0x07) << 3;
    const uint16_t operation_cmd = ((uint16_t)cmd & 0x7F) << 6;
    c1_packet.content.sensor_zoom_cmd_be =  htobe16(sensor_id | zoom_speed | operation_cmd);

    // send packet to gimbal
    return send_packet(c1_packet.bytes, sizeof(c1_packet.bytes));
}

// send camera command2 and corresponding value (e.g. zoom as absolute value)
bool AP_Mount_Viewpro::send_camera_command2(CameraCommand2 cmd, uint16_t value)
{
    // fill in packet
    C2Packet c2_packet {};
    c2_packet.content.frame_id = FrameId::C2;
    c2_packet.content.cmd = cmd;
    c2_packet.content.value_be = htobe16(value);

    // send packet to gimbal
    return send_packet(c2_packet.bytes, sizeof(c2_packet.bytes));
}

// send tracking command and corresponding value (normally zero)
bool AP_Mount_Viewpro::send_tracking_command(TrackingCommand cmd, uint8_t value)
{
    // fill in packet
    E1Packet e1_packet {};
    e1_packet.content.frame_id = FrameId::E1;
    e1_packet.content.source = TrackingSource::EO1; // hard-coded to only affect RGB sensor for now
    e1_packet.content.cmd = cmd;
    e1_packet.content.param2 = value;               // normally zero

    // send packet to gimbal
    return send_packet(e1_packet.bytes, sizeof(e1_packet.bytes));
}

// send camera command2 and corresponding parameter values
bool AP_Mount_Viewpro::send_tracking_command2(TrackingCommand2 cmd, uint16_t param1, uint16_t param2)
{
    // fill in packet
    E2Packet e2_packet {};
    e2_packet.content.frame_id = FrameId::E2;
    e2_packet.content.cmd = cmd;
    e2_packet.content.param1_be = htobe16(param1);
    e2_packet.content.param2_be = htobe16(param2);

    // send packet to gimbal
    return send_packet(e2_packet.bytes, sizeof(e2_packet.bytes));
}

// send vehicle attitude and position to gimbal
bool AP_Mount_Viewpro::send_m_ahrs()
{
    // get current location
    Location loc;
    int32_t alt_amsl_cm = 0;
    if (!AP::ahrs().get_location(loc) || !loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm)) {
        return false;
    }

    // get date and time
    uint16_t year, ms;
    uint8_t month, day, hour, min, sec;
    if (!AP::rtc().get_date_and_time_utc(year, month, day, hour, min, sec, ms)) {
        year = month = day = hour = min = sec = 0;
    }
    uint16_t date = ((year-2000) & 0x7f) | (((month+1) & 0x0F) << 7) | ((day & 0x0F) << 11);
    uint64_t second_hundredths = (((hour * 60 * 60) + (min * 60) + sec) * 100) + (ms * 0.1);

    // get vehicle velocity in m/s in NED Frame
    Vector3f vel_NED;
    IGNORE_RETURN(AP::ahrs().get_velocity_NED(vel_NED));
    float vel_yaw_deg = degrees(vel_NED.xy().angle());

    // get GPS vdop
    uint16_t gps_vdop = AP::gps().get_vdop();

    // fill in packet
    M_AHRSPacket mahrs_packet{};
    mahrs_packet.content.frame_id = FrameId::M_AHRS;
    mahrs_packet.content.data_type = 0x07;  // Bit0: Attitude, Bit1: GPS, Bit2 Gyro
    mahrs_packet.content.pitch_be = htobe16(-degrees(AP::ahrs().get_pitch()) * AP_MOUNT_VIEWPRO_DEG_TO_OUTPUT); // vehicle pitch angle.  1bit=360deg/65536
    mahrs_packet.content.roll_be = htobe16(degrees(AP::ahrs().get_roll()) * AP_MOUNT_VIEWPRO_DEG_TO_OUTPUT);    // vehicle roll angle.  1bit=360deg/65536
    mahrs_packet.content.yaw_be = htobe16(degrees(AP::ahrs().get_yaw()) * AP_MOUNT_VIEWPRO_DEG_TO_OUTPUT);      // vehicle yaw angle.  1bit=360deg/65536
    mahrs_packet.content.date_be = htobe16(date);           // bit0~6:year, bit7~10:month, bit11~15:day
    mahrs_packet.content.seconds_utc[0] = uint8_t((second_hundredths & 0xFF0000ULL) >> 16); // seconds * 100 MSB.  1bit = 0.01sec
    mahrs_packet.content.seconds_utc[1] = uint8_t((second_hundredths & 0xFF00ULL) >> 8);    // seconds * 100 next MSB.  1bit = 0.01sec
    mahrs_packet.content.seconds_utc[2] = uint8_t(second_hundredths & 0xFFULL);             // seconds * 100 LSB.  1bit = 0.01sec
    mahrs_packet.content.gps_yaw_be = htobe16(vel_yaw_deg * AP_MOUNT_VIEWPRO_DEG_TO_OUTPUT); // GPS yaw
    mahrs_packet.content.position_mark_bitmask = 0x0F;      // bit0:new position, bit1:clock fix calced, bit2:horiz calced, bit3:alt calced
    mahrs_packet.content.latitude_be = htobe32(loc.lat);    // latitude.  1bit = 10e-7
    mahrs_packet.content.longitude_be = htobe32(loc.lng);   // longitude.  1bit = 10e-7
    mahrs_packet.content.height_be = htobe32(alt_amsl_cm * 10); // height.  1bit = 1mm
    mahrs_packet.content.ground_speed_N_be = htobe16(vel_NED.x * 100);  // ground speed in North direction. 1bit = 0.01m/s
    mahrs_packet.content.ground_speed_E_be = htobe16(vel_NED.y * 100);  // ground speed in East direction. 1bit = 0.01m/s
    mahrs_packet.content.vdop_be = htobe16(gps_vdop);       // GPS vdop. 1bit = 0.01
    mahrs_packet.content.ground_speed_D_be = htobe16(vel_NED.z * 100);  // speed downwards. 1bit = 0.01m/s

    // send packet to gimbal
    return send_packet(mahrs_packet.bytes, sizeof(mahrs_packet.bytes));
}

// take a picture.  returns true on success
bool AP_Mount_Viewpro::take_picture()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    return send_camera_command(CameraCommand::TAKE_PICTURE, 0);
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_Viewpro::record_video(bool start_recording)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    if (!send_camera_command(start_recording ? CameraCommand::START_RECORD : CameraCommand::STOP_RECORD, 0)) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "%s recording %s", send_text_prefix, start_recording ? "ON": "OFF");
    _last_record_video = start_recording;
    return true;
}

// set zoom specified as a rate or percentage
bool AP_Mount_Viewpro::set_zoom(ZoomType zoom_type, float zoom_value)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // zoom rate
    if (zoom_type == ZoomType::RATE) {
        CameraCommand zoom_cmd = CameraCommand::STOP_FOCUS_AND_ZOOM;
        if (zoom_value < 0) {
            zoom_cmd = CameraCommand::ZOOM_OUT;
        } else if (zoom_value > 0) {
            zoom_cmd = CameraCommand::ZOOM_IN;
        }
        return send_camera_command(zoom_cmd, AP_MOUNT_VIEWPRO_ZOOM_SPEED);
    }

    // zoom percentage
    if (zoom_type == ZoomType::PCT) {
        // convert zoom percentage (0 ~ 100) to zoom value (0 ~ max zoom * 10)
        return send_camera_command2(CameraCommand2::SET_EO_ZOOM, linear_interpolate(0, AP_MOUNT_VIEWPRO_ZOOM_MAX * 10, zoom_value, 0, 100));
    }

    // unsupported zoom type
    return false;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Mount_Viewpro::set_focus(FocusType focus_type, float focus_value)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return SetFocusResult::FAILED;
    }

    switch (focus_type) {
    case FocusType::RATE: {
        CameraCommand focus_cmd = CameraCommand::STOP_FOCUS_AND_ZOOM;
        if (focus_value < 0) {
            focus_cmd = CameraCommand::FOCUS_MINUS;
        } else if (focus_value > 0) {
            focus_cmd = CameraCommand::FOCUS_PLUS;
        }
        if (!send_camera_command(focus_cmd, 0)) {
            return SetFocusResult::FAILED;
        }
        return SetFocusResult::ACCEPTED;
    }
    case FocusType::PCT:
        // not supported
        return SetFocusResult::INVALID_PARAMETERS;
    case FocusType::AUTO:
        if (!send_camera_command(CameraCommand::AUTO_FOCUS, 0)) {
            return SetFocusResult::FAILED;
        }
        return SetFocusResult::ACCEPTED;
    }

    // unsupported focus type
    return SetFocusResult::INVALID_PARAMETERS;
}

// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
// p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Mount_Viewpro::set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    switch (tracking_type) {
    case TrackingType::TRK_NONE:
        return send_tracking_command(TrackingCommand::STOP, 0);
        break;
    case TrackingType::TRK_POINT: {
        return (send_tracking_command(TrackingCommand::START, 0) &&
                send_tracking_command2(TrackingCommand2::SET_POINT, (p1.x - 0.5) * 960, (p1.y - 0.5) * 540));
        break;
    }
    case TrackingType::TRK_RECTANGLE:
        return (send_tracking_command(TrackingCommand::START, 0) &&
                send_tracking_command2(TrackingCommand2::SET_RECT_TOPLEFT, (p1.x - 0.5) * 960, (p1.y - 0.5) * 540) &&
                send_tracking_command2(TrackingCommand2::SET_RECT_BOTTOMRIGHT, (p2.x - 0.5) * 960, (p2.y - 0.5) * 540));
        break;
    }

    // should never reach here
    return false;
}

// send camera information message to GCS
void AP_Mount_Viewpro::send_camera_information(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    static const uint8_t vendor_name[32] = "Viewpro";
    static uint8_t model_name[32] = "Unknown";
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
        0,                      // firmware version uint32_t
        0,                      // focal_length float (mm)
        0,                      // sensor_size_h float (mm)
        0,                      // sensor_size_v float (mm)
        0,                      // resolution_h uint16_t (pix)
        0,                      // resolution_v uint16_t (pix)
        0,                      // lens_id uint8_t
        flags,                  // flags uint32_t (CAMERA_CAP_FLAGS)
        0,                      // cam_definition_version uint16_t
        cam_definition_uri);    // cam_definition_uri char[140]
}

// send camera settings message to GCS
void AP_Mount_Viewpro::send_camera_settings(mavlink_channel_t chan) const
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
        _last_record_video ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE, // camera mode (0:image, 1:video, 2:image survey)
        NaN,                // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaN);               // focusLevel float, percentage from 0 to 100, NaN if unknown
}

#endif // HAL_MOUNT_VIEWPRO_ENABLED
