#include "AP_Mount_config.h"

#if HAL_MOUNT_XFROBOT_ENABLED

#include "AP_Mount_XFRobot.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>

#define SEND_ATTITUDE_TARGET_MS     1000    // send angle targets to gimbal at least once per second
#define HEALTH_TIMEOUT_MS           1000    // become unhealthy if attitude not received within 1 second
#define RECORD_REQUEST_TIMEOUT_MS   1000    // requests to start/stop recording timeout in 1 second
#define SEND_HEADER1                0xA8    // send packet header1
#define SEND_HEADER2                0xE5    // send packet header2
#define RECV_HEADER1                0x8A    // receive packet header1
#define RECV_HEADER2                0x5E    // receive packet header2
#define RECV_LENGTH_MIN             10      // receive packet minimum length in bytes
#define PROTOCOL_VERSION            0x02    // protocol version

const char* AP_Mount_XFRobot::send_text_prefix = "XFRobot:";

////////////////////////////////////////////////////////
// packet structure from autopilot to gimbal
// byte 0: header1 (0xA8)
// byte 1: header2 (0xE5)
// byte 2~3: length (uint16)
// byte 4: protocol version (0x02)
// below is "main data frame"
// byte 5~6: roll control value
// byte 7~8: pitch control value
// byte 9~10: yaw control value
// byte 11: status, Bit0:INS valid, Bit2:control values valid
// byte 12~13: absolute roll angle of vehicle (int16, -18000 ~ +18000)
// byte 14~15: absolute pitch angle of vehicle (int16, -9000 ~ +9000)
// byte 16~17: absolute yaw angle of vehicle (uint16, 0 ~ 36000)
// byte 18~19: North acceleration of vehicle (int16, cm/s/s)
// byte 20~21: East acceleration of vehicle (int16, cm/s/s)
// byte 22~23: Upward acceleration of vehicle (int16, cm/s/s)
// byte 24~25: North speed of vehicle (int16, decimeter/s)
// byte 26~27: East speed of vehicle (int16, decimeter/s)
// byte 28~29: Upward speed of vehicle (int16, decimeter/s)
// byte 30: request code of sub frame, header of requested sub data frame from GCU (aka camera)
// above is end of main data frame
// byte 31~36: reserved/unused
// below is "sub data frame"
// byte 37: header (0x01)
// byte 38~41: longitude of vehicle (int32, 1E7)
// byte 42~45: latitude of vehicle (int32, 1E7)
// byte 46~49: altitude of vehicle (int32, mm)
// byte 50: number of satellites
// byte 51~54: GNSS microsecond (uint32)
// byte 55~56: GNSS week number (uint16)
// byte 57~60: relative altitude (int32, mm, can be zero if unavailable)
// byte 61~68: reserved/unused
// byte 69: order (uint8)
//     0x00: Null (sent to ensure next command is not ignored)
//     0x01: Calibration
//     0x03: Neutral
//     0x10: Angle control
//     0x11: Head lock
//     0x12: Head follow
//     0x13: Orthoview
//     0x14: Euler angle control
//     0x15: Gaze Geo-coordinates, params: Longitude, Latitude, Altitude (12bytes)
//     0x16: Gaze Geo-target lock
//     0x17: Track, params: 0x01, TT, X0, X0, Y0, Y0, X1, X1, Y1, Y1 (10 bytes)
//     0x1A: Click to aim, params: 0x01, X0, X0, Y0, Y0 (5 bytes)
//     0x1C: FPV
// byte 70 ~ S-3: parameter (variable length)
// byte S-2: CRC high
// byte S-1: CRC low

// packet structure from GCU (aka camera)
// byte 0: header1 (0x8A)
// byte 1: header2 (0x5E)
// byte 2~3: length (uint16)
// byte 4: version
// below is "main data frame"
// byte 5: mode: 0x10:angle control, 0x11:head lock, 0x12: head follow, 0x13:orthoview, 0x14:euler angle control, 0x16:gaze, 0x17:track,0x1C:FPV
// byte 6~7: status, Bit0:tracking success, Bit7:range and target coordinate valid, Bit8:ranging on, Bit9:night vision on, Bit10:lighting on, Bit12:Upward powered on
// byte 8~9: horizontal target (int16, -1000 ~ +1000, rightward is positive)
// byte 10~11: vertical target (int16, -1000 ~ +1000, downward is positive)
// byte 12~13: x-axis angle of camera relative to vehicle (int16, -18000 ~ +18000)
// byte 14~15: y-axis angle of camera relative to vehicle (int16, -18000 ~ +18000)
// byte 16~17: z-axis angle of camera relative to vehicle (int16, -18000 ~ +18000)
// byte 18~19: roll angle of camera (absolute) (int16, -9000 ~ +9000)
// byte 20~21: pitch angle of camera (absolute) (int16, -18000 ~ +18000)
// byte 22~23: yaw angle of camera (absolute) (uint16, 0 ~ 36000)
// byte 24~25: X-axis absolute angular velocity of camera (int16, centi-degrees/s)
// byte 26~27: Y-axis absolute angular velocity of camera (int16, centi-degrees/s)
// byte 28~29: Z-axis absolute angular velocity of camera (int16, centi-degrees/s)
// byte 30~36: reserved/unused
// above is end of "main data frame", below is "sub data frame"
// byte 37: header (0x01)
// byte 38: hardware version (uint8)
// byte 39: firmware version (uint8)
// byte 40: pod code
// byte 41~42: error code, Bit7:hardware error, Bit13: mavlink communication freq anomaly, Bit14:BNSS unpositioned, Bit15:GCU hardware error
// byte 43~46: distance from target (int32, decimeters, -1m or 0m is invalid measurement)
// byte 47~50: longitude of target (int32, 1E7)
// byte 51~54: latitude of target (int32, 1E7)
// byte 55~58: altitude of target (int32, mm)
// byte 59~60: zoom rate of RGB camera (uint16, resolution 0.1x)
// byte 61~62: zoom rate of thermal camera (uint16, resolution 0.1x)
// byte 63: thermal camera status, Bit0:low temp alert, Bit1:high temp alert, Bit3:spot temp measurement on, Bit4:Isotherm on, Bit5:temp alert on, Bit6:area temp on, Bit7:temp available
// byte 64~65: camera status, Bit0~Bit2:pic-in-pic mode, Bit4:recording, Bit11:image auto reverse off, Bit12:OSD displays target coordinate, Bit13:OSD on, Bit14:digital zoom on, Bit15:target detection on
// byte 66: timezone (int8, -12 ~ +12)
// byte 67~68: reserved/unused
// end of "sub data frame"
// byte 69: order
// byte 70 ~ S-3: execution state (variable length)
// byte S-2: CRC high
// byte S-1: CRC low
////////////////////////////////////////////////////////

// update mount position - should be called periodically
void AP_Mount_XFRobot::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // reading incoming packets from gimbal
    read_incoming_packets();

    // check for recording request timeout
    check_recording_timeout();

    AP_Mount_Backend::update_mnt_target();

    // send target angles (which may be derived from other target types)
    send_target_to_gimbal();
}

// return true if healthy
bool AP_Mount_XFRobot::healthy() const
{
    // healthy if initialised and attitude from gimbal has been received within the last second
    return _initialised && (AP_HAL::millis() - attitude_latest.update_ms) < HEALTH_TIMEOUT_MS;
}

// take a picture.  returns true on success
bool AP_Mount_XFRobot::take_picture()
{
    // send command to take picture
    return send_simple_command(FunctionOrder::SHUTTER, 0x01);
}

//set camera lens
bool AP_Mount_XFRobot::set_lens(uint8_t lens)
{
    // lens to camera type mapping table
    static const CameraType cam_type_table[] {
        CameraType::MAIN_ZOOM_SUB_THERMAL,
        CameraType::MAIN_THERMAL_SUB_ZOOM,
        CameraType::MAIN_PIP_ZOOM_SUB_THERMAL,
    };

    // sanity check lens values
    if (lens >= ARRAY_SIZE(cam_type_table)) {
        return false;
    }

    // map lens to camera type and send command
    return send_simple_command(FunctionOrder::PIC_IN_PIC, (uint8_t)cam_type_table[lens]);
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_XFRobot::record_video(bool start_recording)
{
    // reject request if waiting for reply on earlier request
    if (recording.request_ms > 0) {
        return false;
    }

    // send command to start/stop recording video
    if (send_simple_command(FunctionOrder::RECORD_VIDEO, uint8_t(start_recording))) {
        // record request to start or stop recording
        recording.request_ms = AP_HAL::millis();
        recording.request_start = start_recording;
        return true;
    }
    return false;
}

// set zoom specified as a rate or percentage
bool AP_Mount_XFRobot::set_zoom(ZoomType zoom_type, float zoom_value)
{
    // zoom rate
    if (zoom_type == ZoomType::RATE) {
        FunctionOrder zoom_fn = FunctionOrder::ZOOM_STOP;
        if (zoom_value < 0) {
            zoom_fn = FunctionOrder::ZOOM_OUT;
        } else if (zoom_value > 0) {
            zoom_fn = FunctionOrder::ZOOM_IN;
        }
        return send_simple_command(zoom_fn, 0x01);
    }

    // unsupported zoom type
    return false;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Mount_XFRobot::set_focus(FocusType focus_type, float focus_value)
{
    if (focus_type == FocusType::AUTO) {
        // send auto focus command
        return send_simple_command(FunctionOrder::FOCUS, 0x01) ? SetFocusResult::ACCEPTED : SetFocusResult::FAILED;
    }

    // unsupported focus type
    return SetFocusResult::INVALID_PARAMETERS;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_XFRobot::get_attitude_quaternion(Quaternion& att_quat)
{
    // gimbal does not provide attitude so simply return targets
    att_quat.from_euler(radians(attitude_latest.roll_ef_deg), radians(attitude_latest.pitch_ef_deg), radians(attitude_latest.yaw_bf_deg));
    return true;
}

// reading incoming packets from gimbal
void AP_Mount_XFRobot::read_incoming_packets()
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
        msg_buff.bytes[msg_buff_len++] = b;

        // protect against overly long messages
        if (msg_buff_len > AP_MOUNT_XFROBOT_RECV_LENGTH_MAX) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (parser.state) {
        case ParseState::WAITING_FOR_HEADER1:
            if (b == RECV_HEADER1) {
                parser.state = ParseState::WAITING_FOR_HEADER2;
            }
            break;
        case ParseState::WAITING_FOR_HEADER2:
            if (b == RECV_HEADER2) {
                parser.state = ParseState::WAITING_FOR_LENGTH_LOW;
            } else {
                reset_parser = true;
            }
            break;
        case ParseState::WAITING_FOR_LENGTH_LOW:
            parser.len_expected = b;
            parser.state = ParseState::WAITING_FOR_LENGTH_HIGH;
            break;
        case ParseState::WAITING_FOR_LENGTH_HIGH:
            parser.len_expected |= ((uint16_t)b << 8);
            if (parser.len_expected < RECV_LENGTH_MIN || parser.len_expected > AP_MOUNT_XFROBOT_RECV_LENGTH_MAX) {
                reset_parser = true;
            } else {
                parser.state = ParseState::WAITING_FOR_DATA;
            }
            break;
        case ParseState::WAITING_FOR_DATA:
            // check if we have received all data bytes.  the final two bytes are for the CRC
            if (msg_buff_len >= parser.len_expected - 2) {
                parser.state = ParseState::WAITING_FOR_CRC_HIGH;
            }
            break;
        case ParseState::WAITING_FOR_CRC_HIGH:
            parser.crc = (uint16_t)b << 8;
            parser.state = ParseState::WAITING_FOR_CRC_LOW;
            break;
        case ParseState::WAITING_FOR_CRC_LOW:
            parser.crc |= b;
            const uint16_t expected_crc = crc16_ccitt(msg_buff.bytes, msg_buff_len - 2, 0);
            if (expected_crc == parser.crc) {
                // successfully received a message, do something with it
                process_packet();
            }
            reset_parser = true;
            break;
        }

        // handle reset of parser
        if (reset_parser) {
            msg_buff_len = 0;
            parser.state = ParseState::WAITING_FOR_HEADER1;
            reset_parser = false;
        }
    }
}

// process successfully decoded packets held in the msg_buff structure
void AP_Mount_XFRobot::process_packet()
{
    // sanity check size of packet received
    if (msg_buff_len < sizeof(GCUSimpleReply)) {
        return;
    }

    // extract latest attitude
    attitude_latest = {
        .roll_ef_deg = msg_buff.simple_reply.main.roll_abs_cd * 0.01f,
        .pitch_ef_deg = msg_buff.simple_reply.main.pitch_abs_cd * 0.01f,
        .yaw_bf_deg = msg_buff.simple_reply.main.angle_z * 0.01f,
        .update_ms = AP_HAL::millis()
    };

    // display hardware and firmware version
    if (!got_firmware_version) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s hw:%.1f fw:%.1f", send_text_prefix, double(msg_buff.simple_reply.main.hardware_version * 0.1), double(msg_buff.simple_reply.main.firmware_version * 0.1));
        got_firmware_version = true;
    }

    // detect failure to take picture
    if (msg_buff.simple_reply.main.order == FunctionOrder::SHUTTER && msg_buff.simple_reply.param.execution_state == 0x01) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s failed to take picture", send_text_prefix);
    }

    // check for failure to start/stop recording
    if (msg_buff.simple_reply.main.order == FunctionOrder::RECORD_VIDEO) {
        if (msg_buff.simple_reply.param.execution_state == 0x0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s recording %s", send_text_prefix, recording.request_start ? "ON" : "OFF");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s failed to %s recording", send_text_prefix, recording.request_start ? "start" : "stop");
        }
        recording.request_ms = 0;
    }
}

// send_target_angles
void AP_Mount_XFRobot::send_target_angles(const MountAngleTarget& angle_target_rad)
{
    // exit immediately if not initialised or not enough space to send packet
    if (!_initialised || _uart->txspace() < sizeof(SetAttitudePacket)) {
        return;
    }

    // prepare packet to send to gimbal
    SetAttitudePacket set_attitude_packet {};

    // bytes 0~1: headers
    set_attitude_packet.content.main.header1 = SEND_HEADER1;
    set_attitude_packet.content.main.header2 = SEND_HEADER2;

    // byte 2~3: length (uint16)
    set_attitude_packet.content.main.length = htole16(sizeof(SetAttitudePacket));

    // byte 4: protocol version (0x02)
    set_attitude_packet.content.main.version = PROTOCOL_VERSION;

    // byte 5~6: roll control value (int16, -18000 ~ +18000)
    // byte 7~8: pitch control value (int16, -18000 ~ +18000)
    // byte 9~10: yaw control value (int16, -18000 ~ +18000)
    set_attitude_packet.content.main.roll_control = htole16(constrain_int16(degrees(angle_target_rad.roll) * 100, -18000, 18000));
    set_attitude_packet.content.main.pitch_control = htole16(constrain_int16(degrees(angle_target_rad.pitch) * 100, -9000, 9000));
    set_attitude_packet.content.main.yaw_control = htole16(constrain_int16(degrees(angle_target_rad.get_bf_yaw()) * 100, -18000, 18000));

    // byte 11: status, Bit0:INS valid, Bit2:control values valid
    const uint8_t status_ins_bit = AP::ahrs().have_inertial_nav() ? (1 << 0) : 0;
    set_attitude_packet.content.main.status = status_ins_bit | (1 << 2);

    // byte 12~13: absolute roll angle of vehicle (int16, -18000 ~ +18000)
    // byte 14~15: absolute pitch angle of vehicle (int16, -9000 ~ +9000)
    // byte 16~17: absolute yaw angle of vehicle (uint16, 0 ~ 36000)
    set_attitude_packet.content.main.roll_abs = htole16(constrain_int16(AP::ahrs().get_roll_deg() * 100, -18000, 18000));
    set_attitude_packet.content.main.pitch_abs = htole16(constrain_int16(AP::ahrs().get_pitch_deg() * 100, -9000, 9000));
    set_attitude_packet.content.main.yaw_abs = htole16(constrain_int16(degrees(wrap_PI(AP::ahrs().get_yaw_rad())) * 100, -18000, 18000));

    // byte 18~19: North acceleration of vehicle (int16, cm/s/s)
    // byte 20~21: East acceleration of vehicle (int16, cm/s/s)
    // byte 22~23: Upward acceleration of vehicle (int16, cm/s/s)
    const Vector3f &accel_ef = AP::ahrs().get_accel_ef();
    set_attitude_packet.content.main.accel_north = htole16(constrain_int16(accel_ef.x * 100, -INT16_MAX, INT16_MAX));
    set_attitude_packet.content.main.accel_east = htole16(constrain_int16(accel_ef.y * 100, -INT16_MAX, INT16_MAX));
    set_attitude_packet.content.main.accel_up = htole16(constrain_int16(-(accel_ef.z + GRAVITY_MSS) * 100, -INT16_MAX, INT16_MAX));

    // byte 24~25: North speed of vehicle (int16, decimeter/s)
    // byte 26~27: East speed of vehicle (int16, decimeter/s)
    // byte 28~29: Upward speed of vehicle (int16, decimeter/s)
    // ToDo: check scale (cm/s or decimeter/s)
    Vector3f velocity_ef;
    if (AP::ahrs().have_inertial_nav() && AP::ahrs().get_velocity_NED(velocity_ef)) {
        set_attitude_packet.content.main.vel_north = htole16(constrain_int16(velocity_ef.x * 10, -INT16_MAX, INT16_MAX));
        set_attitude_packet.content.main.vel_east = htole16(constrain_int16(velocity_ef.y * 10, -INT16_MAX, INT16_MAX));
        set_attitude_packet.content.main.vel_up = htole16(constrain_int16(-velocity_ef.z * 10, -INT16_MAX, INT16_MAX));
    }

    // byte 30: request code of sub frame, header of requested sub data frame from GCU (aka camera)
    set_attitude_packet.content.main.request_code = 0x01;

    // byte 31~36: reserved/unused

    // below is "sub data frame"
    // byte 37: sub header (0x01)
    set_attitude_packet.content.main.sub_header = 0x01;

    // byte 38~41: longitude of vehicle (int32, 1E7)
    // byte 42~45: latitude of vehicle (int32, 1E7)
    // byte 46~49: altitude of vehicle (int32, mm)
    Location veh_loc;
    if (AP::ahrs().get_location(veh_loc)) {
        // longitude
        set_attitude_packet.content.main.longitude = htole32(veh_loc.lng);

        // latitude
        set_attitude_packet.content.main.latitude = htole32(veh_loc.lat);

        // get absolute altitude and convert from cm to mm
        int32_t alt_amsl_cm;
        if (veh_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm)) {
            const int32_t alt_amsl_mm = alt_amsl_cm * 10;
            set_attitude_packet.content.main.alt_amsl = htole32(alt_amsl_mm);
        }
    }

    // byte 50: number of satellites
    set_attitude_packet.content.main.gps_num_sats = AP::gps().num_sats();

    // byte 51~54: GNSS milliseconds (uint32)
    const uint32_t gps_time_week_ms = AP::gps().time_week_ms();
    set_attitude_packet.content.main.gps_week_ms = htole32(gps_time_week_ms);

    // byte 55~56: GNSS week number (uint16)
    const uint16_t gps_time_week = AP::gps().time_week();
    set_attitude_packet.content.main.gps_week = htole16(gps_time_week);

    // byte 57~60: relative altitude (int32, mm, can be zero if unavailable)
    int32_t rel_alt_cm;
    if (veh_loc.initialised() && veh_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, rel_alt_cm)) {        
        const int32_t rel_alt_mm = rel_alt_cm * 10;
        set_attitude_packet.content.main.alt_rel = htole32(rel_alt_mm);
    }

    // byte 61~68: reserved/unused

    // byte 69: order (uint8)
    //     0x00: Null (sent to ensure next command is not ignored)
    //     0x01: Calibration
    //     0x03: Neutral
    //     0x10: Angle control
    set_attitude_packet.content.main.order = FunctionOrder::ANGLE_CONTROL;

    // calculate CRC
    const uint16_t crc16 = crc16_ccitt(set_attitude_packet.bytes, sizeof(SetAttitudePacket) - 2, 0);
    set_attitude_packet.content.crc.crc_high = HIGHBYTE(crc16);
    set_attitude_packet.content.crc.crc_low = LOWBYTE(crc16);

    // send packet to gimbal
    _uart->write(set_attitude_packet.bytes, sizeof(SetAttitudePacket));

    // store time of send
    last_send_ms = AP_HAL::millis();
}

// send simple (1byte) command to gimbal (e.g. take pic, start recording)
// returns true on success, false on failure to send
bool AP_Mount_XFRobot::send_simple_command(FunctionOrder order, uint8_t param)
{
    // exit immediately if not healthy or command buffer is too large
    if (!healthy() || _uart->txspace() < sizeof(SimpleCommand)) {
        return false;
    }

    // prepare packet to send to gimbal
    SimpleCommand simple_command {};

    // bytes 0~1: headers
    simple_command.content.main.header1 = SEND_HEADER1;
    simple_command.content.main.header2 = SEND_HEADER2;

    // byte 2~3: length (uint16)
    simple_command.content.main.length = htole16(sizeof(SimpleCommand));

    // byte 4: protocol version (0x02)
    simple_command.content.main.version = PROTOCOL_VERSION;

    // bytes 5 ~ 29 left as zero

    // byte 30: request code of sub frame, header of requested sub data frame from GCU (aka camera)
    simple_command.content.main.request_code = 0x01;

    // bytes 31 ~ 68 left as zero

    // byte 69: order (uint8)
    // byte 70: parameter (uint8_t)
    simple_command.content.main.order = order;
    simple_command.content.param_1byte = param;

    // calculate CRC
    const uint16_t crc16 = crc16_ccitt(simple_command.bytes, sizeof(SimpleCommand) - 2, 0);
    simple_command.content.crc.crc_high = HIGHBYTE(crc16);
    simple_command.content.crc.crc_low = LOWBYTE(crc16);

    // send packet to gimbal
    _uart->write(simple_command.bytes, sizeof(SimpleCommand));

    // packet sent
    return true;
}

// check for recording timeout
void AP_Mount_XFRobot::check_recording_timeout()
{
    // return immediately if not starting/stopping recording
    if (recording.request_ms == 0) {
        return;
    }

    // check for timeout
    if (AP_HAL::millis() - recording.request_ms > RECORD_REQUEST_TIMEOUT_MS) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s failed to %s recording", send_text_prefix, recording.request_start ? "start" : "stop");
        recording.request_ms = 0;
    }
}

#endif // HAL_MOUNT_XFROBOT_ENABLED
