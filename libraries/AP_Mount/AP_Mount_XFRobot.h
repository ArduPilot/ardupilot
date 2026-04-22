/*
  XFRobot camera gimbal using serial protocol backend class
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_XFROBOT_ENABLED

#include "AP_Mount_Backend_Serial.h"
#include <AP_Math/quaternion.h>

#define AP_MOUNT_XFROBOT_RECV_LENGTH_MAX    80  // maximum number of bytes that will be received from the gimbal

class AP_Mount_XFRobot : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    // has_roll_control - returns true if this mount can control its roll
    bool has_roll_control() const override { return roll_range_valid(); };

    // has_pitch_control - returns true if this mount can control its tilt
    bool has_pitch_control() const override { return pitch_range_valid(); };

    //
    // camera controls for gimbals that include a camera
    //

    // take a picture.  returns true on success
    bool take_picture() override;

    // start or stop video recording.  returns true on success
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set zoom specified as a rate or percentage
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    // set camera lens to cycle through pic-in-pic modes
    bool set_lens(uint8_t lens) override;

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // send text prefix string to reduce flash cost
    static const char* send_text_prefix;

    // camera and gimbal functions (aka order)
    enum class FunctionOrder : uint8_t {
        NONE = 0x00,                // null command
        CALIBRATION = 0x01,         // calibration
        ISOTHERM = 0x02,            // isotherm mode
        NEUTRAL = 0x03,             // neutral mode
        OSD_COORDINATE = 0x06,      // OSD coordinate control (vehicle or target)
        IMAGE_AUTO_REVERSE = 0x07,  // auto reverse on/off
        TIME_ZONE = 0x08,           // set timezone
        ANGLE_CONTROL = 0x10,       // angle control
        HEAD_LOCK = 0x11,           // head lock mode
        HEAD_FOLLOW = 0x12,         // head follow mode
        ORTHOVIEW = 0x13,           // orthoview mode
        EULER_ANGLE_CONTROL = 0x14, // euler angle control
        GAZE_GEO_COORDINATES = 0x15,// gaze mode with geo coordinates
        GAZE_GEO_TARGET_LOCK = 0x16,// gaze mode with geo target lock
        TRACK = 0x17,               // track mode
        CLICK_TO_AIM = 0x1A,        // click to aim
        FPV = 0x1C,                 // first person view
        SHUTTER = 0x20,             // take picture
        RECORD_VIDEO = 0x21,        // record video
        ZOOM_IN = 0x22,             // zoom in continuously
        ZOOM_OUT = 0x23,            // zoom ou continuously
        ZOOM_STOP = 0x24,           // stop zooming
        ZOOM_RATE = 0x25,           // zoom at specfiied rate
        FOCUS = 0x26,               // auto focus
        PALETTE = 0x2A,             // set palette
        NIGHT_VISION = 0x2B,        // set night vision on/off
        AREA_TEMP_MEASURE = 0x30,   // set area temperature measurement on/off
        AREA_TEMP_ALERT = 0x31,     // set area temperature alert on/off
        SPOT_TEMP_MEASURE = 0x33,   // set spot temperature measurement on/off
        OSD = 0x73,                 // show/hide osd
        PIC_IN_PIC = 0x74,          // show/hide picture in picture
        TARGET_DETECTION = 0x75,    // set target detection on/off
        DIGITAL_ZOOM = 0x76,        // set digital zoom on/off
        LIGHTING = 0x80,            // set lighting intensity (0~255)
        RANGING = 0x81,             // set ranging on/off
    };

    // parsing state
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER1,
        WAITING_FOR_HEADER2,
        WAITING_FOR_LENGTH_LOW,
        WAITING_FOR_LENGTH_HIGH,
        WAITING_FOR_DATA,
        WAITING_FOR_CRC_HIGH,
        WAITING_FOR_CRC_LOW
    };

    // camera image types
    enum class CameraType : uint8_t {
        MAIN_PIP_ZOOM_SUB_THERMAL = 1,
        MAIN_THERMAL_SUB_ZOOM = 2,
        MAIN_PIP_THERMAL_SUB_ZOOM = 3,
        MAIN_ZOOM_SUB_THERMAL = 4
       };

    // reading incoming packets from gimbal
    void read_incoming_packets();

    // process successfully decoded packets held in the _msg_buff structure
    void process_packet();

    // XFRobot can only send angles
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_ONLY;
    };

    // send_target_angles
    void send_target_angles(const MountAngleTarget& angle_target_rad) override;

    // send simple (1byte) command to gimbal (e.g. take pic, start recording)
    // returns true on success, false on failure to send
    bool send_simple_command(FunctionOrder order, uint8_t param);

    // check for recording timeout
    void check_recording_timeout();

    // internal variables
    uint32_t last_send_ms;     // system time of last do_mount_control sent to gimbal

    //
    // packet structures
    //

    // SendPacket main and sub frames for sending to the camera gimbal
    struct PACKED SendPacketMainAndSubFrame {
        uint8_t header1;        // byte0, header1 (0x8A)
        uint8_t header2;        // byte1, header2 (0x5E)
        uint16_t length;        // byte2~3, length
        uint8_t version;        // byte 4: version (always 0x02)
        int16_t roll_control;   // byte 5~6: roll control value (int16, -18000 ~ +18000)
        int16_t pitch_control;  // byte 7~8: pitch control value (int16, -18000 ~ +18000)
        int16_t yaw_control;    // byte 9~10: yaw control value (int16, -18000 ~ +18000)
        uint8_t status;         // byte 11: status, Bit0:INS valid, Bit2:control values valid
        int16_t roll_abs;       // byte 12~13: absolute roll angle of vehicle (int16, -18000 ~ +18000)
        int16_t pitch_abs;      // byte 14~15: absolute pitch angle of vehicle (int16, -9000 ~ +9000)
        int16_t yaw_abs;        // byte 16~17: absolute yaw angle of vehicle (uint16, 0 ~ 36000)    
        int16_t accel_north;    // byte 18~19: North acceleration of vehicle (int16, cm/s/s)
        int16_t accel_east;     // byte 20~21: East acceleration of vehicle (int16, cm/s/s)
        int16_t accel_up;       // byte 22~23: Upward acceleration of vehicle (int16, cm/s/s)
        int16_t vel_north;      // byte 24~25: North speed of vehicle (int16, decimeter/s)
        int16_t vel_east;       // byte 26~27: East speed of vehicle (int16, decimeter/s)
        int16_t vel_up;         // byte 28~29: Upward speed of vehicle (int16, decimeter/s)
        uint8_t request_code;   // byte 30: request code of sub frame, header of requested sub data frame from GCU (aka camera)
        uint8_t reserved[6];    // byte 31~36: reserved/unused
        uint8_t sub_header;     // byte 37: header (always 0x01)
        int32_t longitude;      // byte 38~41: longitude of vehicle (int32, 1E7)
        int32_t latitude;       // byte 42~45: latitude of vehicle (int32, 1E7)
        int32_t alt_amsl;       // byte 46~49: altitude of vehicle (int32, mm)
        uint8_t gps_num_sats;   // byte 50: number of satellites
        uint32_t gps_week_ms;   // byte 51~54: GNSS milliseconds (uint32)
        uint16_t gps_week;      // byte 55~56: GNSS week number (uint16)
        int32_t alt_rel;        // byte 57~60: relative altitude (int32, mm, can be zero if unavailable)
        uint8_t reserved2[8];   // byte 61~68: reserved/unused
        FunctionOrder order;    // byte 69: order
    };

    // ReplyPacket main and sub frames for receiving data from camera gimbal
    struct PACKED ReplyPacketMainAndSubFrame {
        uint8_t header1;        // byte0, header1 (0x8A)
        uint8_t header2;        // byte1, header2 (0x5E)
        uint16_t length;        // byte2~3, length
        uint8_t version;        // byte 4: version
        uint8_t mode;           // byte 5: mode: 0x10:angle control, 0x11:head lock, 0x12: head follow, 0x13:orthoview, 0x14:euler angle control, 0x16:gaze, 0x17:track,0x1C:FPV
        uint16_t status;        // byte 6~7: status, Bit0:tracking success, Bit7:range and target coordinate valid, Bit8:ranging on, Bit9:night vision on, Bit10:lighting on, Bit12:Upward powered on
        int16_t horizontal_target; // byte 8~9: horizontal target (int16, -1000 ~ +1000, rightward is positive)
        int16_t vertical_target;   // byte 10~11: vertical target (int16, -1000 ~ +1000, downward is positive)
        int16_t angle_x;        // byte 12~13: x-axis angle (camera-frame pitch) of camera relative to vehicle (int16, -18000 ~ +18000)
        int16_t angle_y;        // byte 14~15: y-axis angle (camera-frame roll) of camera relative to vehicle (int16, -18000 ~ +18000)
        int16_t angle_z;        // byte 16~17: z-axis angle (camera-frame yaw) of camera relative to vehicle (int16, -18000 ~ +18000)
        int16_t roll_abs_cd;    // byte 18~19: roll angle of camera (absolute) (int16, -9000 ~ +9000)
        int16_t pitch_abs_cd;   // byte 20~21: pitch angle of camera (absolute) (int16, -18000 ~ +18000)
        uint16_t yaw_abs_cd;    // byte 22~23: yaw angle of camera (absolute) (uint16, 0 ~ +36000)
        int16_t angvel_x;       // byte 24~25: X-axis absolute angular velocity of camera (int16, centi-degrees/s)
        int16_t angvel_y;       // byte 26~27: Y-axis absolute angular velocity of camera (int16, centi-degrees/s)
        int16_t angvel_z;       // byte 28~29: Z-axis absolute angular velocity of camera (int16, centi-degrees/s)
        uint8_t reserved[7];    // byte 30~36: reserved/unused
        uint8_t sub_header;     // byte 37: header (0x01)
        uint8_t hardware_version; // byte 38: hardware version (uint8)
        uint8_t firmware_version; // byte 39: firmware version (uint8)
        uint8_t pod_code;       // byte 40: pod code
        uint16_t error_code;    // byte 41~42: error code, Bit7:hardware error, Bit13: mavlink communication freq anomaly, Bit14:BNSS unpositioned, Bit15:GCU hardware error
        int32_t target_dist_dm; // byte 43~46: distance from target (int32, decimeters, -1m or 0m is invalid measurement)
        int32_t target_lng;     // byte 47~50: longitude of target (int32, 1E7)
        int32_t target_lat;     // byte 51~54: latitude of target (int32, 1E7)
        int32_t target_alt;     // byte 55~58: altitude of target (int32, mm)
        uint16_t zoom_rate_rgb; // byte 59~60: zoom rate of RGB camera (uint16, resolution 0.1x)
        uint16_t zoom_rate_thermal; // byte 61~62: zoom rate of thermal camera (uint16, resolution 0.1x)
        uint8_t thermal_status; // byte 63: thermal camera status, Bit0:low temp alert, Bit1:high temp alert, Bit3:spot temp measurement on, Bit4:Isotherm on, Bit5:temp alert on, Bit6:area temp on, Bit7:temp available
        uint16_t camera_status; // byte 64~65: camera status, Bit0~Bit2:pic-in-pic mode, Bit4:recording, Bit11:image auto reverse off, Bit12:OSD displays target coordinate, Bit13:OSD on, Bit14:digital zoom on, Bit15:target detection on
        int8_t timezone;        // byte 66: timezone (int8, -12 ~ +12)
        uint8_t reserved2[2];   // byte 67~68: reserved/unused
        FunctionOrder order;   // byte 69: order
    };

    // 1byte Reply Packet.  Used for most replies from GCU
    struct PACKED PacketReply1Byte {
        uint8_t execution_state;    // byte 70, Success:0x00, Fail:0x01, Operating:0x02
    };

    // Packet CRC
    struct PACKED PacketCRC {
        uint8_t crc_high;    // byte 70: CRC high byte
        uint8_t crc_low;     // byte 71: CRC low byte
    };

    // structure to send set-atitude command to gimbal
    union SetAttitudePacket {
        struct PACKED {
            SendPacketMainAndSubFrame main;
            PacketCRC crc;
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // structure to send a simple, 1byte command
    union SimpleCommand {
        struct PACKED {
            SendPacketMainAndSubFrame main;
            uint8_t param_1byte;
            PacketCRC crc;
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // structure to decode incoming packets
    struct PACKED GCUSimpleReply {
        ReplyPacketMainAndSubFrame main;
        PacketReply1Byte param;
        PacketCRC crc;
    };

    // parser related variables
    union {
        GCUSimpleReply simple_reply;                        // simple reply
        uint8_t bytes[AP_MOUNT_XFROBOT_RECV_LENGTH_MAX];    // raw bytes
    } msg_buff;
    uint8_t msg_buff_len;                                   // number of bytes held in msg_buff.bytes

    struct {
        ParseState state;           // state of incoming message processing
        uint8_t len_expected;       // expected number of data bytes we expect to receive
        uint16_t crc;               // latest message's crc
        uint32_t last_received_ms;  // system time of last message received and successfully parsed
    } parser;

    // latest attitude
    struct {
        float roll_ef_deg;      // earth-frame roll angle in degrees
        float pitch_ef_deg;     // earth-frame pitch angle in degrees
        float yaw_bf_deg;       // body-frame yaw angle in degrees
        uint32_t update_ms;     // system time of last update
    } attitude_latest;

    // local variables
    bool got_firmware_version = false;  // true once hardware and firmware version has been received
    struct {
        bool recording;         // true if currently recording video
        bool request_start;     // true if requested start recording and request_ms > 0, false if requested stop recording
        uint32_t request_ms;    // system time of request to start recording, 0 if not requested
    } recording;
};
#endif // HAL_MOUNT_XFROBOT_ENABLED
