/*
  Viewpro gimbal driver using custom serial protocol

  Packet format (courtesy of Viewpro's SDK document)

  -------------------------------------------------------------------------------------------
  Field     Index   Bytes       Description
  -------------------------------------------------------------------------------------------
  Header    0~2     3           0x55 0xAA 0xDC
  Length    3       1           bit0~5: body length, n=all bytes from byte3 to checksum, min=4, max=63
                                bits6~7: frame counter (increment by 1 compared to previous message sent)
  Frame Id  4       1
  Data      5~n+1   n           1st byte is command id (?)
  Checksum  n+2     1           XOR of byte3 to n+1 (inclusive)
 */

#pragma once

#include "AP_Mount_Backend_Serial.h"

#if HAL_MOUNT_VIEWPRO_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>

#define AP_MOUNT_VIEWPRO_PACKETLEN_MAX  63  // maximum number of bytes in a packet sent to or received from the gimbal

class AP_Mount_Viewpro : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Viewpro);

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // return true if this mount accepts roll targets
    bool has_roll_control() const override { return false; }

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    //
    // camera controls
    //

    // take a picture.  returns true on success
    bool take_picture() override;

    // start or stop video recording
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set zoom specified as a rate or percentage
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    // set tracking to none, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    bool set_tracking(TrackingType tracking_type, const Vector2f& top_left, const Vector2f& bottom_right) override;

    // set camera lens as a value from 0 to 5
    bool set_lens(uint8_t lens) override;

    // set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
    // primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
    bool set_camera_source(uint8_t primary_source, uint8_t secondary_source) override;

    // send camera information message to GCS
    void send_camera_information(mavlink_channel_t chan) const override;

    // send camera settings message to GCS
    void send_camera_settings(mavlink_channel_t chan) const override;

    //
    // rangefinder
    //

    // get rangefinder distance.  Returns true on success
    bool get_rangefinder_distance(float& distance_m) const override;

    // enable/disable rangefinder.  Returns true on success
    bool set_rangefinder_enable(bool enable) override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // send text prefix string to reduce flash cost
    static const char* send_text_prefix;

    // packet frame ids
    enum class FrameId : uint8_t {
        HANDSHAKE = 0x00,       // handshake sent to gimbal
        U = 0x01,               // communication configuration control (this packet is sent to gimbal)
        V = 0x02,               // communication configuration status (this is the reply to U)
        HEARTBEAT = 0x10,       // heartbeat received from gimbal
        A1 = 0x1A,              // target angles (sent)
        C1 = 0x1C,              // camera controls commonly used (sent)
        E1 = 0x1E,              // tracking controls commonly used (sent)
        C2 = 0x2C,              // camera controls infrequently used (sent)
        E2 = 0x2E,              // tracking controls infrequently used (sent)
        T1_F1_B1_D1 = 0x40,     // actual roll, pitch, yaw angles (received)
        M_AHRS = 0xB1,          // vehicle attitude and position (sent)
    };

    // U communication configuration control commands
    enum class CommConfigCmd : uint8_t {
        QUERY_FIRMWARE_VER = 0xD0,
        QUERY_MODEL = 0xE4,
    };

    // A1 servo status enum (used in A1, B1 packets)
    enum class ServoStatus : uint8_t {
        MANUAL_SPEED_MODE = 0x01,
        FOLLOW_YAW = 0x03,
        MANUAL_ABSOLUTE_ANGLE_MODE = 0x0B,
        FOLLOW_YAW_DISABLE = 0x0A,
    };

    // C1 image sensor choice
    enum class ImageSensor : uint8_t {
        NO_ACTION = 0x00,   // no image sensor is affected
        EO1 = 0x01,         // electro-optical, aka rgb
        IR = 0x02,          // infrared, aka thermal
        EO1_IR_PIP = 0x03,  // rgb is main, IR is picture-in-picture
        IR_EO1_PIP = 0x04,  // thermal is main, rgb is picture-in-picture
        FUSION = 0x05,      // rgb and thermal are fused
        IR1_13MM = 0x06,    // only valid for 1352 module
        IR2_52MM = 0x07,    // only valid for 1352 module
    };

    // C1 camera commands
    enum class CameraCommand : uint8_t {
        NO_ACTION = 0x00,
        STOP_FOCUS_AND_ZOOM = 0x01,
        ZOOM_OUT = 0x08,
        ZOOM_IN = 0x09,
        FOCUS_PLUS = 0x0A,
        FOCUS_MINUS = 0x0B,
        TAKE_PICTURE = 0x13,
        START_RECORD = 0x14,
        STOP_RECORD = 0x15,
        AUTO_FOCUS = 0x19,
        MANUAL_FOCUS = 0x1A
    };

    // C1 rangefinder commands
    enum class LRFCommand : uint8_t {
        NO_ACTION = 0x00,
        SINGLE_RANGING = 0x01,
        CONTINUOUS_RANGING_START = 0x02,
        LPCL_CONTINUOUS_RANGING_START = 0x03,
        STOP_RANGING = 0x05
    };

    // C2 camera commands
    enum class CameraCommand2 : uint8_t {
        SET_EO_ZOOM = 0x53
    };

    // D1 recording status (received from gimbal)
    enum class RecordingStatus : uint8_t {
        RECORDING_STOPPED = 0x00,
        RECORDING = 0x01,
        PICTURE_MODE = 0x02
    };

    // E1 tracking commands
    enum class TrackingCommand : uint8_t {
        STOP = 0x01,
        START = 0x03,
        SET_POINT = 0x0A,
        SET_RECT_TOPLEFT = 0x0B,
        SET_RECT_BOTTOMRIGHT = 0x0C,
    };

    // E1 tracking source (e.g. which camera)
    enum class TrackingSource : uint8_t {
        EO1 = 0x01,         // electro-optical, aka rgb
        IR = 0x02,          // infrared, aka thermal
        EO2 = 0x03,         // electro-optical, aka rgb
    };

    // E2 tracking commands2
    enum class TrackingCommand2 : uint8_t {
        SET_POINT = 0x0A,
        SET_RECT_TOPLEFT = 0x0B,
        SET_RECT_BOTTOMRIGHT = 0x0C,
    };

    // F1 tracking status (received from gimbal)
    enum class TrackingStatus : uint8_t {
        STOPPED = 0x00,     // not tracking
        SEARCHING = 0x01,   // searching
        TRACKING = 0x02,    // locked onto a target
        LOST = 0x03,        // lost target
    };

    // parsing state
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER1,
        WAITING_FOR_HEADER2,
        WAITING_FOR_HEADER3,
        WAITING_FOR_LENGTH,
        WAITING_FOR_FRAMEID,
        WAITING_FOR_DATA,
        WAITING_FOR_CRC
    };

    // packet formats
    union HandshakePacket {
        struct {
            FrameId frame_id;           // always 0x00
            uint8_t unused;             // always 0x00
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // U packed used to send communication configuration control commands
    // gimbal replies with V packet
    union UPacket {
        struct {
            FrameId frame_id;           // always 0x01
            CommConfigCmd control_cmd;  // see CommConfigCmd enum above
            uint8_t params[9];          // parameters (unused)
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // A1 used to send target angles and rates
    union A1Packet {
        struct {
            FrameId frame_id;           // always 0x1A
            ServoStatus servo_status;   // see ServoStatus enum above
            be16_t yaw_be;              // target yaw angle or rate msb
            be16_t pitch_be;            // target pitch angle or rate msb
            uint8_t unused[4];          // unused
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // C1 used to send camera commands (commonly used)
    union C1Packet {
        struct PACKED {
            FrameId frame_id;           // always 0x1C
            be16_t sensor_zoom_cmd_be;
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // C2 used to send camera commands (less commonly used)
    union C2Packet {
        struct {
            FrameId frame_id;           // always 0x2C
            CameraCommand2 cmd;         // see CameraCommand2 enum above
            be16_t value_be;            // value
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // E1 used to send tracking commands
    union E1Packet {
        struct {
            FrameId frame_id;           // always 0x1E
            TrackingSource source : 3;  // see TrackingSource enum above
            uint8_t unused : 5;         // param1 (unused)
            TrackingCommand cmd;        // see TrackingCommand enum above
            uint8_t param2;             // param2
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // E2 used to send tracking commands2
    union E2Packet {
        struct {
            FrameId frame_id;           // always 0x2E
            TrackingCommand2 cmd;       // see TrackingCommand2 enum above
            be16_t param1_be;
            be16_t param2_be;
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // M_AHRS used to send vehicle attitude and position to gimbal
    union M_AHRSPacket {
        struct PACKED {
            FrameId frame_id;           // always 0xB1
            uint8_t data_type;          // should be 0x07.  Bit0: Attitude, Bit1: GPS, Bit2 Gyro
            uint8_t unused2to8[7];      // unused
            be16_t roll_be;             // vehicle roll angle.  1bit=360deg/65536
            be16_t pitch_be;            // vehicle pitch angle.  1bit=360deg/65536
            be16_t yaw_be;              // vehicle yaw angle.  1bit=360deg/65536
            be16_t date_be;             // bit0~6:year, bit7~10:month, bit11~15:day
            uint8_t seconds_utc[3];     // seconds.  1bit = 0.01sec
            be16_t gps_yaw_be;          // GPS yaw
            uint8_t position_mark_bitmask;  // bit0:new position, bit1:clock fix calced, bit2:horiz calced, bit3:alt calced
            be32_t latitude_be;         // latitude.  1bit = 10e-7
            be32_t longitude_be;        // longitude.  1bit = 10e-7
            be32_t height_be;           // height.  1bit = 1mm
            be16_t ground_speed_N_be;   // ground speed in North direction. 1bit = 0.01m/s
            be16_t ground_speed_E_be;   // ground speed in East direction. 1bit = 0.01m/s
            be16_t vdop_be;             // GPS vdop. 1bit = 0.01
            be16_t ground_speed_D_be;   // speed downwards. 1bit = 0.01m/s
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // reading incoming packets from gimbal and confirm they are of the correct format
    // results are held in the _parsed_msg structure
    void read_incoming_packets();

    // process successfully decoded packets held in the _parsed_msg structure
    void process_packet();

    // calculate crc of the received message
    uint8_t calc_crc(const uint8_t *buf, uint8_t length) const;

    // get the length and frame count byte (3rd byte of all messages)
    // length is all bytes after the header including CRC
    uint8_t get_length_and_frame_count_byte(uint8_t length);

    // send packet to gimbal
    // returns true on success, false if outgoing serial buffer is full
    bool send_packet(const uint8_t* databuff, uint8_t databuff_len);

    // send handshake, gimbal will respond with T1_F1_B1_D1 packet that includes current angles
    void send_handshake();

    // set gimbal's lock vs follow mode
    // lock should be true if gimbal should maintain an earth-frame target
    // lock is false to follow / maintain a body-frame target
    bool set_lock(bool lock);

    // send communication configuration command (aka U packet), gimbal will respond with a V packet
    bool send_comm_config_cmd(CommConfigCmd cmd);

    // send target pitch and yaw rates to gimbal
    // yaw_is_ef should be true if yaw_rads target is an earth frame rate, false if body_frame
    bool send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef);

    // send target pitch and yaw angles to gimbal
    // yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
    bool send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef);

    // send camera command, affected image sensor and value (e.g. zoom speed)
    bool send_camera_command(ImageSensor img_sensor, CameraCommand cmd, uint8_t value, LRFCommand lrf_cmd = LRFCommand::NO_ACTION);

    // send camera command2 and corresponding value (e.g. zoom as absolute value)
    bool send_camera_command2(CameraCommand2 cmd, uint16_t value);

    // send tracking command and corresponding value (normally zero)
    bool send_tracking_command(TrackingCommand cmd, uint8_t value);

    // send camera command2 and corresponding parameter values
    bool send_tracking_command2(TrackingCommand2 cmd, uint16_t param1, uint16_t param2);

    // send vehicle attitude and position to gimbal
    bool send_m_ahrs();

    // internal variables
    uint8_t _msg_buff[AP_MOUNT_VIEWPRO_PACKETLEN_MAX];  // buffer holding latest bytes from gimbal
    uint8_t _msg_buff_len;                          // number of bytes held in msg buff
    const uint8_t _msg_buff_data_start = 2;         // data starts at this byte of _msg_buff

    // parser state and unpacked fields
    struct {
        uint8_t data_len;                           // expected number of data bytes
        uint8_t frame_id;                           // frame id (equivalent to command id)
        uint16_t data_bytes_received;               // number of data bytes received so far
        uint8_t crc;                                // latest message's crc
        ParseState state;                           // state of incoming message processing
    } _parsed_msg;

    // variables for sending packets to gimbal
    uint8_t _last_frame_counter;                    // frame counter sent in last message
    uint32_t _last_update_ms;                       // system time (in milliseconds) that angle or rate targets were last sent
    Vector3f _current_angle_rad;                    // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    uint32_t _last_current_angle_rad_ms;            // system time _current_angle_rad was updated (used for health reporting)
    bool _recording;                                // recording status received from gimbal
    bool _last_lock;                                // last lock mode sent to gimbal
    TrackingStatus _last_tracking_status;           // last tracking status received from gimbal (used to notify users)
    ImageSensor _image_sensor;                      // user selected image sensor (aka camera lens)
    float _zoom_times;                              // zoom times received from gimbal
    uint32_t _firmware_version;                     // firmware version from gimbal
    bool _got_firmware_version;                     // true once we have received the firmware version
    uint8_t _model_name[11] {};                     // model name received from gimbal
    bool _got_model_name;                           // true once we have received model name
    float _rangefinder_dist_m;                      // latest rangefinder distance (in meters)
};

#endif // HAL_MOUNT_VIEWPRO_ENABLED
