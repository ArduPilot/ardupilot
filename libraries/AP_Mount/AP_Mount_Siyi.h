/*
  Siyi gimbal driver using custom serial protocol

  Packet format (courtesy of Siyi's SDK document)

  -------------------------------------------------------------------------------------------
  Field     Index   Bytes       Description
  -------------------------------------------------------------------------------------------
  STX       0       2           0x5566: starting mark
  CTRL      2       1           bit 0: need_ack.  set if the current data packet needs ack
                                bit 1: ack_pack.  set if the current data packate IS an ack
                                bit 2-7: reserved
  Data_len  3       2           Data field byte length.  Low byte in the front
  SEQ       5       2           Frame sequence (0 ~ 65535).  Low byte in the front.  May be used to detect packet loss
  CMD_ID    7       1           Command ID
  DATA      8       Data_len    Data
  CRC16             2           CRC16 check the complete data package.  Low byte in the front
 */

#pragma once

#include "AP_Mount_Backend_Serial.h"

#if HAL_MOUNT_SIYI_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define AP_MOUNT_SIYI_PACKETLEN_MAX     42  // maximum number of bytes in a packet sent to or received from the gimbal

class AP_Mount_Siyi : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Siyi);

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

    // set camera lens as a value from 0 to 8.  ZT30 only
    bool set_lens(uint8_t lens) override;

    // set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
    // primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
    bool set_camera_source(uint8_t primary_source, uint8_t secondary_source) override;

    // send camera information message to GCS
    void send_camera_information(mavlink_channel_t chan) const override;

    // send camera settings message to GCS
    void send_camera_settings(mavlink_channel_t chan) const override;

#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
    // send camera thermal range message to GCS
    void send_camera_thermal_range(mavlink_channel_t chan) const override;
#endif

    // change camera settings not normally used by autopilot
    // THERMAL_PALETTE: 0:WhiteHot, 2:Sepia, 3:IronBow, 4:Rainbow, 5:Night, 6:Aurora, 7:RedHot, 8:Jungle, 9:Medical, 10:BlackHot, 11:GloryHot
    // THERMAL_GAIN: 0:Low gain (50C ~ 550C), 1:High gain (-20C ~ 150C)
    // THERMAL_RAW_DATA: 0:Disable Raw Data (30fps), 1:Enable Raw Data (25fps)
    bool change_setting(CameraSetting setting, float value) override;

    //
    // rangefinder
    //

    // get rangefinder distance.  Returns true on success
    bool get_rangefinder_distance(float& distance_m) const override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

    // get angular velocity of mount. Only available on some backends
    bool get_angular_velocity(Vector3f& rates) override {
        rates = _current_rates_rads;
        return true;
    }
    
private:

    // serial protocol command ids
    enum class SiyiCommandId {
        ACQUIRE_FIRMWARE_VERSION = 0x01,
        HARDWARE_ID = 0x02,
        AUTO_FOCUS = 0x04,
        MANUAL_ZOOM_AND_AUTO_FOCUS = 0x05,
        MANUAL_FOCUS = 0x06,
        GIMBAL_ROTATION = 0x07,
        CENTER = 0x08,
        ACQUIRE_GIMBAL_CONFIG_INFO = 0x0A,
        FUNCTION_FEEDBACK_INFO = 0x0B,
        PHOTO = 0x0C,
        ACQUIRE_GIMBAL_ATTITUDE = 0x0D,
        ABSOLUTE_ZOOM = 0x0F,
        SET_CAMERA_IMAGE_TYPE = 0x11,
        GET_TEMP_FULL_IMAGE = 0x14,
        READ_RANGEFINDER = 0x15,
        SET_THERMAL_PALETTE = 0x1B,
        EXTERNAL_ATTITUDE = 0x22,
        SET_TIME = 0x30,
        SET_THERMAL_RAW_DATA = 0x34,
        SET_THERMAL_GAIN = 0x38,
        POSITION_DATA = 0x3e,
    };

    // Function Feedback Info packet info_type values
    enum class FunctionFeedbackInfo : uint8_t {
        SUCCESS = 0,
        FAILED_TO_TAKE_PHOTO = 1,
        HDR_ON = 2,
        HDR_OFF = 3,
        FAILED_TO_RECORD_VIDEO = 4
    };

    // Photo Function packet func_type values
    enum class PhotoFunction : uint8_t {
        TAKE_PICTURE = 0,
        HDR_TOGGLE = 1,
        RECORD_VIDEO_TOGGLE = 2,
        LOCK_MODE = 3,
        FOLLOW_MODE = 4,
        FPV_MODE = 5
    };

    // parsing state
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER_LOW,
        WAITING_FOR_HEADER_HIGH,
        WAITING_FOR_CTRL,
        WAITING_FOR_DATALEN_LOW,
        WAITING_FOR_DATALEN_HIGH,
        WAITING_FOR_SEQ_LOW,
        WAITING_FOR_SEQ_HIGH,
        WAITING_FOR_CMDID,
        WAITING_FOR_DATA,
        WAITING_FOR_CRC_LOW,
        WAITING_FOR_CRC_HIGH,
    };

    // hardware model enum
    enum class HardwareModel : uint8_t {
        UNKNOWN = 0,
        A2,
        A8,
        ZR10,
        ZR30,
        ZT6,
        ZT30
    } _hardware_model;

    enum class HdrStatus : uint8_t {
        OFF = 0,
        ON  = 1,
    };

    enum class RecordingStatus : uint8_t {
        OFF       = 0,
        ON        = 1,
        NO_CARD   = 2,
        DATA_LOSS = 3,
    };

    enum class GimbalMotionMode : uint8_t {
        LOCK   = 0,
        FOLLOW = 1,
        FPV    = 2,
    };

    enum class GimbalMountingDirection : uint8_t {
        UNDEFINED = 0,
        NORMAL = 1,
        UPSIDE_DOWN = 2,
    };

    enum class VideoOutputStatus : uint8_t {
        HDMI = 0,
        CVBS = 1,
    };

    // Response message for "Acquire Gimbal Confuguration Information" (0x0A)
    typedef struct {
        uint8_t _reserved1;
        HdrStatus hdr_status;
        uint8_t _reserved3;
        RecordingStatus record_status;
        GimbalMotionMode motion_mode;
        GimbalMountingDirection mounting_dir;
        VideoOutputStatus video_mode;
    } GimbalConfigInfo;
    static_assert(sizeof(GimbalConfigInfo) == 7, "GimbalConfigInfo must be 7 bytes");

    // camera image types (aka lens)
    enum class CameraImageType : uint8_t {
        MAIN_PIP_ZOOM_THERMAL_SUB_WIDEANGLE = 0,
        MAIN_PIP_WIDEANGLE_THERMAL_SUB_ZOOM = 1,
        MAIN_PIP_ZOOM_WIDEANGLE_SUB_THERMAL = 2,
        MAIN_ZOOM_SUB_THERMAL = 3,
        MAIN_ZOOM_SUB_WIDEANGLE = 4,
        MAIN_WIDEANGLE_SUB_THERMAL = 5,
        MAIN_WIDEANGLE_SUB_ZOOM = 6,
        MAIN_THERMAL_SUB_ZOOM = 7,
        MAIN_THERMAL_SUB_WIDEANGLE = 8
    };

    typedef struct {
        uint8_t major;
        uint8_t minor;
        uint8_t patch;
    } Version;
    typedef struct {
        Version camera;
        Version gimbal;
        Version zoom;
        bool received; // true once version information has been received
    } FirmwareVersion;

    // reading incoming packets from gimbal and confirm they are of the correct format
    // results are held in the _parsed_msg structure
    void read_incoming_packets();

    // process successfully decoded packets held in the _parsed_msg structure
    void process_packet();

    // send packet to gimbal
    // returns true on success, false if outgoing serial buffer is full
    bool send_packet(SiyiCommandId cmd_id, const uint8_t* databuff, uint8_t databuff_len);
    bool send_1byte_packet(SiyiCommandId cmd_id, uint8_t data_byte);

    // request info from gimbal
    void request_firmware_version() { send_packet(SiyiCommandId::ACQUIRE_FIRMWARE_VERSION, nullptr, 0); }
    void request_hardware_id() { send_packet(SiyiCommandId::HARDWARE_ID, nullptr, 0); }
    void request_configuration() { send_packet(SiyiCommandId::ACQUIRE_GIMBAL_CONFIG_INFO, nullptr, 0); }
    void request_function_feedback_info() { send_packet(SiyiCommandId::FUNCTION_FEEDBACK_INFO, nullptr, 0); }
    void request_gimbal_attitude() { send_packet(SiyiCommandId::ACQUIRE_GIMBAL_ATTITUDE, nullptr, 0); }
    void request_rangefinder_distance() { send_packet(SiyiCommandId::READ_RANGEFINDER, nullptr, 0); }

    // rotate gimbal.  pitch_rate and yaw_rate are scalars in the range -100 ~ +100
    // yaw_is_ef should be true if gimbal should maintain an earth-frame target (aka lock)
    void rotate_gimbal(int8_t pitch_scalar, int8_t yaw_scalar, bool yaw_is_ef);

    // Set gimbal's motion mode if it has changed. Use force=true to always send.
    //   FOLLOW: roll and pitch are in earth-frame, yaw is in body-frame
    //   LOCK: roll, pitch and yaw are all in earth-frame
    //   FPV: roll, pitch and yaw are all in body-frame
    // Returns true if message successfully sent to Gimbal
    bool set_motion_mode(const GimbalMotionMode mode, const bool force=false);

    // send target pitch and yaw rates to gimbal
    // yaw_is_ef should be true if yaw_rads target is an earth frame rate, false if body_frame
    void send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef);

    // send target pitch and yaw angles to gimbal
    // yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
    void send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef);

    // send zoom rate command to camera. zoom out = -1, hold = 0, zoom in = 1
    bool send_zoom_rate(float zoom_value);

    // send zoom multiple command to camera. e.g. 1x, 10x, 30x
    bool send_zoom_mult(float zoom_mult);

    // get zoom multiple max
    float get_zoom_mult_max() const;

    // update zoom controller
    void update_zoom_control();

    // get model name string, returns nullptr if hardware id is unknown
    const char* get_model_name() const;

    // Checks that the firmware version on the Gimbal meets the minimum supported version.
    void check_firmware_version() const;

#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
    // get thermal min/max if available at 5hz
    void request_thermal_minmax();
#endif

    // internal variables
    bool _got_hardware_id;                          // true once hardware id ha been received

    FirmwareVersion _fw_version;                    // firmware version (for reporting for GCS)

    // buffer holding bytes from latest packet.  This is only used to calculate the crc
    uint8_t _msg_buff[AP_MOUNT_SIYI_PACKETLEN_MAX];
    uint8_t _msg_buff_len;
    const uint8_t _msg_buff_data_start = 8;         // data starts at this byte of _msg_buff

    // parser state and unpacked fields
    struct PACKED {
        uint16_t data_len;                          // expected number of data bytes
        uint8_t command_id;                         // command id
        uint16_t data_bytes_received;               // number of data bytes received so far
        uint16_t crc16;                             // latest message's crc
        ParseState state;                           // state of incoming message processing
    } _parsed_msg;

    // variables for sending packets to gimbal
    uint32_t _last_send_ms;                         // system time (in milliseconds) of last packet sent to gimbal
    uint16_t _last_seq;                             // last sequence number used (should be increment for each send)

    // actual attitude received from gimbal
    Vector3f _current_angle_rad;                    // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    Vector3f _current_rates_rads;                   // current angular rates in rad/s (x=roll, y=pitch, z=yaw)
    uint32_t _last_current_angle_rad_ms;            // system time _current_angle_rad was updated
    uint32_t _last_req_current_angle_rad_ms;        // system time that this driver last requested current angle

    // absolute zoom control.  only used for A8 that does not support abs zoom control
    ZoomType _zoom_type;                            // current zoom type
    float _zoom_rate_target;                        // current zoom rate target
    float _zoom_mult;                               // most recent actual zoom multiple received from camera
    uint32_t _last_zoom_control_ms;                 // system time that zoom control was last run

    // Configuration info received from gimbal
    GimbalConfigInfo _config_info;
    
    // rangefinder variables
    uint32_t _last_rangefinder_req_ms;              // system time of last request for rangefinder distance
    uint32_t _last_rangefinder_dist_ms;             // system time of last successful read of rangefinder distance
    float _rangefinder_dist_m;                      // distance received from rangefinder

    // sending of attitude and position to gimbal
    uint32_t _last_attitude_send_ms;
    void send_attitude_position(void);

    // hardware lookup table indexed by HardwareModel enum values (see above)
    struct HWInfo {
        uint8_t hwid[2];
        const char* model_name;
    };
    static const HWInfo hardware_lookup_table[];

#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
    // thermal variables
    struct {
        uint32_t last_req_ms;       // system time of last request for thermal min/max temperatures
        uint32_t last_update_ms;    // system time of last update of thermal min/max temperatures
        float max_C;                // thermal max temp in C
        float min_C;                // thermal min temp in C
        Vector2ui max_pos;          // thermal max temp position on image in pixels. x=0 is left, y=0 is top
        Vector2ui min_pos;          // thermal min temp position on image in pixels. x=0 is left, y=0 is top
    } _thermal;
#endif

    // count of SET_TIME packets, we send 5 times to cope with packet loss
    uint8_t sent_time_count;
};

#endif // HAL_MOUNT_SIYISERIAL_ENABLED
