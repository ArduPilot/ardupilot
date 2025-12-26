/*
  Topotek gimbal driver using custom serial protocol

  Packet format (courtesy of Topotek's SDK document)

  -------------------------------------------------------------------------------------------
  Field                 Index   Bytes       Description
  -------------------------------------------------------------------------------------------
  Frame Header          0       3           type of command
  Address Bit           3       2           the source address comes first, and the destination address comes last
  Data_Len              5       1           data length
  Control Bit           6       1           r -> query w -> setup and control
  Identification Bit    7       3           identification function
  Data                  10      Data_Len
  Check Bit                     2           the frame header is converted to HEX before reaching the check bit,
                                            the sum is done, and the result is converted to ASC-II. Two bytes, the high one first
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_TOPOTEK_ENABLED

#include "AP_Mount_Backend_Serial.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define AP_MOUNT_TOPOTEK_PACKETLEN_MAX              36          // maximum number of bytes in a packet sent to or received from the gimbal
#define AP_MOUNT_RECV_GIMBAL_CMD_CATEGORIES_NUM     7           // parse the number of gimbal command types

class AP_Mount_Topotek : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    // Do not allow copies
    CLASS_NO_COPY(AP_Mount_Topotek);

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    //
    // camera controls for gimbals
    //

    // take a picture.  returns true on success
    bool take_picture() override;

    // start or stop video recording
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set zoom specified as a rate
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    // set focus specified as rate or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    // set tracking to none, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2) override;

    // send command to gimbal to cancel tracking (if necessary)
    // returns true on success, false on failure to send message
    bool cancel_tracking();

    // set camera picture-in-picture mode
    bool set_lens(uint8_t lens) override;

#if HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED
    // set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
    // primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
    bool set_camera_source(uint8_t primary_source, uint8_t secondary_source) override;
#endif

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

    // Topotek can send either rates or angles
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_AND_RATES_ONLY;
    };

private:

    // header type (fixed or variable length)
    // first three bytes of packet determined by this value
    enum class HeaderType : uint8_t {
        FIXED_LEN = 0x00,       // #TP will be sent
        VARIABLE_LEN = 0x01,    // #tp will be sent
    };

    // address (2nd and 3rd bytes of packet)
    // first byte is always U followed by one of the other options
    enum class AddressByte : uint8_t {
        SYSTEM_AND_IMAGE = 68,      // 'D'
        AUXILIARY_EQUIPMENT = 69,   // 'E'
        GIMBAL = 71,                // 'G'
        LENS = 77,                  // 'M'
        NETWORK = 80,               // 'P'
        UART = 85,                  // 'U'
    };

    // control byte (read or write)
    // sent as 7th byte of packet
    enum class ControlByte : uint8_t {
        READ = 114,     // 'r'
        WRITE = 119,    // 'w'
    };

    // parsing state
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER1 = 0,// #
        WAITING_FOR_HEADER2,    // T or t
        WAITING_FOR_HEADER3,    // P or p
        WAITING_FOR_ADDR1,      // normally U
        WAITING_FOR_ADDR2,      // M, D, E, P, G
        WAITING_FOR_DATALEN,
        WAITING_FOR_CONTROL,    // r or w
        WAITING_FOR_ID1,        // e.g. 'G'
        WAITING_FOR_ID2,        // e.g. 'A'
        WAITING_FOR_ID3,        // e.g. 'C'
        WAITING_FOR_DATA,       // normally hex numbers in char form (e.g. '0A')
        WAITING_FOR_CRC_LOW,
        WAITING_FOR_CRC_HIGH,
    };

    // tracking status
    enum class TrackingStatus : uint8_t {
        STOPPED_TRACKING = 0x30,                // not tracking
        WAITING_FOR_TRACKING = 0x31,            // wait to track command status
        TRACKING_IN_PROGRESS = 0x32,            // the status is being tracked
        LENS_UNSUPPORT_TRACK = 0x34,            // this lens does not support tracking
    };

    // identifier bytes
    typedef char Identifier[3];

    // send text prefix string
    static const char* send_message_prefix;

    // reading incoming packets from gimbal and confirm they are of the correct format
    void read_incoming_packets();

    // request gimbal attitude
    void request_gimbal_attitude();

    // request gimbal memory card information
    void request_gimbal_sdcard_info();

    // request gimbal tracking status
    void request_track_status();

    // request gimbal version
    void request_gimbal_version();

    // request gimbal model name
    void request_gimbal_model_name();

    // send angle target in radians to gimbal
    void send_target_angles(const MountAngleTarget& angle_rad) override;

    // send rate target in rad/s to gimbal
    void send_target_rates(const MountRateTarget& rate_rads) override;

    // send time and date to gimbal
    bool send_time_to_gimbal();

    // send GPS-related information to the gimbal
    bool send_location_info();

    // attitude information analysis of gimbal
    void gimbal_angle_analyse();

    // gimbal video information analysis
    void gimbal_record_analyse();

    // information analysis of gimbal storage card
    void gimbal_sdcard_analyse();

    // gimbal tracking information analysis
    void gimbal_track_analyse();

    // gimbal basic information analysis
    void gimbal_version_analyse();

    // gimbal model name message analysis
    void gimbal_model_name_analyse();

    // gimbal distance information analysis
    void gimbal_dist_info_analyse();

    // calculate checksum
    uint8_t calculate_crc(const uint8_t *cmd, uint8_t len) const;

    // hexadecimal to character conversion
    uint8_t hex2char(uint8_t data) const;

    // convert a 4 character hex number to an integer
    // the characters are in the format "1234" where the most significant digit is first
    int16_t hexchar4_to_int16(char high, char mid_high, char mid_low, char low) const;

    // send a fixed length packet to gimbal
    // returns true on success, false if serial port initialization failed
    bool send_fixedlen_packet(AddressByte address, const Identifier id, bool write, uint8_t value);

    // send a variable length packet to gimbal
    // returns true on success, false if serial port initialization failed
    bool send_variablelen_packet(HeaderType header, AddressByte address, const Identifier id, bool write, const uint8_t* databuff, uint8_t databuff_len);

    // set gimbal's lock vs follow mode
    // lock should be true if gimbal should maintain an earth-frame target
    // lock is false to follow / maintain a body-frame target
    bool set_gimbal_lock(bool lock);

    // members
    bool _recording;                                            // recording status (received from gimbal)
    bool _is_tracking;                                          // whether to enable the tracking state
    TrackingStatus _last_tracking_state = TrackingStatus::STOPPED_TRACKING; // last tracking state received from gimbal
    uint8_t _last_mode;                                         // mode during latest update, used to detect mode changes and cancel tracking
    bool _sdcard_status;                                        // memory card status (received from gimbal)
    bool _last_lock;                                            // last lock mode sent to gimbal
    bool _got_gimbal_version;                                   // true if gimbal's version has been received
    bool _got_gimbal_model_name;                                // true if gimbal's model name has been received
    bool _last_zoom_stop;                                       // true if zoom has been stopped (used to re-send in order to handle lost packets)
    bool _last_focus_stop;                                      // true if focus has been stopped (used to re-sent in order to handle lost packets)
    uint8_t _model_name[16];                                    // gimbal model name
    uint8_t _sent_time_count;                                   // count of current time messages sent to gimbal
    uint32_t _firmware_ver;                                     // firmware version
    Vector3f _current_angle_rad;                                // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    uint32_t _last_current_angle_ms;                            // system time (in milliseconds) that angle information received from the gimbal
    uint32_t _last_req_current_info_ms;                         // system time that this driver last requested current gimbal infomation
    uint8_t _last_req_step;                                     // 10hz request loop step (different requests are sent at various steps)
    uint8_t _stop_order_count;                                  // number of stop commands sent since target rates became zero
    float _measure_dist_m = -1.0f;                              // latest rangefinder distance (in meters)
    uint8_t _msg_buff[AP_MOUNT_TOPOTEK_PACKETLEN_MAX];          // buffer holding bytes from latest packet received.  only used to calculate crc
    uint8_t _msg_buff_len;                                      // number of bytes in the msg buffer
    struct {
        ParseState state;                                       // parser state
        uint8_t data_len;                                       // expected number of data bytes
    } _parser;

    // mapping from received message key to member function pointer to consume the message
    typedef struct {
        uint8_t uart_cmd_key[4];                                // gimbal message key;
        void (AP_Mount_Topotek::*func)(void);		            // member function to consume messager
    } UartCmdFunctionHandler;

    // stores command ID and corresponding member functions that are compared with the command received by the gimbal
    UartCmdFunctionHandler uart_recv_cmd_compare_list[AP_MOUNT_RECV_GIMBAL_CMD_CATEGORIES_NUM] = {
        {{"GIA"}, &AP_Mount_Topotek::gimbal_angle_analyse},
        {{"REC"}, &AP_Mount_Topotek::gimbal_record_analyse},
        {{"SDC"}, &AP_Mount_Topotek::gimbal_sdcard_analyse},
        {{"LRF"}, &AP_Mount_Topotek::gimbal_dist_info_analyse},
        {{"TRC"}, &AP_Mount_Topotek::gimbal_track_analyse},
        {{"VSN"}, &AP_Mount_Topotek::gimbal_version_analyse},
        {{"PA2"}, &AP_Mount_Topotek::gimbal_model_name_analyse}
    };
};

#endif // HAL_MOUNT_TOPOTEK_ENABLED
