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

#include "AP_Mount_Backend.h"

#if HAL_MOUNT_TOPOTEK_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define AP_MOUNT_TOPOTEK_PACKETLEN_MAX              36          // maximum number of bytes in a packet sent to or received from the gimbal
#define AP_MOUNT_RECV_GIMBAL_CMD_CATEGORIES_NUM     6           // parse the number of gimbal command types

class AP_Mount_Topotek : public AP_Mount_Backend
{

public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    // Do not allow copies
    CLASS_NO_COPY(AP_Mount_Topotek);

    // init - performs any required initialisation for this instance
    void init() override;

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

    // set camera picture-in-picture mode
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

    // send text prefix string
    static const char* send_message_prefix;

    // reading incoming packets from gimbal and confirm they are of the correct format
    void read_incoming_packets(void);

    // request gimbal attitude
    void request_gimbal_attitude(void);

    // request gimbal memory card information
    void request_gimbal_sdcard_info(void);

    // request gimbal tracking status
    void request_track_status(void);

    // request gimbal basic information
    void request_gimbal_basic_info(void);

    // set angle target in degrees
    void send_angle_target(float roll_rad, float pitch_rad, float yaw_rad, bool yaw_is_ef);

    // sets rate target in deg/s
    void send_rate_target(float roll_rads, float pitch_rads, float yaw_rads, bool yaw_is_ef);

    // send time to gimbal
    bool send_time_to_gimbal(uint64_t &utc_time);

    // send GPS-related information to the gimbal
    bool send_location_info(void);

    // analysis the data information received
    void analyse_packet_data(void);

    // attitude information analysis of gimbal
    void gimbal_angle_analyse(void);

    // gimbal video information analysis
    void gimbal_record_analyse(void);

    // information analysis of gimbal storage card
    void gimbal_sdcard_analyse(void);

    // gimbal tracking information analysis
    void gimbal_track_analyse(void);

    // gimbal basic information analysis
    void gimbal_basic_info_analyse(void);

    // gimbal distance information analysis
    void gimbal_dist_info_analyse(void);

    // add check
    void add_check(int8_t *cmd, uint8_t len);

    // calculate checksum
    int8_t calculate_crc(int8_t *cmd, uint8_t len);

    // hexadecimal to character conversion
    int8_t hex2char(int8_t data);

    // character to number conversion
    uint8_t char_to_number(int8_t data);

    // send packet to gimbal
    // returns true on success, false if serial port initialization failed
    bool send_packet(int8_t* data_packet, uint8_t data_packet_len);

    // set gimbal's lock vs follow mode
    // lock should be true if gimbal should maintain an earth-frame target
    // lock is false to follow / maintain a body-frame target
    bool set_gimbal_lock(bool lock);

    // uart connected to gimbal
    AP_HAL::UARTDriver *_uart;

    // if the serial port initialization is successful, it is considered true
    bool _initialised;

    bool _recording;                                            // recording status (received from gimbal)
    bool _is_tracking;                                          // whether to enable the tracking state
    bool _sdcard_status;                                        // memory card status (received from gimbal)
    bool _last_lock;                                            // last lock mode sent to gimbal
    bool _got_gimbal_basic_info;                                // whether the gimbal information is obtained
    bool _last_zoom_stop = false;                               // record the latest zoom stopped state.
    bool _last_focus_stop = false;                              // record the latest focus stopped state.
    uint8_t _sent_time_count = 0;                               // send count
    int8_t _model_name[11] = "";                                // gimbal type
    ZoomType _zoom_type;                                        // current zoom type
    float _zoom_value;                                          // current zoom value
    uint32_t _firmware_ver = 0;                                 // firmware version
    Vector3f _current_angle_rad;                                // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    uint32_t _last_current_angle_ms = 0;                        // system time (in milliseconds) that angle information received from the gimbal
    uint32_t _last_update_ms = 0;                               // record the system time of the last update.
    uint8_t _last_req_count;                                    // last request count used to slow request to gimbal
    uint32_t _last_zoom_control_ms = 0;                         // system time (in milliseconds) that control gimbal zoom
    uint8_t _stop_order_count = 0;                              // number of stop commands sent since target rates became zero
    float _measure_dist_m = -1.0f;                              // latest rangefinder distance (in meters)

    // buffer holding bytes from latest packet.  This is only used to calculate the crc
    int8_t _msg_buff[AP_MOUNT_TOPOTEK_PACKETLEN_MAX]{};
    uint8_t _msg_buff_len = 0;

    // control gimbal command
    static int8_t _zoom_cmd[15];                            // zoom command
    static int8_t _start_record_video[15];                  // record command
    static int8_t _take_pic[15];                            // photo command
    static int8_t _focus_cmd[15];                           // focus command
    static int8_t _stop_track_cmd[15];                      // stop track command
    static int8_t _begin_track_cmd[25];                     // begin track command
    static int8_t _next_pip_mode[15];                       // picture-in-picture command
    static int8_t _get_gimbal_attitude[15];                 // get gimbal attitude command
    static int8_t _get_gimbal_sdcard_info[15];              // get gimbal memory card information command
    static int8_t _get_gimbal_track_status[15];             // get gimbal tracking status command
    static int8_t _get_gimbal_basic_info[15];               // get gimbal basic information command
    static int8_t _set_yaw_angle_cmd[20];                   // set gimbal yaw angle command
    static int8_t _set_pitch_angle_cmd[20];                 // set gimbal pitch angle command
    static int8_t _set_roll_angle_cmd[20];                  // set gimbal roll angle command
    static int8_t _set_yaw_pitch_roll_speed_cmd[20];        // set the speed of gimbal yaw, pitch and roll command
    static int8_t _set_gimbal_time[45];                     // set the gimbal time command
    static int8_t _set_gimbal_lat[25];                      // set the gimbal's latitude
    static int8_t _set_gimbal_lng[25];                      // set the gimbal's longitude
    static int8_t _set_gimbal_alt[25];                      // set the gimbal's altitude
    static int8_t _set_gimbal_azi[20];                      // send heading information to the gimbal
    static int8_t _set_gimbal_range_enable[15];             // open and close ranging command
    static int8_t _gimbal_control_stop[15];                 // stop control of the gimbal command
    static int8_t _gimbal_lock[15];                         // set whether the gimbal is locked or followed command
    // stores strings and member function pointers
    typedef struct {
        int8_t uart_cmd_key[4];					            // gimbal command key;
        void (AP_Mount_Topotek::*func)(void);		        // the gimbal command corresponding operation
    } UartCmdFunctionHandler;

    // stores command ID and corresponding member functions that are compared with the command received by the gimbal
    UartCmdFunctionHandler uart_recv_cmd_compare_list[AP_MOUNT_RECV_GIMBAL_CMD_CATEGORIES_NUM] = {
        {{"GIA"}, &AP_Mount_Topotek::gimbal_angle_analyse},
        {{"REC"}, &AP_Mount_Topotek::gimbal_record_analyse},
        {{"SDC"}, &AP_Mount_Topotek::gimbal_sdcard_analyse},
        {{"LRF"}, &AP_Mount_Topotek::gimbal_dist_info_analyse},
        {{"TRC"}, &AP_Mount_Topotek::gimbal_track_analyse},
        {{"VSN"}, &AP_Mount_Topotek::gimbal_basic_info_analyse},
    };
};

#endif // HAL_MOUNT_TOPOTEK_ENABLED
