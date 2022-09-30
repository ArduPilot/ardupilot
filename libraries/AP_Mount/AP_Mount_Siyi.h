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

#include "AP_Mount_Backend.h"

#ifndef HAL_MOUNT_SIYI_ENABLED
#define HAL_MOUNT_SIYI_ENABLED HAL_MOUNT_ENABLED
#endif

#if HAL_MOUNT_SIYI_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define AP_MOUNT_SIYI_PACKETLEN_MAX     22  // maximum number of bytes in a packet sent to or received from the gimbal

class AP_Mount_Siyi : public AP_Mount_Backend
{

public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Siyi);

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

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

    // set camera zoom step.  returns true on success
    // zoom out = -1, hold = 0, zoom in = 1
    bool set_zoom_step(int8_t zoom_step) override;

    // set focus in, out or hold.  returns true on success
    // focus in = -1, focus hold = 0, focus out = 1
    bool set_manual_focus_step(int8_t focus_step) override;

    // auto focus.  returns true on success
    bool set_auto_focus() override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

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
        ACQUIRE_GIMBAL_ATTITUDE = 0x0D
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

    // rotate gimbal.  pitch_rate and yaw_rate are scalars in the range -100 ~ +100
    // yaw_is_ef should be true if gimbal should maintain an earth-frame target (aka lock)
    void rotate_gimbal(int8_t pitch_scalar, int8_t yaw_scalar, bool yaw_is_ef);

    // center gimbal
    void center_gimbal();

    // set gimbal's lock vs follow mode
    // lock should be true if gimbal should maintain an earth-frame target
    // lock is false to follow / maintain a body-frame target
    void set_lock(bool lock);

    // send target pitch and yaw rates to gimbal
    // yaw_is_ef should be true if yaw_rads target is an earth frame rate, false if body_frame
    void send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef);

    // send target pitch and yaw angles to gimbal
    // yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
    void send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef);

    // internal variables
    AP_HAL::UARTDriver *_uart;                      // uart connected to gimbal
    bool _initialised;                              // true once the driver has been initialised
    bool _got_firmware_version;                     // true once gimbal firmware version has been received

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
    bool     _last_lock;                            // last lock value sent to gimbal
    uint8_t  _lock_send_counter;                    // counter used to resend lock status to gimbal at regular intervals

    // actual attitude received from gimbal
    Vector3f _current_angle_rad;                    // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    uint32_t _last_current_angle_rad_ms;            // system time _current_angle_rad was updated
    uint32_t _last_req_current_angle_rad_ms;        // system time that this driver last requested current angle

    // variables for camera state
    bool _last_record_video;                        // last record_video state sent to gimbal
};

#endif // HAL_MOUNT_SIYISERIAL_ENABLED
