/*
  STorM32 mount backend class
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_Mount.h"
#include "AP_Mount_Backend.h"

#include "BP_STorM32.h"


#define FIND_GIMBAL_MAX_SEARCH_TIME_MS  60000


class AP_Mount_STorM32_UAVCAN : public AP_Mount_Backend, public BP_STorM32
{

public:
    // Constructor
    AP_Mount_STorM32_UAVCAN(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const { return false; }

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

    // interface to AP_UAVCAN, receive message
    void handle_storm32status_msg(float pitch_deg, float roll_deg, float yaw_deg);

private:
    bool is_failsafe(void);

    // interface to BP_STorM32
    virtual size_t _serial_txspace(void);
    virtual size_t _serial_write(const uint8_t *buffer, size_t size, uint8_t priority);
    virtual uint32_t _serial_available(void);
    virtual int16_t _serial_read(void);
    uint16_t _rcin_read(uint8_t ch);

    // internal variables
    AP_UAVCAN *_ap_uavcan[MAX_NUMBER_OF_CAN_DRIVERS];
    AP_HAL::UARTDriver *_uart;
    AP_Mount::MountType _mount_type;
    bool _initialised;              // true once the driver has been initialised
    uint64_t _task_time_last;
    uint16_t _task_counter;

    //we want to keep that info, just in case, the fields are 16 in size, so add one to convert it to string
    // _initialised also indicates that these fields were set
    char versionstr[16+1];
    char namestr[16+1];
    char boardstr[16+1];
    uint8_t _startupbanner_status; //0: wait, 1: send, 2: has been sent
    void send_startupbanner(void);

    //discovery functions
    void find_CAN(void);
    void find_gimbal(void);

    //bit mask
    enum BITMASKENUM {
        SEND_ATTITUDE = 0x01,
        SEND_CMD_SETINPUTS = 0x02,
        GET_PWM_TARGET_FROM_RADIO = 0x04,
        SEND_CMD_DOCAMERA = 0x08,
    };
    uint16_t _bitmask; //this mask is to control some functions

    // storm32.Status in
    struct {
        float pitch_deg;
        float roll_deg;
        float yaw_deg;
    } _status;
    bool _status_updated;

    void get_status_angles_deg(float* pitch_deg, float* roll_deg, float* yaw_deg);

    // target out
    enum ANGLESTYPEENUM {
        angles_deg = 0, //the STorM32 convention is angles in deg, not rad!
        angles_pwm
    };

    struct {
        enum MAV_MOUNT_MODE mode;
        enum ANGLESTYPEENUM type;
        union {
            struct {
                float pitch;
                float roll;
                float yaw;
            } deg;
            struct {
                uint16_t pitch;
                uint16_t roll;
                uint16_t yaw;
            } pwm;
        };
    } _target;
    bool _target_to_send;
    enum MAV_MOUNT_MODE _target_mode_last;

    void set_target_angles_bymountmode(void);
    void get_pwm_target_angles_from_radio(uint16_t* pitch_pwm, uint16_t* roll_pwm, uint16_t* yaw_pwm);
    void get_valid_pwm_from_channel(uint8_t rc_in, uint16_t* pwm);
    void set_target_angles_deg(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode);
    void set_target_angles_rad(float pitch_rad, float roll_rad, float yaw_rad, enum MAV_MOUNT_MODE mount_mode);
    void set_target_angles_pwm(uint16_t pitch_pwm, uint16_t roll_pwm, uint16_t yaw_pwm, enum MAV_MOUNT_MODE mount_mode);
    void send_target_angles(void);
};
