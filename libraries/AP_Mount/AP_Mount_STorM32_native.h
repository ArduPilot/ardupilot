/*
  STorM32 mount backend class
 */
#pragma once

#include "AP_Mount.h"
#include "AP_Mount_Backend.h"
#include "STorM32_lib.h"

#define FIND_GIMBAL_MAX_SEARCH_TIME_MS  90000 //AP's startup has become quite slow, so give it plenty of time, set to 0 to disable

class AP_Mount_STorM32_native : public AP_Mount_Backend, public STorM32_lib
{

public:
    // Constructor
    AP_Mount_STorM32_native(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // do not allow copies
    AP_Mount_STorM32_native(const AP_Mount_STorM32_native &other) = delete;
    AP_Mount_STorM32_native &operator=(const AP_Mount_STorM32_native&) = delete;

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();
    virtual void update_fast();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const { return false; }

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

private:
    // helper to handle corrupted rcin data
    bool is_failsafe(void);

    // interface to STorM32_lib
    size_t _serial_txspace(void) override;
    size_t _serial_write(const uint8_t *buffer, size_t size, uint8_t priority) override;
    uint32_t _serial_available(void) override;
    int16_t _serial_read(void) override;
    uint16_t _rcin_read(uint8_t ch) override;

    // internal variables
    AP_HAL::UARTDriver *_uart;
    AP_Mount::MountType _mount_type;
    bool _initialised;              // true once the driver has been initialised
    bool _armed;                    // true once the gimbal has reached normal operation state

    enum TASKENUM {
        TASK_SLOT0 = 0,
        TASK_SLOT1,
        TASK_SLOT2,
        TASK_SLOT3,
        TASK_SLOT4,
        TASK_SLOTNUMBER,
    };
    uint64_t _task_time_last;
    uint16_t _task_counter;

    // discovery functions
    void find_gimbal_native(void);

    // send info to gcs functions
    bool _send_armeddisarmed;       // true when a armed/disarmed message should be send out
    void send_text_to_gcs(void);

    // bit mask, allows to enable/disable particular functions/features
    enum BITMASKENUM {
        SEND_STORM32LINK_V2 = 0x01,
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

    void set_status_angles_deg(float pitch_deg, float roll_deg, float yaw_deg);
    void get_status_angles_deg(float* pitch_deg, float* roll_deg, float* yaw_deg);

    // target out
    enum ANGLESTYPEENUM {
        ANGLES_DEG = 0, //the STorM32 convention is angles in deg, not rad!
        ANGLES_PWM
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
    enum MAV_MOUNT_MODE _target_mode_last;

    void set_target_angles_bymountmode(void);
    void get_pwm_target_angles_from_radio(uint16_t* pitch_pwm, uint16_t* roll_pwm, uint16_t* yaw_pwm);
    void get_valid_pwm_from_channel(uint8_t rc_in, uint16_t* pwm);
    void set_target_angles_rad(float pitch_rad, float roll_rad, float yaw_rad, enum MAV_MOUNT_MODE mount_mode);
    void set_target_angles_pwm(uint16_t pitch_pwm, uint16_t roll_pwm, uint16_t yaw_pwm, enum MAV_MOUNT_MODE mount_mode);
    void send_target_angles(void);
};
