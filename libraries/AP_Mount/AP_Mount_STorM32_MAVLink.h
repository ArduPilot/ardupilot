/*
  STorM32 mount backend class v1 & v2
 */
#pragma once

#include "AP_Mount_Backend.h"

#if HAL_MOUNT_STORM32_MAVLINK_V2_ENABLED

class AP_Mount_STorM32_MAVLink : public AP_Mount_Backend
{
public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically at 50 Hz
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // return true if this mount accepts roll or pitch targets
    bool has_roll_control() const override;
    bool has_pitch_control() const override;

    // returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override;

    // get attitude as a quaternion.  Returns true on success
    bool get_attitude_quaternion(Quaternion &att_quat) override;

    // get angular velocity of mount. Only available on some backends
    bool get_angular_velocity(Vector3f& rates) override;

    // set yaw_lock.  If true, the gimbal's yaw target is maintained in earth-frame meaning
    void set_yaw_lock(bool yaw_lock) override {}

    // send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
    // MissionPlaner "understands" gimbal device attitude status, but doesn't use it for campoint, so we
    // still want to send MOUNT_STATUS.
    // => for as long as MissionPlanner doesn't do mount/gimbal status properly
    //    we use this to stream MOUNT_STATUS
    void send_gimbal_device_attitude_status(mavlink_channel_t chan) override;

    // send a GIMBAL_MANAGER_INFORMATION message to GCS
    void send_gimbal_manager_information(mavlink_channel_t chan) override;

    // return gimbal device id
    uint8_t get_gimbal_device_id() const override;

    // return gimbal manager flags used by GIMBAL_MANAGER_STATUS message
    uint32_t get_gimbal_manager_flags() const override;

    // handle gimbal manager flags received from gimbal manager messages
    bool handle_gimbal_manager_flags(uint32_t flags) override;

    // handle GIMBAL_DEVICE_INFORMATION message
    void handle_gimbal_device_information(const mavlink_message_t &msg) override;

    // handle GIMBAL_DEVICE_ATTITUDE_STATUS message
    void handle_gimbal_device_attitude_status(const mavlink_message_t &msg) override;

    // handle_message_extra - allows to process a msg from a gimbal
    void handle_message_extra(const mavlink_message_t &msg) override;

    // send banner
    void send_banner() override;

private:

    // internal variables

    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal, zero if gimbal not yet discovered
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal

    bool _got_device_info;          // gimbal discovered, waiting for gimbal provide device info
    bool _initialised;              // true once all init steps have been passed
    bool _got_radio_rc_channels;    // true when a RADIO_RC_CHANNELS message has been received

    // gimbal and protocol discovery

    // search for gimbal in GCS_MAVLink routing table
    void find_gimbal();

    uint32_t _request_device_info_tlast_ms;
    mavlink_gimbal_device_information_t _device_info;
    uint16_t _device_version_int;

    // request GIMBAL_DEVICE_INFORMATION, we can get this also if 'old' MOUNT messages are used
    void send_cmd_request_gimbal_device_information();

    enum class Protocol {
        UNDEFINED = 0,     // we do not yet know
        MOUNT,             // gimbal uses 'old' MOUNT messages
        GIMBAL_DEVICE,     // gimbal is a v2 gimbal device
    };
    Protocol _protocol;

    // determine protocol based on receiving MOUNT_STATUS or GIMBAL_DEVICE_ATTITUDE_STATUS from gimbal
    void determine_protocol(const mavlink_message_t &msg);

    // pre-arm and healthy checks
    // some need to be made mutable to get around that healthy() is const

    enum STorM32State {
        STARTUP_MOTORS = 0,
        STARTUP_SETTLE,
        STARTUP_CALIBRATE,
        STARTUP_LEVEL,
        STARTUP_MOTORDIRDETECT,
        STARTUP_RELEVEL,
        NORMAL,
        STARTUP_FASTLEVEL,
    };

    bool _gimbal_armed;             // true once the gimbal has reached normal state
    uint32_t _gimbal_error_flags;   // error flags in custom_mode field of the HEARTBEAT message (restricted to 16 bits)
    bool _gimbal_prearmchecks_ok;   // true when the gimbal stops reporting prearm fail in the HEARTBEAT message

    mutable uint8_t _armingchecks_running; // true when ARMING_CHECK_ALL,ARMING_CHECK_CAMERA set and running, we know from healthy()
    bool _healthy;

    bool _prearmchecks_passed;      // becomes true when all checks were passed once at startup
    uint32_t _prearmcheck_sendtext_tlast_ms;
    bool _checks_last;              // result of last check, to detect toggle from true -> false, false -> true
    uint32_t _checks_tlast_ms;

    bool has_failures(char* s);
    bool is_healthy();
    void update_checks();

    uint32_t _request_send_banner_ms;
    void update_send_banner();

    struct {
        uint8_t fast;               // counter to determine faster responses
        uint32_t tlast_ms;          // time last GIMBAL_MANAGER_STATUS was send
        uint32_t flags_last;        // to detect changes
        uint8_t primary_sysid_last;
        uint8_t primary_compid_last;
    } _manager_status;
    void update_manager_status();

    // gimbal target & control

    struct {
        uint32_t received_tlast_ms; // time last MOUNT_STATUS was received (used for health reporting)
    } _mount_status;

    struct {
        uint16_t received_flags;    // obtained from GIMBAL_DEVICE_ATTITUDE_STATUS
        uint32_t received_failure_flags; // obtained from GIMBAL_DEVICE_ATTITUDE_STATUS
        uint32_t received_tlast_ms; // time last GIMBAL_DEVICE_ATTITUDE_STATUS was received (used for health reporting)
    } _device_status;

    uint32_t _flags_from_gimbal_client = UINT32_MAX; // flags received via gimbal manager messages/commands, the UINT32_MAX is important!
    uint16_t _flags_for_gimbal_device;     // flags to be send to gimbal device

    struct {
        float roll;
        float pitch;
        float yaw_bf;
        float delta_yaw;
    } _current_angles = {0.0f, 0.0f, 0.0f, NAN}; // current angles, obtained from MOUNT_STATUS or GIMBAL_DEVICE_ATTITUDE_STATUS, the NAN is important!
    Vector3f _current_omega;        // current angular velocities, obtained from GIMBAL_DEVICE_ATTITUDE_STATUS

    // set the flags for gimbal according to current conditions
    void update_gimbal_device_flags();

    // determines the target angles based on mount mode, does the crucial job of controlling
    void update_target_angles();

    // sends the target angles to gimbal, called in update loop with 12.5 Hz
    void send_target_angles();

    // send do_mount_control command with latest angle targets to the gimbal
    void send_cmd_do_mount_control();

    // send GIMBAL_DEVICE_SET_ATTITUDE with latest angle targets to the gimbal to control attitude
    // When the gimbal receives this message, it can assume it is coming from its gimbal manager
    // (as nobody else is allowed to send this to it). STorM32 thus identifies the message's
    // source sysid & compid as its associated gimbal manager's sysid & compid.
    void send_gimbal_device_set_attitude();

    uint32_t _tahrs_healthy_ms;
    void send_autopilot_state_for_gimbal_device();

    void send_rc_channels();

    uint32_t _send_system_time_tlast_ms;
    void send_system_time();

    // task

    enum Task {
        TASK_SLOT0 = 0,
        TASK_SLOT1,
        TASK_SLOT2,
        TASK_SLOT3
    };
    uint8_t _task_counter;
};

#endif // HAL_MOUNT_STORM32_MAVLINK_V2_ENABLED
