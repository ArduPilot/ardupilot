/*
  DroneCAN gimbal driver

  Implements gimbal control and attitude feedback using the DroneCAN / DSDL / com / xacti messages
  see https://github.com/dronecan/DSDL/tree/master/com/xacti

 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_XACTI_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_Mount.h"

class AP_Mount_Xacti : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_Xacti(class AP_Mount &frontend, class AP_Mount_Params &params, uint8_t instance);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Xacti);

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

    // set zoom specified as a rate or percentage
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    // set camera lens as a value from 0 to 5
    bool set_lens(uint8_t lens) override;

    // set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
    // primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
    bool set_camera_source(uint8_t primary_source, uint8_t secondary_source) override;

    // send camera information message to GCS
    void send_camera_information(mavlink_channel_t chan) const override;

    // send camera settings message to GCS
    void send_camera_settings(mavlink_channel_t chan) const override;

    // subscribe to Xacti DroneCAN messages
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

    // xacti specific message handlers
    static void handle_gimbal_attitude_status(AP_DroneCAN* ap_dronecan, const CanardRxTransfer& transfer, const com_xacti_GimbalAttitudeStatus &msg);
    static void handle_gnss_status_req(AP_DroneCAN* ap_dronecan, const CanardRxTransfer& transfer, const com_xacti_GnssStatusReq &msg);

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

    // Xacti can send either rates or angles
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_AND_RATES_ONLY;
    };

private:

    // send text prefix string to reduce flash cost
    static const char* send_text_prefix;

    // Sensor mode enumeration (aka lens)
    enum class SensorsMode : uint8_t {
        RGB = 0,    // RGB (aka "visible)")
        IR = 1,     // Infrared, aka thermal
        PIP = 2,    // RGB with IR PIP
        NDVI = 3,   // NDVI (vegetation greenness)
    };
    // array of sensor mode enumeration strings for display to user
    // if enum above is updated, also update sensor_mode_str definition in cpp
    static const char* sensor_mode_str[];

    // send target pitch and yaw rates to gimbal
    void send_target_rates(const MountRateTarget &rate_rads) override;

    // send target pitch and yaw angles to gimbal
    void send_target_angles(const MountAngleTarget &angle_rad) override;

    // register backend in detected modules array used to map DroneCAN port and node id to backend
    void register_backend();

    // find backend associated with the given dronecan port and node_id.  also associates backends with zero node ids
    // returns pointer to backend on success, nullptr on failure
    static AP_Mount_Xacti* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id);

    // DroneCAN parameter handling methods
    FUNCTOR_DECLARE(param_int_cb, bool, AP_DroneCAN*, const uint8_t, const char*, int32_t &);
    FUNCTOR_DECLARE(param_string_cb, bool, AP_DroneCAN*, const uint8_t, const char*, AP_DroneCAN::string &);
    FUNCTOR_DECLARE(param_save_cb, void, AP_DroneCAN*, const uint8_t, bool);
    bool handle_param_get_set_response_int(AP_DroneCAN* ap_dronecan, const uint8_t node_id, const char* name, int32_t &value);
    bool handle_param_get_set_response_string(AP_DroneCAN* ap_dronecan, const uint8_t node_id, const char* name, AP_DroneCAN::string &value);
    void handle_param_save_response(AP_DroneCAN* ap_dronecan, const uint8_t node_id, bool success);

    // param enum.  If enum is updated also update _param_names definition in cpp
    enum class Param : uint8_t {
        SingleShot = 0,
        Recording,
        FocusMode,
        SensorMode,
        DigitalZoomMagnification,
        FirmwareVersion,
        Status,
        DateTime,
        OpticalZoomMagnification,
        LAST = OpticalZoomMagnification,            // this should be equal to the final parameter enum
    };
    static const char* _param_names[];              // array of Xacti parameter strings

    // get parameter name for a particular param enum value
    // returns an empty string if not found (which should never happen)
    const char* get_param_name_str(Param param) const;

    // helper function to get and set parameters
    bool set_param_int32(Param param, int32_t param_value);
    bool set_param_string(Param param, const AP_DroneCAN::string& param_value);
    bool get_param_string(Param param);

    // process queue of set parameter items. returns true if set-parameter message was sent
    bool process_set_param_int32_queue();

    // send gimbal control message via DroneCAN
    // mode is 2:angle control or 3:rate control
    // pitch_cd is pitch angle in centi-degrees or pitch rate in cds
    // yaw_cd is angle in centi-degrees or yaw rate in cds
    void send_gimbal_control(uint8_t mode, int16_t pitch_cd, int16_t yaw_cd);

    // send vehicle attitude to gimbal via DroneCAN.  now_ms is current system time
    // returns true if sent so that we avoid immediately trying to also send other messages
    bool send_copter_att_status(uint32_t now_ms);

    // update zoom rate controller.  now_ms is current system time
    // returns true if sent so that we avoid immediately trying to also send other messages
    bool update_zoom_rate_control(uint32_t now_ms);

    // request firmware version.  now_ms is current system time
    // returns true if sent so that we avoid immediately trying to also send other messages
    bool request_firmware_version(uint32_t now_ms);

    // request parameters used to determine camera capabilities.  now_ms is current system time
    // returns true if a param get/set was sent so that we avoid sending other messages
    bool request_capabilities(uint32_t now_ms);

    // set date and time.  now_ms is current system time
    bool set_datetime(uint32_t now_ms);

    // request status.  now_ms is current system time
    // returns true if sent so that we avoid immediately trying to also send other messages
    bool request_status(uint32_t now_ms);

    // check if safe to send message (if messages sent too often camera will not respond)
    // now_ms is current system time
    bool is_safe_to_send(uint32_t now_ms) const;

    // internal variables
    bool _initialised;                              // true once the driver has been initialised

    // attitude received from gimbal
    Quaternion _current_attitude_quat;              // current attitude as a quaternion
    uint32_t _last_current_attitude_quat_ms;        // system time _current_angle_rad was updated
    bool _recording_video;                          // true if recording video
    uint16_t _last_digital_zoom_param_value = 100;  // last digital zoom parameter value sent to camera.  100 ~ 1000 (interval 100)
    uint16_t _last_optical_zoom_param_value = 100;  // last optical zoom parameter value sent to camera.  100 ~ 250 (interval 10)
    struct {
        bool enabled;                               // true if zoom rate control is enabled
        int8_t dir;                                 // zoom direction (-1 to zoom out, +1 to zoom in)
        uint32_t last_update_ms;                    // system time that zoom rate control last updated zoom
    } _zoom_rate_control;

    // firmware version received from gimbal
    struct {
        uint32_t last_request_ms;                   // system time of last request for firmware version
        bool received;                              // true once firmware version has been received
        char str[12] {};                            // firmware version string (11 bytes + 1 null byte)
        uint32_t mav_ver;                           // version formatted for reporting to GCS via CAMERA_INFORMATION message
    } _firmware_version;

    // date and time handling
    struct {
        uint32_t last_request_ms;                   // system time that date/time was last requested
        bool set;                                   // true once date/time has been set
    } _datetime;

    // capability handling
    enum class Capability : uint8_t {
        False = 0,
        True = 1,
        Unknown = 2,
    };
    struct {
        bool received;                              // true if we have determined cameras capabilities
        uint32_t first_request_ms;                  // system time of first request for capabilities (used to timeout)
        uint32_t last_request_ms;                   // system time of last capability related parameter check
        Capability optical_zoom;                    // Yes if camera has optical zoom
    } capabilities = {false, 0, 0, Capability::Unknown};

    // gimbal status handling
    enum class ErrorStatus : uint32_t {
        TAKING_PICTURE = 0x04,                      // currently taking a picture
        RECORDING_VIDEO = 0x08,                     // currently recording video
        CANNOT_TAKE_PIC = 0x20,
        TIME_NOT_SET = 0x10000,
        MEDIA_ERROR = 0x20000,
        LENS_ERROR = 0x40000,
        MOTOR_INIT_ERROR = 0x100000,
        MOTOR_OPERATION_ERROR = 0x200000,
        GIMBAL_CONTROL_ERROR = 0x400000,
        TEMP_WARNING = 0x1000000
    };
    struct {
        uint32_t error_status;                      // see ErrorStatus enum
        uint32_t video_remain_time;                 // max seconds of video that may be recorded before SD card is full
        uint32_t photo_remain_count;                // max number of pics before SD card is full
        uint32_t sd_card_size_mb;                   // SD card size in MB
        uint32_t sd_card_free_mb;                   // SD card remaining size in MB
        uint16_t body;                              // body type
        uint16_t cmos;                              // cmos type
        uint16_t gimbal_pitch;                      // gimbal pitch angle
        uint16_t gimbal_roll;                       // gimbal roll angle
        uint16_t gimbal_yaw;                        // gimbal yaw angle
        uint16_t reserved1;
        uint8_t date_time[7];                       // camera's date and time
        uint8_t reserved2;
        uint32_t exposure_time_us;                  // camera's exposure time in us
        uint16_t apeture;                           // cameras' aperture * 100
        uint16_t iso_sensitivity;                   // camera's iso sensitivity
    } _status;                                      // latest status received
    static_assert(sizeof(_status) == 48, "status must be 48 bytes");           // status should be 48 bytes
    struct {
        uint32_t last_request_ms;                   // system time that status was last requested
        uint32_t last_error_status;                 // last error status reported to user
    } _status_report;
    bool _motor_error;                              // true if status reports motor or control error (used for health reporting)
    bool _camera_error;                             // true if status reports camera error

    // DroneCAN related variables
    static struct DetectedModules {
        AP_Mount_Xacti *driver;                     // pointer to Xacti backends
        AP_DroneCAN* ap_dronecan;                   // DroneCAN interface used by this backend
        uint8_t node_id;                            // DroneCAN node id associated by this backend
    } _detected_modules[AP_MOUNT_MAX_INSTANCES];
    static HAL_Semaphore _sem_registry;             // semaphore protecting access to _detected_modules table
    uint32_t last_send_gimbal_control_ms;           // system time that send_gimbal_control was last called (used to slow down sends to 5hz)
    uint32_t last_send_copter_att_status_ms;        // system time that send_copter_att_status was last called (used to slow down sends to 10hz)
    uint32_t last_send_getset_param_ms;             // system time that a get or set parameter message was sent

    // queue of set parameter int32 items.  set-parameter requests to camera are throttled to improve reliability
    struct SetParamQueueItem {
        Param param;                                // parameter (name)
        int32_t value;                              // parameter value
    };
    ObjectArray<SetParamQueueItem> *_set_param_int32_queue; // queue of set-parameter items
};

#endif // HAL_MOUNT_XACTI_ENABLED
